#!/usr/bin/env python3


'''
uv run src/lerobot/scripts/server/robot_client_lunarpressure.py \
  --host="192.168.1.166" \
  --port=8001 \
  --task="Keep Line-A pressure at 0.1 MPa" \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.block=False \
  --robot.init_type="none" \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.wait_second=0.0 \
  --robot.joint_cmd_threshold_deg=0.05 \
  --robot.velocity=100 \
  --robot.cameras="{ observation.scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, observation.wrist_image: {type: intelrealsense, serial_number_or_name: \"243322073824\", width: 640, height: 480, fps: 30}}" \
  --camera_keys="[ observation.scene_image, observation.wrist_image ]" \
  --robot.id=rm75_follower \
  --frequency=30 \
  --timing_log_interval=5 \
  --timing_log_window=10 \
  --action_chunk_mode=first

'''


from __future__ import annotations

import time
import traceback
from collections import deque
from dataclasses import dataclass, field
from typing import Any, List

import draccus
import numpy as np
from openpi_client.websocket_client_policy import WebsocketClientPolicy

import sys

sys.path.append("src/")

from lerobot.cameras.dummy.configuration_dummy import DummyCameraConfig  # noqa: F401,E402
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401,E402
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401,E402
from lerobot.robots import bi_dummy, bi_piper, bi_realman, dummy, piper, realman  # noqa: F401,E402
from lerobot.robots.config import RobotConfig  # noqa: E402
from lerobot.robots.utils import make_robot_from_config  # noqa: E402
from lerobot.scripts.server.helpers import get_logger  # noqa: E402


@dataclass
class LunarPressureRobotClientConfig:
    robot: RobotConfig

    host: str = "127.0.0.1"
    port: int = 8001
    frequency: int = 30
    task: str = "Keep Line-A pressure at 0.1 MPa"

    timing_log_interval: int = 10
    timing_log_window: int = 20

    result_dir: str = "results/"  # accepted for command compatibility; not used
    camera_keys: List[str] = field(
        default_factory=lambda: ["observation.scene_image", "observation.wrist_image"]
    )
    fps: int = 30  # accepted for command compatibility; ACT pacing uses frequency

    action_chunk_mode: str = "first"  # first | all | paced
    wait_default_seconds: float = 1.0
    stop_on_lunar_control_stop: bool = True
    no_action_modes: List[str] = field(
        default_factory=lambda: ["wait", "settle", "no_action", "observe_wait", "sleep"]
    )


class LunarPressureRobotClient:
    """RoboCOIN-compatible client with LunarPressure control modes.

    The stock OpenPI client assumes every server response contains an action.
    LunarPressure also needs wait/no-action phases for slow Gemini OBSERVE and
    mechanical SETTLE. In those modes this client sleeps and does not call
    robot.send_action(). ACT responses still use normal action/action chunks.
    """

    def __init__(self, config: LunarPressureRobotClientConfig):
        self.config = config
        self.logger = get_logger("lunarpressure_robot_client")
        self.policy = WebsocketClientPolicy(config.host, config.port)
        self.logger.info("Connected to LunarPressure server at %s:%s", config.host, config.port)
        self.logger.info("Server metadata: %s", self.policy.get_server_metadata())

        self.robot = make_robot_from_config(config.robot)
        self.logger.info("Initialized robot: %s", self.robot.name)

        self._loop_idx = 0
        self._timing_history: dict[str, deque[float]] = {}
        self._is_finished = False

    def start(self) -> None:
        self.logger.info("Starting robot client...")
        self.robot.connect()

    def stop(self) -> None:
        self.logger.info("Stopping robot client...")
        self.robot.disconnect()

    def control_loop(self) -> None:
        while not self._is_finished:
            loop_start = time.perf_counter()

            stage_start = time.perf_counter()
            raw_observation = self.robot.get_observation()
            observation_ms = elapsed_ms(stage_start)

            stage_start = time.perf_counter()
            obs = self._prepare_observation(raw_observation)
            prepare_observation_ms = elapsed_ms(stage_start)

            stage_start = time.perf_counter()
            response = self.policy.infer(obs)
            infer_ms = elapsed_ms(stage_start)

            stage_start = time.perf_counter()
            decision = self._interpret_response(response)
            interpret_ms = elapsed_ms(stage_start)

            prepare_action_ms = 0.0
            send_action_ms = 0.0
            wait_ms = 0.0
            actions_sent = 0

            if decision.wait_seconds is not None:
                self.logger.info(
                    "LunarPressure mode=%s: wait %.3fs without send_action (%s)",
                    decision.mode,
                    decision.wait_seconds,
                    decision.reason,
                )
                stage_start = time.perf_counter()
                time.sleep(max(0.0, decision.wait_seconds))
                wait_ms = elapsed_ms(stage_start)
            elif decision.actions is not None:
                actions = self._select_actions(decision.actions)
                for index, action in enumerate(actions):
                    stage_start = time.perf_counter()
                    prepared = self._prepare_action(action)
                    prepare_action_ms += elapsed_ms(stage_start)

                    self.logger.info("Sending action mode=%s action=%s", decision.mode, prepared)
                    stage_start = time.perf_counter()
                    self.robot.send_action(prepared)
                    send_action_ms += elapsed_ms(stage_start)
                    actions_sent += 1

                    if self.config.action_chunk_mode == "paced" and index < len(actions) - 1:
                        time.sleep(self._target_period_s())
            elif decision.finish:
                self.logger.info("LunarPressure requested terminal stop: %s", decision.reason)
                self._is_finished = True
            else:
                raise RuntimeError(f"Server response had no action and no recognized lunar_control mode: {response}")

            active_loop_ms = elapsed_ms(loop_start)
            pace_sleep_ms = 0.0
            if decision.wait_seconds is None and not decision.finish:
                pace_sleep_ms = self._sleep_to_frequency(loop_start) * 1000
            total_loop_ms = elapsed_ms(loop_start)

            self._loop_idx += 1
            self._record_loop_timing(
                observation_ms=observation_ms,
                prepare_observation_ms=prepare_observation_ms,
                infer_ms=infer_ms,
                interpret_ms=interpret_ms,
                prepare_action_ms=prepare_action_ms,
                send_action_ms=send_action_ms,
                wait_ms=wait_ms,
                pace_sleep_ms=pace_sleep_ms,
                active_loop_ms=active_loop_ms,
                total_loop_ms=total_loop_ms,
                actions_sent=float(actions_sent),
            )
            self._maybe_log_loop_timing(response, decision)

    def _prepare_observation(self, observation: dict[str, Any]) -> dict[str, Any]:
        state = []
        for key in self.robot._motors_ft.keys():
            assert key in observation, f"Expected key {key} in observation, but got {observation.keys()}"
            state.append(observation[key])
            observation.pop(key)

        state_array = np.array(state)
        joint_position = state_array[:-1].copy()
        gripper_position = np.array([state_array[-1]], dtype=state_array.dtype)
        joint_velocity = np.zeros_like(joint_position)

        observation["observation.state"] = state_array
        observation["observation.joint_position"] = joint_position
        observation["observation.joint_velocity"] = joint_velocity
        observation["observation.gripper_position"] = gripper_position
        for key, value in list(observation.items()):
            if key.startswith("observation."):
                slash_key = key.replace("observation.", "observation/", 1)
                observation.setdefault(slash_key, value)
        observation["prompt"] = self.config.task
        return observation

    def _interpret_response(self, response: Any) -> "PolicyDecision":
        lunar_control = response.get("lunar_control", {}) if isinstance(response, dict) else {}
        if not isinstance(lunar_control, dict):
            lunar_control = {}
        mode = str(lunar_control.get("mode") or "")
        reason = str(lunar_control.get("reason") or "")

        if self.config.stop_on_lunar_control_stop and bool(lunar_control.get("stop", False)):
            return PolicyDecision(mode=mode or "stop", reason=reason, finish=True)

        if mode in self.config.no_action_modes:
            seconds = lunar_control.get("seconds", lunar_control.get("wait_seconds", self.config.wait_default_seconds))
            return PolicyDecision(mode=mode, reason=reason, wait_seconds=float(seconds))

        actions = extract_actions_or_none(response)
        if actions is not None:
            return PolicyDecision(mode=mode or "action", reason=reason, actions=actions)

        if mode in ("stop_terminal", "done", "close"):
            return PolicyDecision(mode=mode, reason=reason, finish=True)

        return PolicyDecision(mode=mode or "unknown", reason=reason)

    def _select_actions(self, actions: np.ndarray) -> np.ndarray:
        if self.config.action_chunk_mode == "first":
            return actions[:1]
        if self.config.action_chunk_mode in ("all", "paced"):
            return actions
        raise ValueError("action_chunk_mode must be one of: first, all, paced")

    def _prepare_action(self, action: np.ndarray) -> dict[str, Any]:
        assert len(action) == len(self.robot.action_features), (
            f"Action length {len(action)} does not match expected "
            f"{len(self.robot.action_features)}: {self.robot.action_features.keys()}"
        )
        action = np.array(action)
        if action[-1] > 1.5:
            action[-1] = np.clip(action[-1], 0, 1000)
        else:
            action[-1] = 1.0 if action[-1] > 0.5 else 0.0
        return {key: action[i].item() for i, key in enumerate(self.robot.action_features.keys())}

    def _target_period_s(self) -> float:
        if self.config.frequency <= 0:
            return 0.0
        return 1.0 / self.config.frequency

    def _sleep_to_frequency(self, loop_start: float) -> float:
        target = self._target_period_s()
        if target <= 0:
            return 0.0
        remaining = target - (time.perf_counter() - loop_start)
        if remaining > 0:
            time.sleep(remaining)
            return remaining
        return 0.0

    def _record_loop_timing(self, **metrics: float) -> None:
        window = max(1, self.config.timing_log_window)
        for key, value in metrics.items():
            self._timing_history.setdefault(key, deque(maxlen=window)).append(float(value))

    def _avg_timing(self, key: str) -> float:
        values = self._timing_history.get(key)
        if not values:
            return 0.0
        return float(np.mean(values))

    def _maybe_log_loop_timing(self, response: Any, decision: "PolicyDecision") -> None:
        if self.config.timing_log_interval <= 0:
            return
        if self._loop_idx % self.config.timing_log_interval != 0:
            return

        server_timing = response.get("server_timing", {}) if isinstance(response, dict) else {}
        total_loop_ms = self._avg_timing("total_loop_ms")
        active_loop_ms = self._avg_timing("active_loop_ms")
        total_hz = 1000.0 / total_loop_ms if total_loop_ms > 0 else 0.0
        active_hz = 1000.0 / active_loop_ms if active_loop_ms > 0 else 0.0

        self.logger.info(
            "LoopTiming[%s] mode=%s avg/%s | actions=%.2f | obs=%.1fms | "
            "prep_obs=%.1fms | infer=%.1fms | interpret=%.1fms | prep_act=%.1fms | "
            "send=%.1fms | wait=%.1fms | pace_sleep=%.1fms | active=%.1fms (%.2fHz) | "
            "total=%.1fms (%.2fHz) | server=%s",
            self._loop_idx,
            decision.mode,
            max(1, self.config.timing_log_window),
            self._avg_timing("actions_sent"),
            self._avg_timing("observation_ms"),
            self._avg_timing("prepare_observation_ms"),
            self._avg_timing("infer_ms"),
            self._avg_timing("interpret_ms"),
            self._avg_timing("prepare_action_ms"),
            self._avg_timing("send_action_ms"),
            self._avg_timing("wait_ms"),
            self._avg_timing("pace_sleep_ms"),
            active_loop_ms,
            active_hz,
            total_loop_ms,
            total_hz,
            server_timing,
        )


@dataclass
class PolicyDecision:
    mode: str
    reason: str = ""
    actions: np.ndarray | None = None
    wait_seconds: float | None = None
    finish: bool = False


def extract_actions_or_none(response: Any) -> np.ndarray | None:
    if isinstance(response, dict):
        if "action" in response:
            raw = response["action"]
        elif "actions" in response:
            raw = response["actions"]
        elif isinstance(response.get("output"), dict):
            output = response["output"]
            if "action" in output:
                raw = output["action"]
            elif "actions" in output:
                raw = output["actions"]
            else:
                return None
        else:
            return None
    else:
        raw = response

    actions = np.asarray(raw)
    if actions.ndim == 1:
        actions = actions[None, :]
    if actions.ndim != 2:
        raise ValueError(f"Expected action array rank 1 or 2, got shape {actions.shape}")
    return actions


def elapsed_ms(start: float) -> float:
    return (time.perf_counter() - start) * 1000


@draccus.wrap()
def main(cfg: LunarPressureRobotClientConfig) -> None:
    client = LunarPressureRobotClient(cfg)
    client.start()
    try:
        client.control_loop()
    except KeyboardInterrupt:
        client.logger.info("KeyboardInterrupt, stopping")
    except Exception as exc:
        client.logger.error("Error in control loop: %s", exc)
        client.logger.error(traceback.format_exc())
        raise
    finally:
        client.stop()


if __name__ == "__main__":
    main()
