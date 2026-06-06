"""
DreamZero robot client (RoboCOIN side).

Forked from ``robot_client_openpi.py`` so it reuses all of the battle-tested
robot hardware glue (camera capture, Realman arm control, the control loop,
frequency pacing, keyboard stop, video recording). The ONLY thing that differs
is the wire schema: instead of OpenPI's dual-namespace observation dict, this
client speaks a small, explicit DreamZero-native schema.

Why a separate client: the DreamZero policy is trained on three synchronized
camera views (``nominal`` / ``purturbated_c1`` / ``purturbated_c2``) and an
8-D state (7 joint angles + 1 gripper). Keeping the schema explicit here is the
single best defense against the three silent rollout bugs (normalization stats,
relative-action decoding, and view ordering) -- this client sends raw robot
units and named views, and the DreamZero server owns all the risky conversions.

Transport: we reuse OpenPI's ``WebsocketClientPolicy`` purely as a content-
agnostic websocket+msgpack pipe. We are NOT compatible with OpenPI policy
servers; we talk only to the DreamZero server, which agrees on the schema below.

=========================== WIRE SCHEMA ===========================
Request (client -> server), one dict per control step::

    {
        "video.nominal_image":        np.uint8 (H, W, 3),   # raw camera frame
        "video.purturbated_c1_image": np.uint8 (H, W, 3),
        "video.purturbated_c2_image": np.uint8 (H, W, 3),
        "state.joint_pos":            np.float (7,),  # joint angles, RADIANS (model units)
        "state.gripper_pos":          np.float (1,),  # gripper, raw robot value
        "prompt":                     str,            # task description
    }

Response (server -> client)::

    {
        "actions":       np.float (N, 8),  # [7 joint RADIANS, 1 gripper], model units
        "server_timing": {...},            # optional, for logging
    }

The server is responsible for: resizing + assembling the 2x2 training grid,
normalization, model inference, and relative-action decoding. The RoboCOIN
Realman robot class already converts hardware degrees <-> radians, so this
client relays RADIANS (model units) in and out -- no unit conversion here.
===================================================================

The three configured ``camera_keys`` are mapped, in order, to
``nominal`` / ``purturbated_c1`` / ``purturbated_c2``. Configure the cameras in
the SAME physical order they were recorded during data collection.

Example command (3 physical cameras + Realman arm)::

    python src/lerobot/scripts/server/robot_client_dreamzero.py \
        --host="192.168.1.166" \
        --port=8000 \
        --task="Pick up all silver cube-like objects with cables, and place them into the cardboard box" \
        --robot.type=realman \
        --robot.ip="192.168.1.17" \
        --robot.port=8080 \
        --robot.block=False \
        --robot.use_canfd=True \
        --robot.canfd_follow=False \
        --robot.velocity=70 \
        --robot.joint_cmd_threshold_deg=0.1 \
        --robot.cameras="{ observation.images.nominal_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, observation.images.purturbated_c1_image: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}, observation.images.purturbated_c2_image: {type: intelrealsense, serial_number_or_name: \"243322073824\", width: 640, height: 480, fps: 30} }" \
        --camera_keys="[ observation.images.nominal_image, observation.images.purturbated_c1_image, observation.images.purturbated_c2_image ]" \
        --robot.id=rm75_follower \
        --frequency=10
"""

from collections import deque
from importlib.util import find_spec

if find_spec("openpi_client") is None:
    raise ImportError("openpi_client is not installed. Please install it via `pip install openpi-client`.")

import draccus
import imageio
import numpy as np
import os
import time
import threading
import traceback
from dataclasses import dataclass, field
from typing import List
from sshkeyboard import listen_keyboard, stop_listening

from openpi_client.websocket_client_policy import WebsocketClientPolicy

import sys
sys.path.append('src/')

from lerobot.cameras.dummy.configuration_dummy import DummyCameraConfig
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.robots import (
    bi_dummy,
    bi_piper,
    bi_realman,
    dummy,
    piper,
    realman,
)
from lerobot.scripts.server.helpers import get_logger


# Server-side view names, in the order the three configured cameras map to.
DREAMZERO_VIEW_KEYS = [
    "video.nominal_image",
    "video.purturbated_c1_image",
    "video.purturbated_c2_image",
]


@dataclass
class DreamZeroRobotClientConfig:
    robot: RobotConfig

    host: str = "127.0.0.1"
    port: int = 8000
    frequency: int = 10
    timing_log_interval: int = 10
    timing_log_window: int = 20
    task: str = "do something"

    result_dir: str = "results/"
    # The three camera keys, in the order they map to nominal / c1 / c2.
    camera_keys: List[str] = field(default_factory=lambda: [
        'observation.images.nominal_image',
        'observation.images.purturbated_c1_image',
        'observation.images.purturbated_c2_image',
    ])
    fps: int = 10


class VideoRecorder:
    def __init__(
        self,
        save_dir,
        fps: int = 30,
    ):
        self.save_dir = save_dir
        self.fps = fps
        self._frames = []

        os.makedirs(self.save_dir, exist_ok=True)

    def add(self, frame):
        if isinstance(frame, list):
            # [(H, W, C), ...] -> (H, W * N, C)
            frame = np.concatenate(frame, axis=1)
        self._frames.append(frame)

    def save(self, task, success):
        save_path = os.path.join(self.save_dir, f"{task.replace('.', '')}_{'success' if success else 'failed'}_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
        print(f'Saving video to {save_path}...')
        imageio.mimwrite(save_path, self._frames, fps=self.fps)
        self._frames = []


class KeyboardListener:
    def __init__(self):
        self._listener = threading.Thread(target=listen_keyboard, args=(self._on_press,))
        self._listener.daemon = True

        self._quit = False
        self._success = None

    def listen(self):
        self._listener.start()

    def reset(self):
        self._quit = False
        self._success = None

    def _on_press(self, key):
        if key == 'q':
            self._quit = True

        elif key == 'y':
            self._success = True
            stop_listening()

        elif key == 'n':
            self._success = False
            stop_listening()


class DreamZeroRobotClient:
    def __init__(self, config: DreamZeroRobotClientConfig):
        self.config = config
        self.logger = get_logger('dreamzero_robot_client')

        if len(config.camera_keys) != len(DREAMZERO_VIEW_KEYS):
            raise ValueError(
                f"DreamZero expects exactly {len(DREAMZERO_VIEW_KEYS)} camera views "
                f"({DREAMZERO_VIEW_KEYS}), but got {len(config.camera_keys)} camera_keys: "
                f"{config.camera_keys}. Configure the 3 physical cameras in collection order."
            )

        self.video_recorder = VideoRecorder(config.result_dir, fps=config.fps)
        self.keyboard_listener = KeyboardListener()

        self.policy = WebsocketClientPolicy(config.host, config.port)
        self.logger.info(f'Connected to DreamZero server at {config.host}:{config.port}')

        self.robot = make_robot_from_config(config.robot)
        self.logger.info(f'Initialized robot: {self.robot.name}')

        self._loop_idx = 0
        self._timing_history: dict[str, deque[float]] = {}
        self._is_finished = False

    def start(self):
        self.keyboard_listener.listen()
        self.logger.info('Starting robot client...')
        # Start the rollout from the robot's CURRENT pose -- do NOT drive the
        # arm to a preset init_state on connect. base_robot.connect() only moves
        # when init_type is 'joint' or 'end_effector'; any other value skips the
        # move and simply records the current joint state as the init anchor.
        # This mimics the openpi client (the policy takes over from wherever the
        # arm already is), and avoids the "weird initial pose" jump the default
        # init_state ([0,0,0,0]) would otherwise cause.
        if getattr(self.robot, "config", None) is not None:
            prev_init_type = getattr(self.robot.config, "init_type", None)
            self.robot.config.init_type = 'none'
            self.logger.info(
                f'Overriding robot init_type {prev_init_type!r} -> \'none\': '
                f'starting in place (no move-to-init on connect).'
            )
        self.robot.connect()

    def control_loop(self):
        while not self._is_finished:
            loop_start = time.perf_counter()

            stage_start = time.perf_counter()
            raw_observation = self.robot.get_observation()
            observation_ms = (time.perf_counter() - stage_start) * 1000

            stage_start = time.perf_counter()
            obs = self._prepare_observation(raw_observation)
            prepare_observation_ms = (time.perf_counter() - stage_start) * 1000

            self.logger.info(f'Prompt: {obs["prompt"]}')

            stage_start = time.perf_counter()
            response = self.policy.infer(obs)
            infer_ms = (time.perf_counter() - stage_start) * 1000

            stage_start = time.perf_counter()
            actions = self._extract_actions(response)
            extract_actions_ms = (time.perf_counter() - stage_start) * 1000

            prepare_action_ms = 0.0
            send_action_ms = 0.0
            after_action_ms = 0.0
            for action in actions:
                stage_start = time.perf_counter()
                action = self._prepare_action(action)
                prepare_action_ms += (time.perf_counter() - stage_start) * 1000
                self.logger.info(f'Received action: {action}')

                stage_start = time.perf_counter()
                self.robot.send_action(action)
                send_action_ms += (time.perf_counter() - stage_start) * 1000

                stage_start = time.perf_counter()
                self._after_action()
                after_action_ms += (time.perf_counter() - stage_start) * 1000

            active_loop_ms = (time.perf_counter() - loop_start) * 1000

            sleep_s = 0.0
            if self.config.frequency > 0:
                sleep_s = 1 / self.config.frequency
                time.sleep(sleep_s)
            total_loop_ms = (time.perf_counter() - loop_start) * 1000

            self._loop_idx += 1
            self._record_loop_timing(
                observation_ms=observation_ms,
                prepare_observation_ms=prepare_observation_ms,
                infer_ms=infer_ms,
                extract_actions_ms=extract_actions_ms,
                prepare_action_ms=prepare_action_ms,
                send_action_ms=send_action_ms,
                after_action_ms=after_action_ms,
                active_loop_ms=active_loop_ms,
                sleep_ms=sleep_s * 1000,
                total_loop_ms=total_loop_ms,
                actions_per_response=float(len(actions)),
            )
            self._maybe_log_loop_timing(response)

    def stop(self):
        self.logger.info('Stopping robot client...')
        self.robot.disconnect()

    def _prepare_observation(self, observation):
        """Build the DreamZero-native observation dict.

        Pull the 8-D state (7 joint angles in RADIANS + gripper) from the robot's
        motor features, and the 3 named views from the configured camera_keys.
        get_observation() already returns model units (radians); we relay as-is.
        """
        # State: iterate the robot's motor feature keys in order (joint_*_pos, gripper_pos).
        state = []
        for key in self.robot._motors_ft.keys():
            assert key in observation, f"Expected key {key} in observation, but got {list(observation.keys())}"
            state.append(observation[key])
        state = np.asarray(state, dtype=np.float32)
        joint_pos = state[:-1].copy()           # (7,) joint angles, degrees
        gripper_pos = state[-1:].copy()          # (1,) gripper, raw robot value

        obs = {
            "state.joint_pos": joint_pos,
            "state.gripper_pos": gripper_pos,
            "prompt": self.config.task,
        }

        # Views: map the 3 configured cameras (in order) to nominal / c1 / c2.
        for cam_key, view_key in zip(self.config.camera_keys, DREAMZERO_VIEW_KEYS):
            resolved = self._resolve_camera_key(observation, cam_key)
            obs[view_key] = np.asarray(observation[resolved])

        return obs

    def _resolve_camera_key(self, observation, cam_key):
        """Resolve a configured camera key against the robot's observation dict.

        The robot's get_observation() keys frames by the camera dict names, which
        in practice are the bare view names (e.g. 'nominal_image'), even when
        camera_keys is configured in the longer 'observation.images.<name>' form.
        Match the exact key first, then fall back to the basename (last '.'-
        segment) so both styles work without forcing a specific camera-config
        naming convention.
        """
        if cam_key in observation:
            return cam_key
        base = cam_key.rsplit('.', 1)[-1]
        if base in observation:
            return base
        raise KeyError(
            f"Camera key {cam_key!r} (basename {base!r}) not found in observation; "
            f"available keys: {list(observation.keys())}"
        )

    def _prepare_action(self, action):
        """Map an (8,) action row to the robot's action feature dict.

        The server returns model units ([7 joint RADIANS, 1 gripper]); the robot's
        send_action() converts radians->degrees for the SDK. We keep the gripper
        compatible with both binary (0/1) and 0-1000 position commands.
        """
        assert len(action) == len(self.robot.action_features), \
            f"Action length {len(action)} does not match expected {len(self.robot.action_features)}: {list(self.robot.action_features.keys())}"
        # np.array() (not asarray) forces a writable copy: the action row comes
        # from the msgpack-deserialized server response, whose backing buffer is
        # read-only, so the in-place gripper assignment below would otherwise
        # raise "assignment destination is read-only".
        action = np.array(action)

        if action[-1] > 1.5:
            action[-1] = np.clip(action[-1], 0, 1000)
        else:
            action[-1] = 1.0 if action[-1] > 0.5 else 0.0

        return {key: action[i].item() for i, key in enumerate(self.robot.action_features.keys())}

    def _extract_actions(self, response):
        if isinstance(response, dict):
            if "actions" in response:
                actions = response["actions"]
            elif "action" in response:
                actions = response["action"]
            elif "output" in response and isinstance(response["output"], dict):
                output = response["output"]
                if "actions" in output:
                    actions = output["actions"]
                elif "action" in output:
                    actions = output["action"]
                else:
                    raise KeyError(
                        f"Expected action payload in response['output'], but got keys: {list(output.keys())}"
                    )
            else:
                raise KeyError(f"Expected 'actions' or 'action' in response, but got keys: {list(response.keys())}")
        else:
            actions = response

        actions = np.asarray(actions)
        if actions.ndim == 1:
            actions = actions[None, :]
        return actions

    def _record_loop_timing(self, **metrics):
        window = max(1, self.config.timing_log_window)
        for key, value in metrics.items():
            history = self._timing_history.setdefault(key, deque(maxlen=window))
            history.append(float(value))

    def _avg_timing(self, key: str) -> float:
        values = self._timing_history.get(key)
        if not values:
            return 0.0
        return float(np.mean(values))

    def _maybe_log_loop_timing(self, response):
        if self.config.timing_log_interval <= 0:
            return
        if self._loop_idx % self.config.timing_log_interval != 0:
            return

        server_timing = response.get("server_timing", {}) if isinstance(response, dict) else {}
        server_infer_ms = server_timing.get("infer_ms", None)
        server_prev_total_ms = server_timing.get("prev_total_ms", None)

        total_loop_ms = self._avg_timing("total_loop_ms")
        active_loop_ms = self._avg_timing("active_loop_ms")
        total_hz = 1000.0 / total_loop_ms if total_loop_ms > 0 else 0.0
        active_hz = 1000.0 / active_loop_ms if active_loop_ms > 0 else 0.0

        msg = (
            f"LoopTiming[{self._loop_idx}] avg/{max(1, self.config.timing_log_window)} | "
            f"actions={self._avg_timing('actions_per_response'):.2f} | "
            f"obs={self._avg_timing('observation_ms'):.1f}ms | "
            f"prep_obs={self._avg_timing('prepare_observation_ms'):.1f}ms | "
            f"infer={self._avg_timing('infer_ms'):.1f}ms | "
            f"extract={self._avg_timing('extract_actions_ms'):.1f}ms | "
            f"prep_act={self._avg_timing('prepare_action_ms'):.1f}ms | "
            f"send={self._avg_timing('send_action_ms'):.1f}ms | "
            f"after={self._avg_timing('after_action_ms'):.1f}ms | "
            f"sleep={self._avg_timing('sleep_ms'):.1f}ms | "
            f"active={active_loop_ms:.1f}ms ({active_hz:.2f}Hz) | "
            f"total={total_loop_ms:.1f}ms ({total_hz:.2f}Hz)"
        )
        if server_infer_ms is not None:
            msg += f" | server_infer={float(server_infer_ms):.1f}ms"
        if server_prev_total_ms is not None:
            msg += f" | server_prev_total={float(server_prev_total_ms):.1f}ms"
        self.logger.info(msg)

    def _after_action(self):
        obs = self.robot.get_observation()
        frames = [obs[self._resolve_camera_key(obs, key)] for key in self.config.camera_keys]
        self.video_recorder.add(frames)

        if self.keyboard_listener._quit:
            print('Success? (y/n): ', end='', flush=True)
            while self.keyboard_listener._success is None:
                time.sleep(0.1)
            print('Got:', self.keyboard_listener._success)
            self.video_recorder.save(task=self.config.task, success=self.keyboard_listener._success)
            self._is_finished = True


@draccus.wrap()
def main(cfg: DreamZeroRobotClientConfig):
    client = DreamZeroRobotClient(cfg)
    client.start()

    try:
        client.control_loop()
    except KeyboardInterrupt:
        client.stop()
    except Exception as e:
        client.logger.error(f'Error in control loop: {e}')
        client.logger.error(traceback.format_exc())
    finally:
        client.stop()


if __name__ == "__main__":
    main()
