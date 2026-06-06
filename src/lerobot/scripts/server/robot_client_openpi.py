"""
Example command:

1. Dummy robot & dummy policy:

```python
python src/lerobot/scripts/server/robot_client_openpi.py \
    --host="127.0.0.1" \
    --port=18000 \
    --robot.type=bi_dummy_end_effector \
    --robot.cameras="{ observation.images.cam_high: {type: dummy, width: 640, height: 480, fps: 5},observation.images.cam_left_wrist: {type: dummy, width: 640, height: 480, fps: 5},observation.images.cam_right_wrist: {type: dummy, width: 640, height: 480, fps: 5}}" \
    --robot.id=black \
    --robot.delta_with="previous" \
    --robot.pose_units="[m, m, m, radian, radian, radian, m]" \
    --robot.model_pose_units="[m, m, m, radian, radian, radian, m]" \
    --task="fold the towel" \
    --fps 10
```

```python
python src/lerobot/scripts/server/robot_client_openpi.py \
    --host="127.0.0.1" \
    --port=18000 \
    --robot.type=bi_realman \
    --robot.cameras="{ observation.images.cam_high: {type: dummy, width: 640, height: 480, fps: 5},observation.images.cam_left_wrist: {type: dummy, width: 640, height: 480, fps: 5},observation.images.cam_right_wrist: {type: dummy, width: 640, height: 480, fps: 5}}" \
    --robot.init_type="joint"
    --robot.init_ee_state="[0, 0, 0, 0, 0, 0, 0]" \
    --robot.base_euler="[0, 0, 0]" \
    --robot.id=black 
```

peach
```python
python src/lerobot/scripts/server/robot_client_openpi.py \
  --host="127.0.0.1"     \
  --port=18000     \
  --task="put peach into basket"    \
  --robot.type=bi_realman_end_effector     \
  --robot.ip_left="169.254.128.18"    \
  --robot.port_left=8080     \
  --robot.ip_right="169.254.128.19"     \
  --robot.port_right=8080     \
  --robot.block=False \
  --robot.cameras="{ observation.images.cam_high: {type: opencv, index_or_path: 8, width: 640, height: 480, fps: 30}, observation.images.cam_left_wrist: {type: opencv, index_or_path: 14, width: 640, height: 480, fps: 30},observation.images.cam_right_wrist: {type: opencv, index_or_path: 20, width: 640, height: 480, fps: 30}}"     \
  --robot.init_type="joint"     \
  --robot.id=black    \
  --robot.delta_with=previous
```

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


@dataclass
class OpenPIRobotClientConfig:
    robot: RobotConfig

    host: str = "127.0.0.1"
    port: int = 18000
    frequency: int = 10
    timing_log_interval: int = 10
    timing_log_window: int = 20
    task: str = "do something"

    result_dir: str = "results/"
    camera_keys: List[str] = field(default_factory=lambda: [
        'observation.images.cam_high', 'observation.images.cam_left_wrist', 'observation.images.cam_right_wrist'
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


class OpenPIRobotClient:
    def __init__(self, config: OpenPIRobotClientConfig):
        self.config = config
        self.logger = get_logger('openpi_robot_client')

        self.video_recorder = VideoRecorder(config.result_dir, fps=config.fps)
        self.keyboard_listener = KeyboardListener()

        self.policy = WebsocketClientPolicy(config.host, config.port)
        self.logger.info(f'Connected to OpenPI server at {config.host}:{config.port}')

        self.robot = make_robot_from_config(config.robot)
        self.logger.info(f'Initialized robot: {self.robot.name}')

        self._loop_idx = 0
        self._timing_history: dict[str, deque[float]] = {}
        self._is_finished = False
    
    def start(self):
        self.keyboard_listener.listen()
        self.logger.info('Starting robot client...')
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

            # self.logger.info(f'Sent observation: {list(obs.keys())}')
            self.logger.info(f'Prompt: {obs["prompt"]}')

            stage_start = time.perf_counter()
            response = self.policy.infer(obs)
            infer_ms = (time.perf_counter() - stage_start) * 1000

            stage_start = time.perf_counter()
            actions = self._extract_actions(response)
            # actions = actions[2:5]
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
        state = []
        for key in self.robot._motors_ft.keys():
            assert key in observation, f"Expected key {key} in observation, but got {observation.keys()}"
            state.append(observation[key])
            observation.pop(key)
        
        state = np.array(state)
        joint_position = state[:-1].copy()
        gripper_position = np.array([state[-1]], dtype=state.dtype)
        joint_velocity = np.zeros_like(joint_position)

        # Emit both RoboCOIN native state and legacy split joint fields.
        observation['observation.state'] = state
        observation['observation.joint_position'] = joint_position
        observation['observation.joint_velocity'] = joint_velocity
        observation['observation.gripper_position'] = gripper_position
        # Keep both dot and slash observation namespaces for OpenPI server variants.
        for key, value in list(observation.items()):
            if key.startswith("observation."):
                slash_key = key.replace("observation.", "observation/", 1)
                observation.setdefault(slash_key, value)
        observation['prompt'] = self.config.task
        return observation
    
    def _prepare_action(self, action):
        assert len(action) == len(self.robot.action_features), \
            f"Action length {len(action)} does not match expected {len(self.robot.action_features)}: {self.robot.action_features.keys()}"
        action = np.array(action)

        # Keep the final gripper channel compatible with both legacy 0-1000 commands
        # and newer binary 0/1 labels used in RM75 collection.
        if action[-1] > 1.5:
            action[-1] = np.clip(action[-1], 0, 1000)
        else:
            action[-1] = 1.0 if action[-1] > 0.5 else 0.0

        return {key: action[i].item() for i, key in enumerate(self.robot.action_features.keys())}

    def _extract_actions(self, response):
        if isinstance(response, dict):
            if "action" in response:
                actions = response["action"]
            elif "actions" in response:
                actions = response["actions"]
            elif "output" in response and isinstance(response["output"], dict):
                output = response["output"]
                if "action" in output:
                    actions = output["action"]
                elif "actions" in output:
                    actions = output["actions"]
                else:
                    raise KeyError(
                        f"Expected action payload in response['output'], but got keys: {list(output.keys())}"
                    )
            else:
                raise KeyError(f"Expected 'action' or 'actions' in response, but got keys: {list(response.keys())}")
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
        frames = [obs[key] for key in self.config.camera_keys]
        self.video_recorder.add(frames)

        if self.keyboard_listener._quit:
            print('Success? (y/n): ', end='', flush=True)
            while self.keyboard_listener._success is None:
                time.sleep(0.1)
            print('Got:', self.keyboard_listener._success)
            self.video_recorder.save(task=self.config.task, success=self.keyboard_listener._success)
            self._is_finished = True


@draccus.wrap()
def main(cfg: OpenPIRobotClientConfig):
    client = OpenPIRobotClient(cfg)
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
