# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

########################################################################################
# Utilities
########################################################################################


import logging
import os
import select
import sys
import threading
import time
import traceback
from contextlib import nullcontext
from copy import copy
from functools import cache

import numpy as np
import torch
from deepdiff import DeepDiff
from termcolor import colored

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import DEFAULT_FEATURES
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot.robots import Robot


def log_control_info(robot: Robot, dt_s, episode_index=None, frame_index=None, fps=None):
    log_items = []
    if episode_index is not None:
        log_items.append(f"ep:{episode_index}")
    if frame_index is not None:
        log_items.append(f"frame:{frame_index}")

    def log_dt(shortname, dt_val_s):
        nonlocal log_items, fps
        info_str = f"{shortname}:{dt_val_s * 1000:5.2f} ({1 / dt_val_s:3.1f}hz)"
        if fps is not None:
            actual_fps = 1 / dt_val_s
            if actual_fps < fps - 1:
                info_str = colored(info_str, "yellow")
        log_items.append(info_str)

    # total step time displayed in milliseconds and its frequency
    log_dt("dt", dt_s)

    # TODO(aliberts): move robot-specific logs logic in robot.print_logs()
    if not robot.robot_type.startswith("stretch"):
        for name in robot.leader_arms:
            key = f"read_leader_{name}_pos_dt_s"
            if key in robot.logs:
                log_dt("dtRlead", robot.logs[key])

        for name in robot.follower_arms:
            key = f"write_follower_{name}_goal_pos_dt_s"
            if key in robot.logs:
                log_dt("dtWfoll", robot.logs[key])

            key = f"read_follower_{name}_pos_dt_s"
            if key in robot.logs:
                log_dt("dtRfoll", robot.logs[key])

        for name in robot.cameras:
            key = f"read_camera_{name}_dt_s"
            if key in robot.logs:
                log_dt(f"dtR{name}", robot.logs[key])

    info_str = " ".join(log_items)
    logging.info(info_str)


@cache
def is_headless():
    """Detects if python is running without a monitor."""
    try:
        import pynput  # noqa

        return False
    except Exception:
        print(
            "Error trying to import pynput. Switching to headless mode. "
            "As a result, the video stream from the cameras won't be shown, "
            "and you won't be able to change the control flow with keyboards. "
            "For more info, see traceback below.\n"
        )
        traceback.print_exc()
        print()
        return True


def predict_action(
    observation: dict[str, np.ndarray],
    policy: PreTrainedPolicy,
    device: torch.device,
    use_amp: bool,
    task: str | None = None,
    robot_type: str | None = None,
):
    observation = copy(observation)
    with (
        torch.inference_mode(),
        torch.autocast(device_type=device.type) if device.type == "cuda" and use_amp else nullcontext(),
    ):
        # Convert to pytorch format: channel first and float32 in [0,1] with batch dimension
        for name in observation:
            observation[name] = torch.from_numpy(observation[name])
            if "image" in name:
                observation[name] = observation[name].type(torch.float32) / 255
                observation[name] = observation[name].permute(2, 0, 1).contiguous()
            observation[name] = observation[name].unsqueeze(0)
            observation[name] = observation[name].to(device)

        observation["task"] = task if task else ""
        observation["robot_type"] = robot_type if robot_type else ""

        # Compute the next action with the policy
        # based on the current observation
        action = policy.select_action(observation)

        # Remove batch dimension
        action = action.squeeze(0)

        # Move to cpu, if not already the case
        action = action.to("cpu")

    return action


def _toggle_pause(events: dict) -> None:
    if not events.get("pause_allowed", False):
        print("Pause is only available during reset loops.")
        return
    events["pause_recording"] = not events["pause_recording"]
    if events["pause_recording"]:
        print("Paused during reset. Press 'p' again to resume.")
    else:
        print("Resumed reset loop.")


class _TerminalKeyListener:
    """TTY key listener for headless environments (ssh/tmux)."""

    def __init__(self, events: dict):
        self.events = events
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        print(
            "Terminal controls enabled: "
            "'s' or RightArrow=save/continue, 'l' or LeftArrow=rerecord, "
            "'q' or Esc=stop, 'p'=pause(reset only)."
        )

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=0.5)

    def _handle_key(self, key: str) -> None:
        if key in ("\x1b[C", "s", "S", "r", "R"):
            print("Save command received. Exiting current loop...")
            self.events["exit_early"] = True
        elif key in ("\x1b[D", "l", "L"):
            print("Rerecord command received. Exiting loop and rerecording the last episode...")
            self.events["rerecord_episode"] = True
            self.events["exit_early"] = True
        elif key in ("\x1b", "q", "Q"):
            print("Stop command received. Stopping data recording...")
            self.events["stop_recording"] = True
            self.events["exit_early"] = True
        elif key in ("p", "P"):
            _toggle_pause(self.events)

    def _run(self):
        if not sys.stdin.isatty():
            logging.warning("stdin is not a TTY; terminal key controls are disabled.")
            return

        try:
            import termios
            import tty
        except Exception:
            logging.warning("termios/tty unavailable; terminal key controls are disabled.")
            return

        fd = sys.stdin.fileno()
        old_attrs = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._stop.is_set():
                ready, _, _ = select.select([fd], [], [], 0.1)
                if not ready:
                    continue

                raw = os.read(fd, 1)
                if not raw:
                    continue

                key = raw.decode(errors="ignore")
                if key == "\x1b":
                    # Decode common escape sequences, e.g. arrows: ESC [ C / ESC [ D
                    seq = key
                    for _ in range(2):
                        ready2, _, _ = select.select([fd], [], [], 0.005)
                        if not ready2:
                            break
                        nxt = os.read(fd, 1)
                        if not nxt:
                            break
                        seq += nxt.decode(errors="ignore")
                    key = seq

                self._handle_key(key)
        except Exception as e:
            logging.warning(f"Terminal key listener stopped due to error: {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)


def init_keyboard_listener():
    # Allow to exit early while recording an episode or resetting the environment,
    # by tapping the right arrow key '->'. This might require a sudo permission
    # to allow your terminal to monitor keyboard events.
    events = {}
    events["exit_early"] = False
    events["rerecord_episode"] = False
    events["stop_recording"] = False
    events["pause_recording"] = False
    # Pause is only allowed in reset loops, not during episode recording.
    events["pause_allowed"] = False

    if is_headless():
        logging.warning(
            "Headless environment detected. Falling back to terminal key controls if TTY is available."
        )
        if sys.stdin.isatty():
            listener = _TerminalKeyListener(events)
            listener.start()
        else:
            logging.warning("stdin is not a TTY. Keyboard control is unavailable in this session.")
            listener = None
        return listener, events

    # Only import pynput if not in a headless environment
    from pynput import keyboard

    def on_press(key):
        try:
            if key == keyboard.Key.right:
                print("Right arrow key pressed. Exiting loop...")
                events["exit_early"] = True
            elif key == keyboard.Key.left:
                print("Left arrow key pressed. Exiting loop and rerecord the last episode...")
                events["rerecord_episode"] = True
                events["exit_early"] = True
            elif key == keyboard.Key.esc:
                print("Escape key pressed. Stopping data recording...")
                events["stop_recording"] = True
                events["exit_early"] = True
            elif hasattr(key, "char") and key.char in ("s", "S", "r", "R"):
                print("Save command received. Exiting current loop...")
                events["exit_early"] = True
            elif hasattr(key, "char") and key.char in ("l", "L"):
                print("Rerecord command received. Exiting loop and rerecording the last episode...")
                events["rerecord_episode"] = True
                events["exit_early"] = True
            elif hasattr(key, "char") and key.char in ("q", "Q"):
                print("Stop command received. Stopping data recording...")
                events["stop_recording"] = True
                events["exit_early"] = True
            elif hasattr(key, "char") and key.char in ("p", "P"):
                _toggle_pause(events)
        except Exception as e:
            print(f"Error handling key press: {e}")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    return listener, events


def sanity_check_dataset_name(repo_id, policy_cfg):
    _, dataset_name = repo_id.split("/")
    # either repo_id doesnt start with "eval_" and there is no policy
    # or repo_id starts with "eval_" and there is a policy

    # Check if dataset_name starts with "eval_" but policy is missing
    if dataset_name.startswith("eval_") and policy_cfg is None:
        raise ValueError(
            f"Your dataset name begins with 'eval_' ({dataset_name}), but no policy is provided ({policy_cfg.type})."
        )

    # Check if dataset_name does not start with "eval_" but policy is provided
    if not dataset_name.startswith("eval_") and policy_cfg is not None:
        raise ValueError(
            f"Your dataset name does not begin with 'eval_' ({dataset_name}), but a policy is provided ({policy_cfg.type})."
        )


def sanity_check_dataset_robot_compatibility(
    dataset: LeRobotDataset, robot: Robot, fps: int, features: dict
) -> None:
    fields = [
        ("robot_type", dataset.meta.robot_type, robot.robot_type),
        ("fps", dataset.fps, fps),
        ("features", dataset.features, {**features, **DEFAULT_FEATURES}),
    ]

    mismatches = []
    for field, dataset_value, present_value in fields:
        diff = DeepDiff(dataset_value, present_value, exclude_regex_paths=[r".*\['info'\]$"])
        if diff:
            mismatches.append(f"{field}: expected {present_value}, got {dataset_value}")

    if mismatches:
        raise ValueError(
            "Dataset metadata compatibility check failed with mismatches:\n" + "\n".join(mismatches)
        )
