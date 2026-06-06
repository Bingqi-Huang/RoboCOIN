#!/usr/bin/env python3
"""Sidecar keyboard gripper controller for a Realman arm.

Run this in a dedicated terminal. It reads keystrokes from *this* terminal,
commands the leader-arm gripper, and writes a small JSON state file that
RoboCOIN can read to produce a binary gripper action.

Keys:
  c : continuous force-controlled grasp (pick_on)
  o : release / fully open
  r : reset -> open gripper, then movej to reset pose
  s : print current shared state file
  q : quit
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import termios
import time
import tty
from dataclasses import asdict, dataclass

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

#PnP task
RESET_JOINTS_DEG = [0.0, -10.0, 0.0, -53.0, 0.0, -110.0, -240.0]
# Valve task: RESET_JOINTS_DEG = [8.48, -10.754, -44.358, -39.306, 44.202, -50.656, -262.664]

@dataclass
class SharedState:
    command: str  # "open" | "close"
    value: int    # 0=open, 1=close
    timestamp: float
    speed: int
    force: int
    source: str = "keyboard_sidecar"


class RawKeyReader:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key(self) -> str:
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            return ch
        return ch


class GripperSidecar:
    def __init__(
        self,
        ip: str,
        port: int,
        state_file: str,
        close_speed: int,
        close_force: int,
        open_speed: int,
        reset_speed: int,
    ):
        self.ip = ip
        self.port = port
        self.state_file = state_file
        self.close_speed = close_speed
        self.close_force = close_force
        self.open_speed = open_speed
        self.reset_speed = reset_speed

        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = None
        self.current_value = None

    def connect(self) -> None:
        self.handle = self.arm.rm_create_robot_arm(self.ip, self.port)
        if getattr(self.handle, "id", -1) == -1:
            raise RuntimeError(f"Failed to connect to Realman arm at {self.ip}:{self.port}")
        print(f"Connected to leader arm {self.ip}:{self.port} (handle id={self.handle.id})")

    def disconnect(self) -> None:
        try:
            self.arm.rm_delete_robot_arm()
        except Exception:
            pass

    def write_state(self, command: str, value: int, speed: int, force: int) -> None:
        data = SharedState(
            command=command,
            value=value,
            timestamp=time.time(),
            speed=speed,
            force=force,
        )
        tmp = self.state_file + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(asdict(data), f)
        os.replace(tmp, self.state_file)
        self.current_value = value

    def close(self) -> None:
        ret = self.arm.rm_set_gripper_pick_on(self.close_speed, self.close_force, False, 0)
        print(f"[CLOSE] rm_set_gripper_pick_on(speed={self.close_speed}, force={self.close_force}) -> {ret}")
        self.write_state("close", 1, self.close_speed, self.close_force)

    def open(self) -> None:
        ret = self.arm.rm_set_gripper_release(self.open_speed, False, 0)
        print(f"[OPEN ] rm_set_gripper_release(speed={self.open_speed}) -> {ret}")
        self.write_state("open", 0, self.open_speed, self.close_force)

    def reset(self) -> None:
        print("[RESET] Step 1/2: release gripper")
        ret = self.arm.rm_set_gripper_release(self.open_speed, False, 0)
        print(f"[RESET] rm_set_gripper_release(speed={self.open_speed}) -> {ret}")
        self.write_state("open", 0, self.open_speed, self.close_force)

        # 给夹爪一点时间真正张开，避免带物或机构未完全释放就 movej
        time.sleep(0.3)

        print("[RESET] Step 2/2: movej to reset pose (degree)")
        ret = self.arm.rm_movej(
            RESET_JOINTS_DEG,
            self.reset_speed,   # velocity
            0,                  # r
            0,                  # trajectory connect
            True,               # block until finished
        )
        print(f"[RESET] rm_movej(joints_deg={RESET_JOINTS_DEG}, v={self.reset_speed}, block=True) -> {ret}")

    def print_status(self) -> None:
        try:
            ret, state = self.arm.rm_get_gripper_state()
            print(f"[STATE] ret={ret} state={state}")
        except Exception as e:
            print(f"[STATE] read failed: {e}")

        try:
            with open(self.state_file, "r", encoding="utf-8") as f:
                print(f"[FILE ] {f.read()}")
        except FileNotFoundError:
            print(f"[FILE ] {self.state_file} does not exist yet")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--ip", required=True)
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--state-file", default="/tmp/realman_gripper_state.json")
    p.add_argument("--close-speed", type=int, default=500)
    p.add_argument("--close-force", type=int, default=1000)
    p.add_argument("--open-speed", type=int, default=500)
    p.add_argument("--reset-speed", type=int, default=30)
    p.add_argument("--init-open", action="store_true")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    os.makedirs(os.path.dirname(args.state_file), exist_ok=True)

    ctl = GripperSidecar(
        ip=args.ip,
        port=args.port,
        state_file=args.state_file,
        close_speed=args.close_speed,
        close_force=args.close_force,
        open_speed=args.open_speed,
        reset_speed=args.reset_speed,
    )

    def _cleanup(*_):
        print("\nExiting...")
        ctl.disconnect()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    ctl.connect()
    if args.init_open:
        ctl.open()
    else:
        # Ensure a state file exists even before the first keypress.
        ctl.write_state("open", 0, ctl.open_speed, ctl.close_force)

    print("Keys: [c]=close(continuous force grip), [o]=open/release, [r]=reset(open+movej), [s]=status, [q]=quit")
    print(f"Shared state file: {args.state_file}")
    print(f"Reset joints (degree): {RESET_JOINTS_DEG}")

    with RawKeyReader() as reader:
        while True:
            key = reader.read_key().lower()
            if key == "c":
                ctl.close()
            elif key == "o":
                ctl.open()
            elif key == "r":
                ctl.reset()
            elif key == "s":
                ctl.print_status()
            elif key == "q":
                _cleanup()
            else:
                pass


if __name__ == "__main__":
    raise SystemExit(main())