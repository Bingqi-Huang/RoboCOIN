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

import platform
import time


def busy_wait(seconds):
    if seconds <= 0:
        return

    end_time = time.perf_counter() + seconds
    if platform.system() == "Darwin" or platform.system() == "Windows":
        # On Mac and Windows, `time.sleep` is not accurate and we need to use this while loop trick,
        # but it consumes CPU cycles.
        while time.perf_counter() < end_time:
            pass
    else:
        # Hybrid wait on Linux: sleep most of the interval, then spin for the final ~1ms
        # to reduce consistent overshoot from scheduler wake-up latency.
        spin_window_s = 0.001
        sleep_s = seconds - spin_window_s
        if sleep_s > 0:
            time.sleep(sleep_s)
        while time.perf_counter() < end_time:
            pass


def safe_disconnect(func):
    # TODO(aliberts): Allow to pass custom exceptions
    # (e.g. ThreadServiceExit, KeyboardInterrupt, SystemExit, UnpluggedError, DynamixelCommError)
    def wrapper(robot, *args, **kwargs):
        try:
            return func(robot, *args, **kwargs)
        except Exception as e:
            if robot.is_connected:
                robot.disconnect()
            raise e

    return wrapper
