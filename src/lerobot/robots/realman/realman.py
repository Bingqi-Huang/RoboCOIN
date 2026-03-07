"""
Realman robot implementation.
"""

from importlib.util import find_spec
import numpy as np
import time
from ..base_robot import BaseRobot
from .configuration_realman import RealmanConfig


class Realman(BaseRobot):
    """
    Realman robot implementation.
    Params:
    - config: RealmanConfig
    """

    config_class = RealmanConfig
    name = "realman"

    def __init__(self, config: RealmanConfig) -> None:
        super().__init__(config)
        self.config = config
        self._last_gripper_cmd = None
        self._last_joint_cmd = None

    def _check_dependency(self) -> None:
        """
        Check for dependencies required by the Realman robot.
        Raises ImportError if the required package is not found.
        """
        if find_spec("Robotic_Arm") is None:
            raise ImportError(
                "Realman robot requires the Robotic_Arm package. "
                "Please install it using 'pip install Robotic_Arm'."
            )
    
    def _connect_arm(self) -> None:
        """
        Connect to the Realman robot arm.
        Initializes the RoboticArm interface and creates a robot arm handle.
        """
        from Robotic_Arm.rm_robot_interface import (
            RoboticArm, 
            rm_thread_mode_e,
        )
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.arm.rm_create_robot_arm(self.config.ip, self.config.port)
        self.arm.rm_set_arm_run_mode(1)
    
    def _disconnect_arm(self) -> None:
        """
        Disconnect from the Realman robot arm.
        Destroys the robot arm handle.
        """
        ret_code = self.arm.rm_destroy()
        if ret_code != 0:
            raise RuntimeError(f'Failed to disconnect: {ret_code}')
    
    # def _set_joint_state(self, state: np.ndarray) -> None:
    #     """
    #     Set the joint state of the Realman robot.
    #     Uses the RoboticArm interface to move the joints and set the gripper position.
    #     Raises RuntimeError if the movement fails.
    #     Params:
    #     - state: np.ndarray of joint positions
    #     """
    #     state = list(state)
    #     success = self.arm.rm_movej(state[:-1], v=self.config.velocity, r=0, connect=0, block=self.config.block)

    #     if success != 0:
    #         raise RuntimeError(f'Failed movej')
    #     success = self.arm.rm_set_gripper_position(int(state[-1]), block=self.config.block, timeout=3)
    #     if success != 0:
    #         raise RuntimeError('Failed set gripper')

    #     if not self.config.block:
    #         time.sleep(self.config.wait_second)
    def _set_joint_state(self, state: np.ndarray) -> None:
        state = list(state)
        joint_cmd = np.array(state[:-1], dtype=float)

        should_send_joint = True
        if self._last_joint_cmd is not None and self.config.joint_cmd_threshold_deg > 0:
            if np.all(np.abs(joint_cmd - self._last_joint_cmd) < self.config.joint_cmd_threshold_deg):
                should_send_joint = False

        if should_send_joint:
            if self.config.use_canfd:
                success = self.arm.rm_movej_canfd(
                    joint_cmd.tolist(),
                    self.config.canfd_follow,
                    self.config.canfd_expand,
                    self.config.canfd_trajectory_mode,
                    self.config.canfd_radio,
                )
                if success != 0:
                    raise RuntimeError(
                        f"Failed movej_canfd: {success}. "
                        "If canfd_follow=True, ensure control period <= 10ms per SDK docs."
                    )
            else:
                success = self.arm.rm_movej(
                    joint_cmd.tolist(),
                    v=self.config.velocity,
                    r=0,
                    connect=0,
                    block=self.config.block,
                )
                if success != 0:
                    raise RuntimeError("Failed movej")
            self._last_joint_cmd = joint_cmd

        if self.config.gripper_use_binary_cmd:
            gripper_cmd = 1 if float(state[-1]) > self.config.gripper_cmd_threshold else 0

            # 只有状态变化时才发 gripper 命令，避免每一帧重复轰炸控制器
            if gripper_cmd != self._last_gripper_cmd:
                if gripper_cmd == 1:
                    success = self.arm.rm_set_gripper_pick_on(
                        self.config.gripper_close_speed,
                        self.config.gripper_close_force,
                        False,
                        0,
                    )
                    if success != 0:
                        raise RuntimeError(f"Failed gripper pick_on: {success}")
                else:
                    success = self.arm.rm_set_gripper_release(
                        self.config.gripper_open_speed,
                        False,
                        0,
                    )
                    if success != 0:
                        raise RuntimeError(f"Failed gripper release: {success}")

                self._last_gripper_cmd = gripper_cmd
        else:
            success = self.arm.rm_set_gripper_position(
                int(state[-1]),
                block=self.config.block,
                timeout=3,
            )
            if success != 0:
                raise RuntimeError("Failed set gripper")

        if should_send_joint and not self.config.block:
            time.sleep(self.config.wait_second)
    
    def _get_joint_state(self) -> np.ndarray:
        """
        Get the joint state of the Realman robot.
        Uses the RoboticArm interface to retrieve the current joint and gripper states.
        Raises RuntimeError if retrieval fails.
        Returns:
        - state: np.ndarray of joint positions
        """
        ret_code, joint = self.arm.rm_get_joint_degree()
        if ret_code != 0:
            raise RuntimeError(f'Failed to get joint state: {ret_code}')
        # ret_code, grip = self.arm.rm_get_gripper_state()
        # grip = grip['actpos']
        # if ret_code != 0:
        #     raise RuntimeError(f'Failed to get gripper state: {ret_code}')
        # return np.array(joint + [grip])
        gripper_binary = 0 if self._last_gripper_cmd in (None, 0) else 1
        return np.array(joint + [gripper_binary], dtype=float)
    
    def _set_ee_state(self, state: np.ndarray) -> None:
        """
        Set the end-effector state of the Realman robot.
        Uses the RoboticArm interface to compute inverse kinematics and set joint states accordingly.
        Raises RuntimeError if inverse kinematics fails.
        Params:
        - state: np.ndarray of end-effector positions
        """
        from Robotic_Arm.rm_robot_interface import rm_inverse_kinematics_params_t
        state = list(state)
        ret_code, joint = self.arm.rm_algo_inverse_kinematics(rm_inverse_kinematics_params_t(
            q_in=self._get_joint_state()[:-1],
            q_pose=state[:-1],
            flag=1
        ))
        if ret_code != 0:
            print('IK error:', ret_code)
        self._set_joint_state(joint + [state[-1]])

    def _get_ee_state(self) -> np.ndarray:
        """
        Get the end-effector state of the Realman robot.
        Uses the RoboticArm interface to compute forward kinematics based on current joint states.
        Raises RuntimeError if retrieval fails.
        Returns:
        - state: np.ndarray of end-effector positions
        """
        joint = self._get_joint_state()
        pose = self.arm.rm_algo_forward_kinematics(joint[:-1], flag=1)
        return np.array(pose + [joint[-1]])
