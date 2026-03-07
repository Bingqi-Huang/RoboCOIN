from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

def get_current_pose(ip="192.168.1.18", port=8080):
    print(f"Connecting to Realman arm at {ip}:{port}...")
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    arm.rm_create_robot_arm(ip, port)
    
    # Ensure arm is in the correct run mode
    arm.rm_set_arm_run_mode(1)
    
    ret_joint, joint = arm.rm_get_joint_degree()
    ret_grip, grip = arm.rm_get_gripper_state()
    
    if ret_joint == 0 and ret_grip == 0:
        # Combine joint degrees and gripper actual position
        pose = joint + [grip['actpos']]
        # Round to 3 decimal places for a cleaner command line argument
        formatted_pose = [round(x, 3) for x in pose]
        
        print("\n--- Copy the array below ---")
        print(f"[{', '.join(map(str, formatted_pose))}]")
        print("----------------------------\n")
    else:
        print(f"Failed to read state. Joint err: {ret_joint}, Gripper err: {ret_grip}")
        
    arm.rm_destroy()

if __name__ == "__main__":
    # Reading from the Leader arm. You can change the IP to read the Follower arm if needed.
    get_current_pose(ip="192.168.1.18", port=8080)