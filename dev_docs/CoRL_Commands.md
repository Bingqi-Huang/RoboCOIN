# Open leader Arm
```bash
uv run ./DEBUG/Realman_Gripper.py \
  --ip 192.168.1.17 \
  --port 8080 \
  --state-file /tmp/realman_gripper_state.json \
  --close-speed 500 \
  --close-force 1000 \
  --open-speed 500 \
  --init-open
```
# Task 1: pick up the blue batery and place it into cardboard box.
Note: the three D435I cameras are stable at `640x480@15fps` on the current USB topology. Avoid `640x480@30fps` unless the cameras are moved to USB3 bandwidth.

```bash
uv run ./src/lerobot/record.py \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.init_type="none" \
  --robot.block=False \
  --robot.wait_second=0.0 \
  --robot.visualize=False \
  --robot.draw_2d=False \
  --robot.draw_3d=False \
  --display_data=False \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.canfd_expand=0 \
  --robot.canfd_trajectory_mode=1 \
  --robot.canfd_radio=50 \
  --robot.joint_cmd_threshold_deg=0.05 \
  --robot.cameras="{nominal_image: {type: intelrealsense, serial_number_or_name: 317222074160, width: 640, height: 480, fps: 15}, purturbated_c1_image: {type: intelrealsense, serial_number_or_name: 141722078357, width: 640, height: 480, fps: 15}, purturbated_c2_image: {type: intelrealsense, serial_number_or_name: 135122074505, width: 640, height: 480, fps: 15}}" \
  --robot.id=rm75_follower \
  --teleop.type=realman_leader \
  --teleop.ip="192.168.1.18" \
  --teleop.port=8080 \
  --teleop.id=rm75_leader \
  --teleop.init_type="none" \
  --robot.velocity=70 \
  --dataset.repo_id="CoRL/Task1_new" \
  --dataset.num_episodes=50 \
  --dataset.single_task="pick up the blue battery and place it into cardboard box." \
  --dataset.push_to_hub=False \
  --dataset.video=True \
  --dataset.fps=15 \
  --dataset.reset_time_s=120 \
  --dataset.episode_time_s=30 \
  --dataset.num_image_writer_processes=0 \
  --dataset.num_image_writer_threads_per_camera=2 \
  --resume=True
```

```bash
uv run src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.240" \
  --port=8000 \
  --task="pick up the blue battery and place it into cardboard box." \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.block=False \
  --robot.init_type="none" \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.wait_second=0.0 \
  --robot.joint_cmd_threshold_deg=0.1 \
  --robot.velocity=90\
  --robot.cameras="{nominal_image: {type: intelrealsense, serial_number_or_name: 317222074160, width: 640, height: 480, fps: 15}}" \
  --camera_keys="[nominal_image]" \
  --robot.id=rm75_follower \
  --frequency=15
```
243322073824

# Task 2: close the laptop lid.
Note: the three D435I cameras are stable at `640x480@15fps` on the current USB topology. Avoid `640x480@30fps` unless the cameras are moved to USB3 bandwidth.

```bash
uv run ./src/lerobot/record.py \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.init_type="none" \
  --robot.block=False \
  --robot.wait_second=0.0 \
  --robot.visualize=False \
  --robot.draw_2d=False \
  --robot.draw_3d=False \
  --display_data=False \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.canfd_expand=0 \
  --robot.canfd_trajectory_mode=1 \
  --robot.canfd_radio=50 \
  --robot.joint_cmd_threshold_deg=0.05 \
  --robot.cameras="{nominal_image: {type: intelrealsense, serial_number_or_name: 317222074160, width: 640, height: 480, fps: 15}, purturbated_c1_image: {type: intelrealsense, serial_number_or_name: 141722078357, width: 640, height: 480, fps: 15}, purturbated_c2_image: {type: intelrealsense, serial_number_or_name: 135122074505, width: 640, height: 480, fps: 15}}" \
  --robot.id=rm75_follower \
  --teleop.type=realman_leader \
  --teleop.ip="192.168.1.18" \
  --teleop.port=8080 \
  --teleop.id=rm75_leader \
  --teleop.init_type="none" \
  --robot.velocity=70 \
  --dataset.repo_id="CoRL/Task2" \
  --dataset.num_episodes=48 \
  --dataset.single_task="close the laptop lid." \
  --dataset.push_to_hub=False \
  --dataset.video=True \
  --dataset.fps=15 \
  --dataset.reset_time_s=120 \
  --dataset.episode_time_s=30 \
  --dataset.num_image_writer_processes=0 \
  --dataset.num_image_writer_threads_per_camera=2 \
  --resume=True
```


```bash
uv run src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.240" \
  --port=8000 \
  --task="close the laptop lid." \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.block=False \
  --robot.init_type="none" \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.wait_second=0.0 \
  --robot.joint_cmd_threshold_deg=0.1 \
  --robot.velocity=25 \
  --robot.cameras="{nominal_image: {type: intelrealsense, serial_number_or_name: 135122074505, width: 640, height: 480, fps: 30}}" \
  --camera_keys="[nominal_image]" \
  --robot.id=rm75_follower \
  --frequency=15

```

# Task 3: Takeoff the headphone from the stand.
Note: the three D435I cameras are stable at `640x480@15fps` on the current USB topology. Avoid `640x480@30fps` unless the cameras are moved to USB3 bandwidth.

```bash
uv run ./src/lerobot/record.py \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.init_type="none" \
  --robot.block=False \
  --robot.wait_second=0.0 \
  --robot.visualize=False \
  --robot.draw_2d=False \
  --robot.draw_3d=False \
  --display_data=False \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.canfd_expand=0 \
  --robot.canfd_trajectory_mode=1 \
  --robot.canfd_radio=50 \
  --robot.joint_cmd_threshold_deg=0.05 \
  --robot.cameras="{nominal_image: {type: intelrealsense, serial_number_or_name: 317222074160, width: 640, height: 480, fps: 15}, purturbated_c1_image: {type: intelrealsense, serial_number_or_name: 141722078357, width: 640, height: 480, fps: 15}, purturbated_c2_image: {type: intelrealsense, serial_number_or_name: 135122074505, width: 640, height: 480, fps: 15}}" \
  --robot.id=rm75_follower \
  --teleop.type=realman_leader \
  --teleop.ip="192.168.1.18" \
  --teleop.port=8080 \
  --teleop.id=rm75_leader \
  --teleop.init_type="none" \
  --robot.velocity=70 \
  --dataset.repo_id="CoRL/Task3_new" \
  --dataset.num_episodes=50 \
  --dataset.single_task="takeoff the headphone from the stand." \
  --dataset.push_to_hub=False \
  --dataset.video=True \
  --dataset.fps=15 \
  --dataset.reset_time_s=120 \
  --dataset.episode_time_s=45 \
  --dataset.num_image_writer_processes=0 \
  --dataset.num_image_writer_threads_per_camera=2 \
  --resume=True
  ```


```bash
uv run src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.240" \
  --port=8000 \
  --task="takeoff the headphone from the stand." \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.block=False \
  --robot.init_type="none" \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.wait_second=0.0 \
  --robot.joint_cmd_threshold_deg=0.01\
  --robot.velocity=100 \
  --robot.cameras="{nominal_image: {type: intelrealsense, serial_number_or_name: 141722078357, width: 640, height: 480, fps: 15}}" \
  --camera_keys="[nominal_image]" \
  --robot.id=rm75_follower \
  --frequency=15

```




# TEST
uv run ./src/lerobot/record.py \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.init_type="none" \
  --robot.block=False \
  --robot.wait_second=0.0 \
  --robot.visualize=False \
  --robot.draw_2d=False \
  --robot.draw_3d=False \
  --display_data=False \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.canfd_expand=0 \
  --robot.canfd_trajectory_mode=1 \
  --robot.canfd_radio=50 \
  --robot.joint_cmd_threshold_deg=0.05 \
  --robot.cameras="{nominal_image: {type: intelrealsense, serial_number_or_name: 317222074160, width: 640, height: 480, fps: 15}, purturbated_c1_image: {type: intelrealsense, serial_number_or_name: 141722078357, width: 640, height: 480, fps: 15}, purturbated_c2_image: {type: intelrealsense, serial_number_or_name: 135122074505, width: 640, height: 480, fps: 15}}" \
  --robot.id=rm75_follower \
  --teleop.type=realman_leader \
  --teleop.ip="192.168.1.18" \
  --teleop.port=8080 \
  --teleop.id=rm75_leader \
  --teleop.init_type="none" \
  --robot.velocity=70 \
  --dataset.repo_id="CoRL/Test" \
  --dataset.num_episodes=25 \
  --dataset.single_task="takeoff the headphone from the stand." \
  --dataset.push_to_hub=False \
  --dataset.video=True \
  --dataset.fps=15 \
  --dataset.reset_time_s=12000 \
  --dataset.episode_time_s=45 \
  --dataset.num_image_writer_processes=0 \
  --dataset.num_image_writer_threads_per_camera=2 \
