# RM75 太阳能板擦拭任务 - 端到端实机操作手册 (RoboCOIN + OpenPI)

这份文档是你（或你的 Agent 团队）在连接着两台真实的 RealMan (RM75) 机械臂的电脑上，从环境配置到数据采集，再到训练控车的**完整实操指南**。强烈建议将这篇文档保存在操作电脑的桌面或工程目录下随时查阅。

---

## 🏎️ 第一步：环境与硬件准备

1. **硬件连接检查**
   - 确保从臂（执行任务的机械臂）通过网线/局域网连接，IP 设为 `192.168.1.18`，能被电脑 `ping` 通。
   - 确保主臂（你手里握着遥操的机械臂）IP 设为 `192.168.1.19`，也能被 `ping` 通。
   - 插入免驱的 Webcam（作为全局相机）。在终端输入 `ls /dev/video*`，找到它生成的串口号（比如 `/dev/video0`）。
   - 插入 Intel RealSense D435i（作为腕部相机）。

2. **环境变量与存储位置设置**
   如果你的系统盘空间有限，请一定在终端声明数据集将要保存到的大容量硬盘路径：
   ```bash
   export LEROBOT_HOME="/mnt/大容量硬盘路径/RobotData"
   ```

3. **依赖安装**
   请在一个隔离的 conda 环境（比如 `openpi`）中执行：
   ```bash
   # 1. 安装 RM75 官方 SDK
   pip install Robotic_Arm
   
   # 2. 克隆并安装 RoboCOIN 开发版
   git clone https://github.com/FlagOpen/RoboCOIN.git
   cd RoboCOIN
   pip install -e .
   ```

*(准备工作完成！)*

---

## 🎬 第二步：开始数据采集（双臂主从遥操）

将擦拭抹布固定在从臂的夹爪上，并将从臂拉至太阳能板前的待机位置。
手握主臂，准备开始录制。在终端运行以下采集命令：

```bash
uv run ./src/lerobot/record.py \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.cameras="{scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist_image: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}}" \
  --robot.id=rm75_follower \
  --teleop.type=realman_leader \
  --teleop.ip="192.168.1.18" \
  --teleop.port=8080 \
  --teleop.id=rm75_leader \
  --teleop.init_type="none" \
  --dataset.repo_id="myorg/rm75_wipe_solar" \
  --dataset.num_episodes=10 \
  --dataset.single_task="wipe the solar panel" \
  --dataset.push_to_hub=False
```

- Command for picking silver USB-CAN box to white container.
```bash
uv run ./src/lerobot/record.py \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.cameras="{scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist_image: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}}" \
  --robot.id=rm75_follower \
  --teleop.type=realman_leader \
  --teleop.ip="192.168.1.18" \
  --teleop.port=8080 \
  --teleop.id=rm75_leader \
  --teleop.init_type="none" \
  --robot.wait_second=0.01 \
  --robot.velocity=30 \
  --dataset.repo_id="myorg/rm75_pick_place_silver_box" \
  --dataset.num_episodes=20 \
  --dataset.single_task="Pick up the silver box and place it into the white container"\
  --dataset.push_to_hub=False \
   --dataset.fps=15
```



**TEST COMMANDS**
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
 --robot.cameras="{scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
 --robot.id=rm75_follower \
 --teleop.type=realman_leader \
 --teleop.ip="192.168.1.18" \
 --teleop.port=8080 \
 --teleop.id=rm75_leader \
 --teleop.init_type="none" \
 --robot.velocity=70 \
 --dataset.repo_id="bingqi/rm75_canfd_verified_test_profile" \
 --dataset.num_episodes=1 \
 --dataset.single_task="debug" \
 --dataset.push_to_hub=False \
 --dataset.video=True \
 --dataset.fps=15 \
 --dataset.reset_time_s=5 \
 --dataset.episode_time_s=20 \
 --dataset.num_image_writer_processes=0 \
 --dataset.num_image_writer_threads_per_camera=2 \
 --dataset.profile_loop_timing=True \
 --dataset.profile_loop_timing_every_n=50 \





```

**TEST WITH 2 CAMERAS**
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
 --robot.cameras="{scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist_image: {type: opencv, index_or_path: 6, width: 640, height: 480, fps: 30}}" \
 --robot.id=rm75_follower \
 --teleop.type=realman_leader \
 --teleop.ip="192.168.1.18" \
 --teleop.port=8080 \
 --teleop.id=rm75_leader \
 --teleop.init_type="none" \
 --robot.velocity=70 \
 --dataset.repo_id="bingqi/rm75_canfd_verified_test" \
 --dataset.num_episodes=1 \
 --dataset.single_task="debug" \
 --dataset.push_to_hub=False \
 --dataset.video=True \
 --dataset.fps=20 \
 --dataset.reset_time_s=5 \
 --dataset.episode_time_s=20 \
 --dataset.num_image_writer_processes=0 \
 --dataset.num_image_writer_threads_per_camera=2 \
 --dataset.profile_loop_timing=True \
 --dataset.profile_loop_timing_every_n=20
 ```

- Command for pick 3 (or more) silver DC-DC converters to white container.

This is an AB test to test the performance effect with prompt inluding [A-side]detailed object description(silver box-like object) & [B-Side] professional object description(a DC-DC converter)

[A-Side]
```bash
- Command for pick 2 (or more) silver DC-DC converters to cardboard box.

This is an AB test to test the performance effect with prompt inluding [A-side]detailed object description(silver box-like object) & [B-Side] professional object description(a DC-DC converter)

[A-Side]
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
  --robot.cameras="{scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist_image: {type: intelrealsense, serial_number_or_name: 243322073824, width: 640, height: 480, fps: 30}}" \
  --robot.id=rm75_follower \
  --teleop.type=realman_leader \
  --teleop.ip="192.168.1.18" \
  --teleop.port=8080 \
  --teleop.id=rm75_leader \
  --teleop.init_type="none" \
  --robot.velocity=70 \
  --dataset.repo_id="bingqi/rm75_pick_place_2_converters_a_side" \
  --dataset.num_episodes=15 \
  --dataset.single_task="Pick up all silver cube-like objects with cables, and place them into the cardboard box" \
  --dataset.push_to_hub=False \
  --dataset.video=True \
  --dataset.fps=15 \
  --dataset.reset_time_s=120 \
  --dataset.episode_time_s=60 \
  --dataset.num_image_writer_processes=0 \
  --dataset.num_image_writer_threads_per_camera=2 \
  --resume=True

```

[B-Side]
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
  --robot.cameras="{scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist_image: {type: intelrealsense, serial_number_or_name: 243322073824, width: 640, height: 480, fps: 30}}" \
  --robot.id=rm75_follower \
  --teleop.type=realman_leader \
  --teleop.ip="192.168.1.18" \
  --teleop.port=8080 \
  --teleop.id=rm75_leader \
  --teleop.init_type="none" \
  --robot.velocity=70 \
  --dataset.repo_id="bingqi/rm75_pick_place_2_converters_b_side" \
  --dataset.num_episodes=100 \
  --dataset.single_task="Pick up all silver DC-DC converters with cables and place them into the cardboard box" \
  --dataset.push_to_hub=False \
  --dataset.video=True \
  --dataset.fps=15 \
  --dataset.reset_time_s=120 \
  --dataset.episode_time_s=60 \
  --dataset.num_image_writer_processes=0 \
  --dataset.num_image_writer_threads_per_camera=2

```





*(注意：请根据实际情况替换 `--robot.cameras` 中的 `index_or_path`)*

**操作快捷键：**
- ▶️ **开始/保存**：按下电脑键盘的 **右方向键 (`→`)** 开始录制当前 episode。控制主臂完成擦拭后，**再按一次**右方向键结束并保存。
- ❌ **删除重来**：碰倒东西了？直接按 **左方向键 (`←`)** 丢弃刚才那条废片。
- 🚪 **退出录制**：达到 50 条后，按 **`ESC`** 优雅退出，数据会自动落盘到 `$LEROBOT_HOME/myorg/rm75_wipe_solar/` 目录下（底层已包含 HDF5 的升级版本 `.parquet` 文件和 `.mp4` 录像）。


### Leader Arm Gripper control

Run below command side by side with the above record script, use key `c` to close gripper and `o` to open.
``bash

uv run ./DEBUG/Realman_Gripper.py \
  --ip 192.168.1.18 \
  --port 8080 \
  --state-file /tmp/realman_gripper_state.json \
  --close-speed 500 \
  --close-force 1000 \
  --open-speed 500 \
  --init-open

uv run ./DEBUG/Realman_Gripper.py \
  --ip 192.168.1.17 \
  --port 8080 \
  --state-file /tmp/realman_gripper_state.json \
  --close-speed 500 \
  --close-force 1000 \
  --open-speed 500 \
  --init-open

---

## 🧠 第三步：代码修复与模型训练 (在 GPU 服务器)

这一步你需要移步到带有大显存 GPU（如 RTX 5090）的服务器（你可以把刚刚采集完的数据目录拷过去，放在服务器的 `~/.cache/huggingface/lerobot/` 或指定目录下）。

1. **修正 OpenPI 的接口代码（极其重要！）**
   因为 RoboCOIN 吐出的特征向量默认已经拼接好，所以你需要使用文本编辑器或让 Agent Swarm 打开 [openpi/src/openpi/research/shared/rm75_policy.py](file:///mnt/SharedData/Research/openpi/src/openpi/research/shared/rm75_policy.py)。
   
    **修改 A (169行附近，[LeRobotRM75DataConfig](file:///mnt/SharedData/Research/openpi/src/openpi/research/shared/rm75_policy.py#153-203))**
    ```diff
    - "observation/joint_position": "observation.joint_position",
    - "observation/joint_velocity": "observation.joint_velocity",
    - "observation/gripper_position": "observation.gripper_position",
    + "observation/state": "observation.state",
    ```
    
    **修改 B (99行附近，`RM75Inputs.__call__`)**
    ```diff
    - state = np.concatenate([
    -    data["observation/joint_position"],
    -    data["observation/gripper_position"],
    - ])
    + state = data["observation/state"]  # RoboCOIN 的 state 已经是 8D
    ```

2. **计算数据集统计特征**
   ```bash
   uv run scripts/compute_norm_stats.py --config-name pi05_rm75_wipe
   ```

3. **启动 LoRA 微调 (SFT)**
   ```bash
   XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
   uv run scripts/train.py pi05_rm75_wipe \
       --exp-name=wipe_panel_native_lora \
       --overwrite
   ```
*(等待几小时，直到训练跑完约 10k steps。你的最优权重将会保存在 `checkpoints/pi05_rm75_wipe/wipe_panel_native_lora/` 下)*

---

## 🚀 第四步：实机推理（让机器人自己擦板子）

训练完成后，保持带有 GPU 的服务器与机械臂电脑处在同一局域网（或同机）。

**1. 在 GPU 节点：启动 OpenPI 服务器端**
这个命令会加载大模型并开启一个运算服务监听网络请求：
```bash
uv run  src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.166" \
  --port=8000 \
  --task="Pick up all silver cube-like objects with cables, and place them into the cardboard box" \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.block=False \
  --robot.init_type="none" \
  --robot.cameras="{ observation.scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, observation.wrist_image: {type: intelrealsense, serial_number_or_name: 243322073824, width: 640, height: 480, fps: 30}}" \
  --camera_keys="[ observation.scene_image, observation.wrist_image ]" \
  --robot.id=rm75_follower

```

## At robot side
Using can_fd for faster response:
```bash
uv run src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.166" \
  --port=8000 \
  --task="Pick up all silver cube-like objects with cables, and place them into the cardboard box" \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.block=False \
  --robot.init_type="none" \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.wait_second=0.0 \
  --robot.joint_cmd_threshold_deg=0.1 \
  --robot.velocity=70 \
  --robot.cameras="{ observation.scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, observation.wrist_image: {type: intelrealsense, serial_number_or_name: \"243322073824\", width: 640, height: 480, fps: 30}}" \
  --camera_keys="[ observation.scene_image, observation.wrist_image ]" \
  --robot.id=rm75_follower \
  --frequency=10
```


*(假设这台 GPU 机器的局域网 IP 是 `192.168.1.100`)*

**2. 在机械臂电脑：启动 RoboCOIN 控制客户端**
运行一条命令，它就会不停抓取摄像头与关节状态 -> 发向 GPU 服务器 -> 接收动作下发电机执行：
```bash
python scripts/server/robot_client_openpi.py \
  --host="192.168.1.100" \
  --port=8000 \
  --task="wipe the solar panel" \
  --robot.type=realman \
  --robot.ip="192.168.1.18" \
  --robot.port=8080 \
  --robot.init_type="joint" \
  --robot.cameras="{ observation.scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, observation.wrist_image: {type: realsense, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
  --robot.id=rm75_follower
```





Using can_fd for faster response:



[ATTENTION!!!!!]use port 8000 for native openpi server and 8001 for the LunarPressure task
```bash
uv run src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.166" \
  --port=8000 \
  --task="Pick up all silver cube-like objects with cables, and place them into the cardboard box" \
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
  --timing_log_window=10
```



## OOD Tests:
- Pick stuff Out of box (Never learned) FAILED!
```bash
uv run src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.166" \
  --port=8000 \
  --task="Take all silver cube-like objects with cables out of cardboard box" \
  --robot.type=realman \
  --robot.ip="192.168.1.17" \
  --robot.port=8080 \
  --robot.block=False \
  --robot.init_type="none" \
  --robot.use_canfd=True \
  --robot.canfd_follow=False \
  --robot.wait_second=0.0 \
  --robot.joint_cmd_threshold_deg=0.05 \
  --robot.velocity=70 \
  --robot.cameras="{ observation.scene_image: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, observation.wrist_image: {type: intelrealsense, serial_number_or_name: \"243322073824\", width: 640, height: 480, fps: 30}}" \
  --camera_keys="[ observation.scene_image, observation.wrist_image ]" \
  --robot.id=rm75_follower \
  --frequency=100
  ```

- Place to another container:
```bash
uv run src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.166" \
  --port=8000 \
  --task="Pick up all silver cube-like objects with cables, and place them into the white container" \
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
  --timing_log_window=10

  ```


```bash

uv run src/lerobot/scripts/server/robot_client_openpi.py \
  --host="192.168.1.166" \
  --port=8000 \
  --task="Pick objects from carboard box to white container" \
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
  --timing_log_window=10
```




### Summarize on Generalization EXP results

---
Note: The defaults are 1 Light, silver box, cardbaord box, Pick all object to container

| Num | Lighting | Object Type | Container Type | Action Type | More OOD Condition | Success |
|---|---|---|---|---|---|---|
| 1 | More Light | 0 | 0 | 0 | YES |
| 1 | More Light | Changed Siler object + Black radio | 0 | 0 | Only silver box succeed, black radio is too thin to grasp.  |
| 3 | More Light | Changed Siler object + Blue Box | 0 | 0 | 
| 3 | More Light | 0 | White Container | 0 | YES |
| 4 | 0 | 0 | 0 | Put left one only |  |


Ambigous language prompt


# Observed Generalization

- It is able to adjust it's grasp point when target is dynamically re-positioned or removed.
- It is able to know find a empty spot in the container to drop an object.
- It will stop when all things in the container, but this is too stimulated (See below 3)
# How to Improve?



1. Add more recovery data: Make the model understand not everytime it close the gripper means it need to move obj back to container. Maybe add one more dim of the gripper state, to guide robot understand if it grasped sth or not.
2. More Grasp depth variation: Grasp objects that are lower and the arm need to go more towards the table to pick.
3. **Add more object specific data:** "put blue cylinder into box (but there's some other object on the table)" . **It should accept objects that not in the container as an END OF EPISODE.**