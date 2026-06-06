# 启动 DreamZero 机器人客户端（RoboCOIN 侧）

这份文档说明如何在**真实 Realman (RM75) 机械臂**上启动 DreamZero 策略的控制客户端
`src/lerobot/scripts/server/robot_client_dreamzero.py`，连接到 DreamZero 推理服务器。

> 这个客户端是从 `robot_client_openpi.py` fork 来的，**复用了全部机器人控制逻辑**
> （相机采集、机械臂控制、控制循环、频率、键盘急停、录像），只改了和服务器通信的
> **数据格式**：发 DreamZero 需要的 3 路命名视图 + 8 维状态，收回 `(N, 8)` 动作块。
> 所有归一化 / 相对动作解码 / 拼图都在**服务器端**完成。单位全程弧度（模型单位）,
> 硬件度↔弧度由 RoboCOIN 的 realman 机器人类自己处理,client/server 都不转。

---

## 0. 前置条件

- DreamZero 推理服务器已经在另一台（或同一台）机器上跑起来了，监听 `host:port`
  （服务器侧启动见 dreamzero 仓库的 `docs/REALMAN_INFERENCE_SERVER.md`）。
- 本机已安装 `openpi-client`（客户端只用它当 websocket+msgpack 的传输管道）：
  ```bash
  pip install openpi-client
  ```
- 三个物理相机已接好，并且**和数据采集时的物理顺序一致**。

---

## 1. 关键：相机顺序必须和训练一致

DreamZero 训练用了 **3 路同步相机**，对应三个视图：

```
观测视图 1 -> video.nominal_image          （主/标称视角）
观测视图 2 -> video.purturbated_c1_image    （扰动视角 1）
观测视图 3 -> video.purturbated_c2_image    （扰动视角 2）
```

客户端把 `--camera_keys` 里列出的 3 个相机**按顺序**映射到上面三个视图：
`camera_keys[0] -> nominal`，`camera_keys[1] -> c1`，`camera_keys[2] -> c2`。

⚠️ **顺序错了模型直接废**。务必让 `--camera_keys` 的顺序 = 采集时 nominal / c1 / c2 的物理相机顺序。

---

## 2. 启动命令

```bash
uv run python src/lerobot/scripts/server/robot_client_dreamzero.py \
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
```

参数说明：

| 参数 | 含义 |
|---|---|
| `--host` / `--port` | DreamZero 推理服务器地址（不是机器人地址） |
| `--task` | 任务文本，会作为 `prompt` 发给模型 |
| `--robot.ip` / `--robot.port` | Realman 机械臂控制器地址 |
| `--robot.use_canfd=True` | 用 CANFD 实时跟随下发关节（高频控制建议开） |
| `--robot.joint_cmd_threshold_deg` | 关节命令变化小于该角度（度）就不下发，抑制抖动 |
| `--robot.cameras` | 3 个相机的设备配置（类型/索引/分辨率） |
| `--camera_keys` | 3 个相机键，**按 nominal / c1 / c2 顺序** |
| `--frequency` | 控制循环频率（Hz） |

> 单位说明：客户端收发的关节都是 **弧度（radian，模型单位）**。RoboCOIN 的 realman
> 机器人类在 `send_action` 内部把弧度转成硬件的度,所以 client/server 都**不转单位**。

---

## 3. 运行时操作

- 启动后客户端进入控制循环：采图 → 发服务器 → 收动作块 → 逐条下发 → 按 `--frequency` 节流。
- 键盘：
  - `q` 结束本次 rollout（随后会问成功与否）
  - `y` / `n` 标记成功 / 失败，并保存这次 rollout 的录像到 `--result_dir`（默认 `results/`）。
- 终端每 `--timing_log_interval` 步打印一次各阶段耗时（采图 / 推理 / 下发等），方便排查瓶颈。

---

## 4. 排查

- **`ImportError: openpi_client`**：`pip install openpi-client`（仅用作传输管道）。
- **连接不上服务器**：确认 `--host/--port` 指向的是 DreamZero 服务器、防火墙放行、服务器已 ready。
- **机械臂不动 / 动作乱**：先在 dreamzero 仓库做**开环验证**（拿训练数据对比预测 vs 真值），
  确认服务器侧的 normstats / 相对解码 / 视图顺序都对了，再上真机。
- **相机报错**：用 `ls /dev/video*` 查 opencv 索引；RealSense 用 `serial_number_or_name`。
- **断言 “Expected camera key ...”**：`--camera_keys` 里的名字必须和 `--robot.cameras` 里的键一致。

---

## 5. 这个客户端和 openpi 版的区别（给维护者）

只有数据 schema 不同，机器人控制部分完全一致：

- 发出去：`video.nominal_image / video.purturbated_c1_image / video.purturbated_c2_image`
  （原始相机帧）+ `state.joint_pos`(7, 弧度) + `state.gripper_pos`(1) + `prompt`。
- 收回来：`{"actions": (N, 8)}`，即 `[7 关节角(弧度), 1 夹爪]`，模型单位。
- 没有 openpi 那套 `observation.` / `observation/` 双命名空间，也没有相机键的隐式约定。
