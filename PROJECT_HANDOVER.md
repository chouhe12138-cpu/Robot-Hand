# UR3e 双相机抓取与人机交接：项目交接文档

更新日期：2026-10-19。本文是当前状态快照；旧过程记录见 [LOCAL_SETUP.md](LOCAL_SETUP.md)。当日晚间已完成抓取 + 20 cm 抬升的真机验证，完整自动交接仍未跑通。

## 2026-10-20 实验进展补充

最新状态与少量实验截图见 [2026-10-20 精选实验记录](docs/experiments/2026-10-20/README.md)。下文 2026-10-19 内容为历史快照；发生冲突时以最新记录及代码为准。

## 1. 项目目标与当前结论

项目位于 `/home/ros2/Desktop/Robot-Hand/UR3e_Handover-master`，使用 UR3e、Robotiq 2F-85、腕部 ZED 2i 和外部 Intel RealSense D435i，实现桌面抓取及人机交接。

用户期望：程序从固定 Home 开始；D435i 在 ZED 看不到物体时提供观察位置引导；人工选物、审视双相机画面后，在 READY 状态按空格开始；之后由程序自行完成抓取、交接及回位，不依赖聊天 agent 逐步控制。人工选物可以保留，暂不需要语音。

**当前不是已验收的完整自动交接系统。** 已多次完成小黄鸭抓取和 20 cm 抬升，其中 17:05 一次在抬升后进入等待右手阶段，但 30 秒内未检测到右手。完整自动递送、跟随和松爪尚未验证通过。当前抓取成功率偏低：小物体仍会发生碰撞，大物体常因所需开口超过约 77 mm 被筛选。最新的无解 IK 保护、控制脚本自动恢复、D435i 选区锁定已接入，部分只做了离线验证。

## 2. 硬件、环境与坐标约定

| 项目 | 当前配置 |
| --- | --- |
| 机械臂 | UR3e，IP `192.168.1.10` |
| 夹爪 | Robotiq 2F-85，URCap Socket `63352` |
| 机器人通信 | RTDE `30004`；Dashboard `29999` |
| 腕部相机 | ZED 2i，序列号 `35818386`，眼在手上 |
| 外部相机 | D435i，序列号 `135122079629`，眼在手外 |
| 系统 | Ubuntu 22.04，ROS 2 Humble，Python 3.10 |
| Conda | `/home/ros2/miniconda3/envs/urho` |
| GPU | RTX 4060 Laptop，8 GB；PyTorch 2.6.0+cu124 |
| 活动 TCP | tool0 沿 Z 正方向 `0.135 m`；转接板厚 `0.011 m` |

每个终端均从项目根目录加载环境：

```bash
cd /home/ros2/Desktop/Robot-Hand/UR3e_Handover-master
source activate_handover.bash
```

不要激活废弃工作区的 `.venv`。本项目激活脚本加载本地 ZED SDK、ROS 包、模型路径及缓存配置。当前环境不需要重装。

控制器基座坐标与 ROS `base_link` 的转换为 `diag(-1, -1, 1)`，不能混用。代码中的物理右手在未镜像 MediaPipe 图像中对应标签 `Left`。TCP、外参和夹爪几何不能通过经验偏移随意修正。

此前工具沙箱看不到 GPU、相机和机器人网络，沙箱外访问正常；这不是 GPU 驱动故障。用户已授权本次开发过程使用 GPU 与指定真机测试，但每轮动作仍应遵循当轮任务范围。

## 3. 主要文件与迁移资产

| 文件或目录 | 用途 |
| --- | --- |
| [AGENTS.md](AGENTS.md) | 协作约定：可复用、可维护、可读优先；减少无关测试与改动 |
| [activate_handover.bash](activate_handover.bash) | 本机环境入口 |
| [LOCAL_SETUP.md](LOCAL_SETUP.md) | 环境、迁移与逐次测试记录 |
| [robot2human_handover.py](src/graspnet-baseline/zedInhand_rsTohand/robot2human_handover.py) | 当前重点修改的“机器人递给人”入口 |
| [human2robot_handover.py](src/graspnet-baseline/zedInhand_rsTohand/human2robot_handover.py) | “人递给机器人”入口；未同步本轮 Home、READY、Esc 等流程改动 |
| [handover_runtime.py](handover_runtime.py) | Home 读取、回位保护、运动配置、抬升位置计算 |
| [robotiq_grasp.py](robotiq_grasp.py) | 两个入口共用的实体夹爪显示、筛选、TCP 映射 |
| [tools/calibration/](tools/calibration/) | 标定采集、求解与验证脚本，从 `/home/ros2/Desktop/Object-Hand` 迁入 |
| [d435_handeye_core.py](d435_handeye_core.py) | 标定板检测、姿态多样性检查与眼在手外求解 |
| [object_hand/calibration.py](object_hand/calibration.py) | 标定数学工具（变换、均值、误差、OpenCV 手眼求解） |
| [run_d435_handeye.bash](run_d435_handeye.bash) | D435i 标定入口：空格采集 20 组并自动求解 |
| [run_zed_handeye.bash](run_zed_handeye.bash) | ZED 标定入口：手动换姿态、空格采集 20 组并自动求解 |
| [robotiq_2f85_model.py](robotiq_2f85_model.py) | 实体夹爪几何、联动、凸包碰撞检查 |
| [robot_home.json](config/robot_home.json) | 用户现场摆好的固定 Home，使用弧度关节值 |
| [handover_motion.json](config/handover_motion.json) | 当前抬升、速度和加速度 |
| [robotiq_2f85.json](config/robotiq_2f85.json) | 夹爪安装、行程和碰撞参数 |
| `config/ur3e_factory_calibration.yaml` | UR 驱动使用的机械臂出厂运动学参数 |
| `config/ur3e_calibrated.urdf` | 本地标定 URDF，供离线运动学分析使用 |
| `vendor/ros2_robotiq_gripper/robotiq_description/` | 实体模型网格及描述、许可证；不作为另一套驱动启动 |
| `calibration_data/` | 标定原始记录、求解结果及历史版本 |
| `captured_data/` | 实景图片、点云、抓取候选及部分执行记录 |

从废弃仓库 `/home/ros2/Desktop/Object-Hand` 迁入了标定、权重、SDK 与实体夹爪资源。未整体迁移旧环境和旧实验代码。历史单次测试中的 `/tmp` 脚本部分直接引用旧仓库模块，不能当作本项目正式入口，也不要把这些临时文件作为后续开发基础。

权重位置：

- GraspNet：`src/graspnet-baseline/logs/log_rs/checkpoint-rs.tar`
- SAM2：`src/SAM2_streaming/checkpoints/sam2.1/sam2.1_hiera_small.pt`
- FFS：`src/Fast-FoundationStereo/weights/23-36-37/model_best_bp2_serialize.pth`
- ZED SDK、相机内参：`.zed-sdk/`
- 本机依赖记录：`requirements-local.txt`、`requirements-local-constraints.txt`、`requirements-local-lock.txt`

## 4. 当前标定

- ZED：启动时自动选择最新的 `calibration_data/zed_eye_in_hand_*/calibrated_result/zed_tool0_calibration.json`；当前为 `20261019_140354_716625`。
- D435i：启动时自动选择最新的、`quality_passed` 为真的 `calibration_data/d435_eye_to_hand_*/calibrated_result/d435_base_calibration.json`；当前为 `20261018_204340_085125`。
- 生效启动文件：`src/calibration_result/ur3e_eye_in_hand/eye_in_hand_zed2i.launch.py` 与 `src/calibration_result/ur3e_eye_to_hand/eye_to_hand_rs_d435i.launch.py`；两者在启动时读取上述文件，不再硬编码外参。
- 旧标定目录不会自动删除，保留为历史记录；新标定完成后重启终端 3/4 即可生效，不需要修改 launch 文件。

ZED 变换为 `tool0 → zed_inhand_camera_frame_optical`；D435i 为 `base_link → rs_tohand_color_optical_frame`。同一 child frame 只启动一个发布者。历史上游启动文件归档在 `calibration_data/upstream_launch/`。

标定入口从 `/home/ros2/Desktop/Object-Hand` 迁入，采集与求解逻辑未改：

```bash
./run_zed_handeye.bash     # 手动换姿态，空格保存 20 组后自动求解
./run_d435_handeye.bash    # 固定相机，空格保存 20 组后自动求解
./run_d435_handeye.bash --solve-only calibration_data/<session>
```

ZED 入口使用 TCP `0 0 0.135 0 0 0` 和 7×5、30 mm 方格、22 mm 标记板；采集只读取 RTDE 状态，不发送运动指令。D435i 采集时相机固定、标定板刚性固定在末端。求解结果写入各自会话的 `calibrated_result/`；新标定不会自动删除旧结果，也不会自动重启正在运行的静态 TF。发布 TF 前确保同一 child frame 只有一个发布者。

旧实物抓取成功提供了局部使用证据，但两份外参尚未完成独立、系统的实物精度验收。ZED 曾出现 self-calibration failed 警告，SDK 仍加载本机内参；不要把该警告直接等同于手眼标定失效，也不能据此保证定位精度。

## 5. 自主启动抓取测试

以下共五个终端。**每个终端先执行第 2 节的环境加载命令**，并保持在项目根目录。若对应服务已经运行，不要重复启动。

终端 1–4 也可以用脚本一次启动（主程序仍单独运行）：

```bash
./start_handover.sh       # 启动缺失的服务，等待 /joint_states、夹爪节点和两路 TF 就绪
./stop_handover.sh        # 停止脚本启动的服务
./stop_handover.sh --all  # 停止本项目全部相关进程，包括手动启动的
```

脚本会跳过已经在运行的服务，日志写入 `logs/`，不启动主程序。`--all` 只匹配本项目的驱动、夹爪、两路外参和主程序，不碰其他项目；两种停止方式都不要在持物时使用。

终端 1：机器人驱动与状态发布。

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e robot_ip:=192.168.1.10 launch_rviz:=false \
  kinematics_params_file:="$PWD/config/ur3e_factory_calibration.yaml"
```

终端 2：夹爪服务。

```bash
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py \
  --ros-args -p robot_ip:=192.168.1.10
```

终端 3：ZED 外参，启动时自动读取最新标定。

```bash
ros2 launch calibration_result eye_in_hand_zed2i.launch.py
```

终端 4：D435i 外参，启动时自动读取最新且质量检查通过的标定。

```bash
ros2 launch calibration_result eye_to_hand_rs_d435i.launch.py
```

终端 5：仅抓取测试。

```bash
python -u src/graspnet-baseline/zedInhand_rsTohand/robot2human_handover.py \
  --manual --grasp-only --debug
```

- `--manual`：鼠标选物，跳过终端选物模式询问，无须大模型 API 密钥。
- `--grasp-only`：抓取并抬升后保持，不进入自动递送和松爪。
- `--debug`：保存视觉和候选数据，不代表禁止机械臂运动。

程序启动会尝试回 Home；夹爪服务启动会进行自动标定，包含闭合与张开。因此启动不是纯只读操作。当前 Socket 适配器的退出清理会重置夹爪激活；持物时不要把停止夹爪服务当作“保持夹持”的手段。

若需要将终端输出留档，可将终端 5 替换为：

```bash
mkdir -p logs
python -u src/graspnet-baseline/zedInhand_rsTohand/robot2human_handover.py \
  --manual --grasp-only --debug 2>&1 | tee logs/grasp_test.log
```

自动递送原入口去掉 `--grasp-only` 即可进入原流程，但该完整流程尚未验收，不能把本命令理解为已经验证可用。

### 5.1 每次测试前检查驱动与 TF

`ur_ros2_control_node` 曾两次静默退出（无 core、无错误日志），表现为 `base_link` 与 `zed_inhand_camera_frame_optical` 不在同一 TF 树，主程序所有抓取准备都报 `Camera orientation TF unavailable`。测试前先确认：

```bash
ros2 topic hz /joint_states                 # 应持续输出
ros2 run tf2_ros tf2_echo base_link tool0   # 应持续输出变换
```

若缺失，重启终端 1 的驱动。夹爪节点和两路静态外参可保持运行，不必重复启动。

## 6. 界面操作与当前流程

1. 程序回 Home，打开 RealSense Viewer 与 ZED Viewer。
2. 在 RealSense Viewer 中框选目标周围的工作区域，按 W。
3. 程序根据 D435i 深度与外参计算观察位置，以关节运动让 ZED 前往观察。当前按工作区中心引导，不是自动跨相机物体匹配。
4. 在 ZED Viewer 中选择待抓物体；SAM2 稳定后进行 FFS、GraspNet、方向与实体夹爪筛选。
5. 最新代码进一步检查接近、抓取和指定抬升端点的逆解及控制器安全限制，仅通过者显示 READY。
6. 审视两路画面后，在相机窗口按空格执行。没有现成候选时，空格仅触发准备；必须显示 READY 后再次按空格。
7. 夹爪闭合、沿接近方向退开，再沿控制器基座 Z 抬升。仅抓取模式进入 `GRASP_TEST_HOLD`，保持夹持。

| 按键 | 行为 |
| --- | --- |
| W | 根据 D435i 选定工作区前往 ZED 观察位置 |
| 空格 | READY 后抓取；未准备时只请求生成候选 |
| R | 空闲时清除选择并允许重新选物 |
| 0 | 空闲、ERROR、抓取测试保持或等待右手时：张开夹爪并回 Home；用于人眼发现抓取失败但程序未识别的情况 |
| Esc | 相机或 Open3D 窗口聚焦时，停止机器人程序并退出；不回 Home、不主动松爪 |
| Q | 空闲或抓取测试保持状态下正常回 Home，再退出；不主动松爪 |

D435i 与 ZED 当前均锁定一次选择，避免单击聚焦窗口时追加第二个分割对象。D435i 按 W 清空跟踪缓存后仍保持锁定；R 或返回 Home 开始新轮次才解锁。最新 D435i 锁定修改尚未做真机界面回归。

READY 最长保留 30 秒，接受空格时还检查双相机新鲜度、机械臂位姿是否变化及是否静止。没有独立的物体位移检测。端点可达性检查也不等同于完整路径避障。

Esc 退出主测试程序后，独立启动的四个 ROS 服务仍可能继续运行。不要由文档推断当前物理状态；下一轮启动前以现场和程序状态为准。

## 7. Home、抬升与夹爪模型设置

Home 来自用户现场调整，约为 `[0°, -90°, 0°, -90°, 0°, 0°]`。正式读取 `config/robot_home.json`，旧的 `config/d435_home.json` 仅作历史记录，不生效。Home 可能处于笛卡尔奇异姿态，回位使用关节运动。

当前运动设置：

```json
{
  "post_grasp_lift_height_m": 0.2,
  "grasp_move_speed_m_s": 0.1,
  "grasp_move_acceleration_m_s2": 0.2
}
```

当前 20 cm 表示最终 TCP 的基座 Z 相比抓取 TCP 增加 0.20 m，不是物体底部离桌面 20 cm。8 cm 接近退离距离与该参数独立。40 cm 在当前远端抓取位置超出可达范围，恢复前需先重规划抬升路径。

启动、正常完成和超时会尝试回同一 Home；右手等待超时保留夹爪状态，抓取执行异常则先张开夹爪再回 Home。右手等待超时 30 秒，跟随交接阶段超时 60 秒。`0` 键可在人眼发现抓取失败时主动张开并回 Home。断连、控制脚本停止、安全停或仍在运动时可能拒绝回位并进入 ERROR；Esc 明确不触发回位。Home 路径尚未完成完整环境避障验证。

GraspNet 权重未更换。实体 F85 网格替换了显示和碰撞几何；预测的简化抓取表示通过真实指垫中心映射到活动 TCP。筛选包含两种对称方向、闭合采样和接近采样，保留 85 mm 最大开口、8 mm 开口余量及原碰撞阈值。不能把“换实体模型”理解为模型重新训练过。

## 8. 实测结果与证据

| 记录 | 结果与边界 |
| --- | --- |
| `captured_data/physical_2f85_test/` | 实景视觉链路通过，1024 个原始候选最终保留 2 个实体夹爪候选；本次记录未执行动作 |
| `captured_data/single_grasp_test_02/` | 单次真机抓取小黄鸭成功；0.10 m/s，退离约 8 cm，OBJ=2，外部照片确认离桌；使用独立的受检单次测试脚本，不等同于当前主入口完整验证 |
| `captured_data/handover_test/release_result.json` | 用户明确确认托稳后，机械臂静止、仅张开夹爪完成交接；没有验证自动追手递送 |
| `captured_data/20261019_160318/` | 正确分割小黄鸭，5 个候选；后续掩膜切到其他区域，重试失败 |
| `captured_data/20261019_161105/` | 输入掩膜位于画面上方其他区域，不是小黄鸭；无方向合格候选 |
| `captured_data/20261019_161213/` | 视觉有 2 个候选；READY 后按空格，在运动前被抓取/抬升安全检查拒绝 |
| `captured_data/20261019_161328/` | 唯一方向合格候选宽约 68.5 mm，被实体指尖穿透检查拒绝 |
| `captured_data/20261019_161425/` | 最小候选倾角约 30.31°，全部未通过现有 30°方向限制 |
| `captured_data/20261019_165138/` | 唯一方向合格候选预测开口 85.3 mm，超过 77 mm 上限，无可用抓取 |
| `captured_data/20261019_165841/` | 方向合格 12 个、实体筛选 2 个，其中 1 个抬升无 IK；最终 READY 并执行 |
| 16:58 运行 | 夹爪已夹住物体（stalled），旧判定只看 reached_goal，误报 `Gripper close failed` 并回 Home；已修复 |
| 17:05 运行 | 抓取 + 20 cm 抬升成功：闭合 gap=0.038 m、effort=59.6 N、stalled=True；抬升后等待右手，30 秒内未检测到手，用户 Esc |
| 17:08–17:20 运行 | `ur_ros2_control_node` 静默退出，无 `/joint_states`，TF 两棵树断开，所有抓取准备报 `Camera orientation TF unavailable`；重启驱动后恢复 |

ROS 日志位于 `/home/ros2/.ros/log/`。READY 后无动作的关键日志：`python_40866_1792397500148.log`，含 `Grasp execution failed: Grasp/lift pose exceeds controller safety limits`。

后续新采集的 `grasp_filter_info.json` 额外记录网络原始数、NMS 数、方向筛选数、端点拒绝原因及抬升高度。旧字段 `raw_count` 实际记录 NMS 后数量，分析历史数据时不要把它当作网络原始数量。

## 9. 已知问题与尚未完成的工作

### 9.1 抬升高度与当前远端抓取位置的可达范围

最新一次（16:23）候选的控制器 TCP 端点为：

- 抓取：`[-0.43526, -0.23029, -0.02437] m`
- 接近：`[-0.45172, -0.21516, 0.05244] m`
- 40 cm 抬升：`[-0.45172, -0.21516, 0.37563] m`（真机 IK 拒绝值 `[-0.451734, -0.215161, 0.375636]`）

保持该 XY 与朝向，离线标定 URDF 多初值 IK 对抓取、接近残差为 0，对 40 cm 抬升最小残差 5.7 cm；该姿态下 TCP 最高约 0.26 m，即从抓取 TCP 可用抬升约 0.28 m。旧记录的 40 cm 端点同样无解。用户已确认 20 cm 可用，当前配置改为 0.20 m；离线核对 20 cm 抬升端点（z≈0.176 m）残差为 0。

上述仅为离线运动学结果，控制器安全限位与完整路径仍需真机受控验证。若以后恢复 40 cm，需要评估向基座收回后抬升等方案及完整路径，不要只增加 Z 或直接放宽限制。

### 9.2 完整交接仍有差距

- D435i 目前引导工作区观察，仍需在 ZED 中再次选物；尚无同一物体的跨相机自动关联。
- 原自动递送使用右手检测、跟随与握拳触发，递送端点直接依赖掌心坐标；物体尺寸、指垫与手掌的接触间距尚未系统处理，不能将先前静止松爪视为该流程验证。
- 新 Home、UI、Esc 和最新端点预检查主要改在机器人递给人入口，另一个方向尚未统一。
- 无解位姿会让 ur_rtde 的 URScript `get_inverse_kin()` 终止控制脚本；16:23 的停止符合这一机制。16:45/16:51 又在观察位 IK 查询处出现脚本停止，但该位姿离线枚举出 4 组精确解，外部触发仍未确定。现 `ensure_running()` 会自动尝试恢复脚本；`inverse_kinematics()` 先用 `getInverseKinematicsHasSolution()` 探测（无解不会停脚本），重传后等待 1 秒再发命令。机器人 PolyScope 为 5.12.4，支持该接口。以上只做离线检查，未真机验证。
- 16:51 的唯一方向合格候选预测开口 85.3 mm，超过 `85 − 8 = 77 mm` 的允许上限，被开口检查剔除，因此该次没有可用抓取位姿。`filter_grasps()` 现会记录 `width_rejected` / `clearance_rejected` 数量，便于区分“没有候选”和“候选被筛掉”。
- 夹爪闭合用物体接触（adapter 的 `stalled=True`）判定成功，不再只依赖 `reached_goal`；夹持力默认 60 N（原 140 N）。16:58 的抓取实际已夹住物体但被旧判定当成失败，尚未在修复后重测完整交接。
- 有效候选数受观察位置、分割、深度、方向及碰撞筛选影响。未为获取候选而放宽物理筛选。
- 抓取成功率偏低：小物体抓取仍会出现碰撞（用户现场观察）；大物体所需开口常超过 `85 − 8 = 77 mm`，被筛选后没有候选。碰撞筛选只使用当前相机点云，不包含未观测障碍或完整机械臂路径。
- 完整自动交接尚未跑通：17:05 抬升后 30 秒内没有检测到右手；跟随、握拳触发、下降到掌心、松爪和回位还没有一次完整成功的真机记录。

### 9.3 下一位接手者的工作顺序

恢复开发时先阅读本文和 AGENTS.md，再按用户选定的范围推进：

1. 优先提高抓取成功率：分别针对“小物体碰撞”和“大物体超过 77 mm”做最小测试，检查观察位、分割掩膜、候选方向与碰撞筛选，而不是直接放宽阈值。
2. 每次测试前检查 `/joint_states` 与 TF；驱动静默退出时先重启终端 1。
3. 验证右手交接：抬升后 30 秒内把右手放入 D435i 视野，确认 `Physical Right hand` 骨架出现；依次验证跟随、握拳、下降到掌心、松爪、回位。
4. 若恢复 40 cm，先规划“向基座收回后抬升”等可达路径，不要只增加 Z。
5. 最后统一 `human2robot_handover.py` 与 `robot2human_handover.py` 的 Home、READY、`0` 键和脚本恢复逻辑。

不需要重新安装环境、重训模型、重跑全量测试或重新迁移资产。修改及验证只围绕恢复工作的具体目标进行。


## 2026-10-20 交接绕转修复（仅离线验证）

本节覆盖上文旧交接流程说明。最新真机日志在检测右手后执行固定关节交接姿态时出现 `Joint motion failed`；用户确认碰撞后自动停下，最后由用户按 `0` 松爪回 Home。未记录动作起点关节角，不能确定实际绕转角度。

- 右手持续握拳 0.7 秒开始交接；本次握拳不会同时触发递送，进入跟随后须先松拳再握拳。等待仍为 30 秒。交接移动期间继续绘制骨架。
- 删除固定关节交接姿态和底座角归一化移动；从抬升后的实际姿态直接进入短段直线跟随，保持夹爪朝向。
- 跟随及递送路径按不超过 1 cm 间隔预检查 IK，以前一点为初值、选择最近等价关节角并检查控制器关节/位姿限位。软件保护为单采样关节变化不超过 15°、相对抬升姿态关节变化不超过 90°；这些是拒绝运动的保守策略，不是硬件限位。实际执行仍用 moveL，采样关节值不作为 moveJ 目标。
- 交接运动失败、路径拒绝或等待/跟随超时进入 ERROR，取消后续交接，不自动松爪或回 Home；人工 `0` 操作保留。抓取阶段的原异常处理与正常交接完成回 Home 保留。
- 语法及提取实际方法的离线模拟检查通过；未真机验证。采样 IK 检查不构成路径避障或奇异性证明，看不到的障碍、持物几何及正常回 Home 路径仍未建模。

### 14:04 跟随仅移动一段后的修复

`logs/grasp_test.log` 显示首次握拳成功启动跟随，第一段执行后，第二段因 `Handover must preserve the lifted gripper orientation` 进入 ERROR，之后握拳不再触发；最后仍为用户按 `0` 松爪回 Home。原检查将实际 TCP 与目标朝向按 1e-4 rad（约 0.006°）严格比较，混淆了测量/跟踪偏差和修改目标朝向。

现固定目标始终与交接参考比较，实际 TCP 独立采用 0.5° 软件跟踪容差；超限仍拒绝。采样检查插值实际朝向到固定目标的小角度修正，递送沿用同一目标朝向。ERROR 时继续显示骨架，并在 D435i 画面保留停止原因与“手势已禁用”提示。以实际方法离线模拟含 0.017° 实测偏差的连续两段跟随通过，过大偏差、目标旋转、IK 跳变、无解和安全状态异常仍拒绝。此修改未真机验证；日志未记录被拒绝时的实际角度，不能据此声称已测得真机跟踪误差。



### 当前操作入口（已移除独立诊断模式）

`--test`、独立诊断脚本及其生成的诊断目录已删除。正常运行继续使用 `--manual --debug`；仅抓取使用 `--manual --grasp-only --debug`。原有 `captured_data/<时间>/` 抓取点云、掩膜、候选等记录保留本地，不提交 GitHub。

IDLE 时 R/r 可在推理期间取消选区并重新选择，旧推理结果作废；新推理等待旧推理结束。两个相机窗口支持保持比例缩放。交接预检查保持原静止速度阈值，最多等待 1 秒反馈稳定。RTDE 初始化失败时在加载相机/模型前明确退出，避免空连接进入工作区动作；Ctrl+C 清理避免重复关闭 ROS。以上修复仅离线验证。
