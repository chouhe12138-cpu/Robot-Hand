# 本机环境与迁移记录

文档定位（2026-10-21）：本文件保留环境说明及 10 月 19 日迁移、逐次测试的历史记录；当前流程、参数、标定选择和待办统一见 [PROJECT_HANDOVER.md](PROJECT_HANDOVER.md)。历史段落中的“本次”“当前”“尚未同步”均指当时版本，不代表最新状态，也不延续硬件授权。

2026-10-19 从 `/home/ros2/Desktop/Object-Hand` 迁移。本项目根目录为
`/home/ros2/Desktop/Robot-Hand/UR3e_Handover-master`。

## 每次使用

```bash
cd /home/ros2/Desktop/Robot-Hand/UR3e_Handover-master
source activate_handover.bash
```

脚本加载 ROS 2 Humble、已有独立 Conda `urho` 环境、本项目 ZED SDK、模型代码路径和 ROS 包。
无需激活旧仓库中的 `.venv`，无需重新安装模型依赖。缓存存于本项目 `.cache/`。
`requirements-local.txt` 是安装依赖列表，`requirements-local-constraints.txt` 保留兼容版本约束，
`requirements-local-lock.txt` 记录迁移时实际使用的 `urho` 环境；它是本机快照，不是跨机器安装脚本。

## 文件位置

- GraspNet：`src/graspnet-baseline/logs/log_rs/checkpoint-rs.tar`
- SAM2：`src/SAM2_streaming/checkpoints/sam2.1/sam2.1_hiera_small.pt`
- FFS：`src/Fast-FoundationStereo/weights/23-36-37/`，包含权重和配套 `cfg.yaml`
- 本地 ZED SDK 和相机内参：`.zed-sdk/`；主脚本通过 `optional_settings_path` 使用本地内参目录
- 原始采集、求解结果与历史验证报告：`calibration_data/`
- 机器人出厂运动学标定：`config/ur3e_factory_calibration.yaml`
- 旧项目的夹爪/TCP 配置及初始姿态记录：`config/robotiq_2f85.json`、`config/d435_home.json`

`config/robotiq_2f85.json` 由实体夹爪模型加载；`config/d435_home.json` 仅保留初始姿态记录，交接脚本不会自动加载它。
原始记录中的 `source_session` 等绝对路径保留历史信息，文件已移动到本项目对应相对路径。
旧仓库的 `.venv`、构建产物和实验程序未迁移；本项目使用独立的 `urho` 环境和新编译的 ROS 包。

## 生效的手眼标定

启动文件在节点启动时选择最新完整 ZED 标定及最新质量通过的 D435i 标定；需重启对应节点才会读取新结果，不能由文件推断当前正在发布的 TF。帧名称保持与交接代码一致：

```bash
ros2 launch calibration_result eye_in_hand_zed2i.launch.py
ros2 launch calibration_result eye_to_hand_rs_d435i.launch.py
```

本次文档核对时最新文件（2026-10-21）：

- ZED：`calibration_data/zed_eye_in_hand_20261020_130247_132605/calibrated_result/zed_tool0_calibration.json`
- D435i：`calibration_data/d435_eye_to_hand_20261020_130813_896205/calibrated_result/d435_base_calibration.json`
- 仓库原始外参启动文件：`calibration_data/upstream_launch/`
- 旧仓库较早的本地外参启动文件：`calibration_data/legacy_launch/`

外参直接使用结果中的 `translation_m` 与 `quaternion_xyzw`，不修改坐标约定。
两份结果都未完成独立实物验证。ZED 标定记录的活动 TCP 为 tool0 沿 Z 方向 0.135 m，
使用前须确认控制器 TCP 和相机安装仍与采集时一致。同一 child frame 只能有一个 TF 发布者。

## 启动顺序

每个终端先执行上面的环境激活命令，再分别运行以下四条命令。机器人 IP 需要与主脚本的
`self.ur_ip` 一致；当前为 `192.168.1.10`。

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.1.10 launch_rviz:=false kinematics_params_file:="$PWD/config/ur3e_factory_calibration.yaml"
```

```bash
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.10
```

```bash
ros2 launch calibration_result eye_in_hand_zed2i.launch.py
```

```bash
ros2 launch calibration_result eye_to_hand_rs_d435i.launch.py
```

另一个终端激活环境后，人工选择模式无需 API 密钥：

```bash
# 在项目根目录，只运行需要的一个方向
python -u src/graspnet-baseline/zedInhand_rsTohand/human2robot_handover.py --manual --debug
# 或 python -u src/graspnet-baseline/zedInhand_rsTohand/robot2human_handover.py --manual --debug
```

主脚本会直接连接机器人，初始化期间可能运动；`--debug` 不会禁用运动。
若使用大模型选择模式，需自行配置 API 密钥与权限，勿提交密钥。历史环境检查和后续真机测试分别记录在下文。

## 迁移时环境限制（历史）

已确认 GPU 在沙箱外正常可用：NVIDIA GeForce RTX 4060 Laptop GPU，8 GB 显存，
驱动 580.126.09，PyTorch 2.6.0+cu124。沙箱内缺少 `/dev/nvidia*`，属于设备隔离，
不需要重装驱动。Codex 执行 GPU 任务需使用已获授权的沙箱外执行；本机终端按正常方式运行。
豆包 API 密钥和模型权限也需由使用者配置，未发起付费 API 请求。
当前使用文字或鼠标选物，暂不配置语音。自动选物时在终端选择文字提示；手动模式直接鼠标选物。

## 重新编译本项目 ROS 包

```bash
source activate_handover.bash
colcon build --packages-select calibration_result robotiq_2f_urcap_adapter \
  --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
```

## 迁移时离线检查结果（历史）

- PyTorch 2.6.0+cu124、OpenCV 4.11.0、MediaPipe 0.10.21，以及相机、RTDE、豆包 SDK、GraspNet CUDA 扩展导入通过。
- GraspNet、SAM2、FFS 权重 CPU 加载通过；沙箱外 GPU 检查也通过：PointNet2/kNN CUDA 运算、GraspNet 推理、SAM2 提示分割与跟踪、FFS 深度推理。
- GPU 检查使用合成输入（GraspNet 4096 点、SAM2 256×256、FFS 224×320/2 次迭代），不代表真实相机分辨率下的速度或完整交接已验证。
- `calibration_result` 和 `robotiq_2f_urcap_adapter` 编译完成，GripperCommand 类型支持加载通过。
- 激活脚本 Shell 语法和修改的 Python 文件语法检查通过。

## 历史开发与测试记录（2026-10-19）

以下内容保留当时行为和验证边界；当前共享筛选、朝向保留、异常策略及入口差异见项目交接文档。

### GraspNet 的实体 2F-85 模型

两个主交接入口现共用 `robotiq_grasp.py` 和从旧工作区复用的 `robotiq_2f85_model.py`。
实体网格、联动关节描述及许可证位于 `vendor/ros2_robotiq_gripper/`；这是描述资源，不启动其硬件驱动。
现有 GraspNet 权重及网络输出格式不变，训练所用简化抓取表示通过实际指垫中心映射到实体夹爪。

- Open3D 显示实体碰撞 STL 网格，包含掌部、联动手指和转接板。
- 用实际逐连杆凸包筛选得分靠前的最多 120 个候选，检查张开接近及闭合到预测宽度的采样状态。
- 保留配置的 85 mm 最大开口和 8 mm 开口余量，超过可用行程的候选不执行。
- 筛选同时检查平行夹爪的两个对称方向，执行仍选择离当前姿态较近的方向。
- 实际指垫中点对齐 GraspNet 的手指中点；按 11 mm 转接板和 135 mm 活动 TCP 计算目标位姿，替代原先将预测平移直接当作 TCP 的方式及经验 Z 偏移。

筛选使用当前相机观测到的完整场景点云，不能覆盖未观测障碍物，也不包含机械臂完整路径规划。
本次仅做针对性的离线几何检查：宽度与关节转换、指垫/TCP 对齐、真实掌部碰撞、显示网格位置及超行程拒绝；没有执行真机动作。

## 小黄鸭实景视觉测试（2026-10-19）

两台相机采集成功；ZED 实景经 SAM2、FFS 和 GraspNet 后得到 1024 个原始候选、65 个 NMS 候选、21 个方向合格候选，实体 2F-85 筛选保留 2 个。最佳候选预测开口 73.4 mm。数据及局部夹爪图位于 `captured_data/physical_2f85_test/`。修复 Open3D 调试截图未主动绘制的问题（`do_render=True`）。

本轮没有发送机械臂或夹爪命令。ZED 出现 self-calibration failed 警告；以上结果验证视觉链路和模型接入，不证明定位精度或真机抓取成功。

## 单次真机抓取（2026-10-19）

小黄鸭单次抓取已完成：执行速度 0.10 m/s、加速度 0.20 m/s²，闭合后沿接近方向退回约 8 cm；夹爪报告 OBJ=2，外部 D435i 图像确认小黄鸭离开桌面。结束时机械臂停止、夹爪保持夹持，未递物或自动松爪。执行数据位于 `captured_data/single_grasp_test_02/`。

执行前曾因像素亮度变化停止一次，当时未发送动作；随后使用曝光稳健的图像配准并重新生成、筛选计划完成本次抓取。该实物记录只证明此次场景成功，不代表全部交接流程通过验收。


## 固定 Home 与窗口抓取确认（2026-10-19）

本次接入范围为 `robot2human_handover.py`（机器人递给人）；`human2robot_handover.py` 的流程尚未同步修改。

- `config/robot_home.json` 记录用户摆好的真实关节位置，约 `[0, -90, 0, -90, 0, 0]` 度。程序读取弧度值；不沿用旧的 `config/d435_home.json`。记录时只读 RTDE，未运动。
- 启动、正常退出、交接完成，以及无有效点云/抓取候选、观察失败、抓取/递送异常时，尝试回到同一个 Home；等待右手 30 秒或交接阶段 60 秒超时也尝试回位。回位不会改变夹爪状态。
- 安全停、断连、机械臂仍运动等情况下禁止强制回位，进入 ERROR。Home 使用关节运动，避免对伸直姿态使用笛卡尔直线回位。
- 原有 D435i 工作区选择保留：在 RealSense 窗口框选工作区域，按 `w`，根据深度与外参计算 ZED 观察位置，以关节运动前往。无 API key 时可直接手动选择，不依赖语音。该逻辑按工作区中心引导，尚未新增所选物体跨相机自动匹配。
- 到观察位置后，终端选择 `2`（手动），在 ZED 窗口选择物体。SAM2 稳定后自动生成候选，只显示 READY，不执行抓取。两个相机窗口继续显示；Open3D 显示候选夹爪。
- 审视两路图像后，在任一 OpenCV 相机窗口聚焦时按空格执行；取消抓取前的终端 y/n 确认。尚无候选时空格只生成候选，须显示 READY 后再次按空格。
- READY 最长保留 30 秒；相机中断、机器人位置改变或仍在运动时不执行。期间物体仍须保持原位；不包含独立的物体移动检测。
- 空闲时 `r` 清除候选，`q` 正常回位退出；任务执行或推理期间禁止用这些键中断后台线程。紧急停止仍使用机器人实体急停。

验证：修改文件语法检查通过；以模拟 RTDE 验证 Home 成功回位、已到位不重复运动、安全停拒绝、运动中拒绝。没有运行新版本的真机运动、双相机 UI 或完整自动交接。机器人自身的关节安全限位检查不等同于环境避障；Home/观察位置的实际路径仍需真机受控验证。

## 抬高后的抓取测试

`config/handover_motion.json` 将抓取后 TCP 高度设为抓取 TCP 的基座 Z +0.20 m（当前值）；先按原 8 cm 接近路径退离，再竖直抬高，独立于夹爪朝向。抓取段速度 0.10 m/s、加速度 0.20 m/s²。该高度是 TCP 位移，不是物体底部到桌面的精确距离。

原先的 0.40 m 在 16:23 的远端抓取位置无 IK 解：离线标定 URDF 显示该姿态最高只能抬到约 0.26 m TCP（相对抓取 TCP 约 0.28 m）。用户确认 20 cm 可用于交接，故当前值改为 0.20 m；离线核对 20 cm 抬升端点可达。未做真机验证。

启动同一主程序时增加 `--manual --grasp-only`，即可使用鼠标选物、READY 后空格执行，并在抬高后保持夹持，不进入人机递送。处于 `GRASP_TEST_HOLD` 时，`q` 回 Home 并退出，仍不松爪。纯手动模式不再要求大模型 API 密钥。

本次参数的计算与入口加载检查不等同于抬高后的真机抓取成功；实物结果另行记录。


## 本轮失败定位与停止键修复

已停止本轮测试；停止时 RTDE 显示关节速度为零、机械臂在 Home，夹爪 POS=3、OBJ=3（打开、空载）。未执行抓取及 40 cm 抬升。

保存的三次视觉结果说明：`20261019_160318` 正确分割小黄鸭（3638 像素，边界 [377,226,457,288]），保留 5 个抓取候选；`20261019_160335` 的目标掩膜变成图像左边的大区域（21084 像素，边界 [45,0,180,359]），候选为 0；自动降低 10 cm 后的 `20261019_160343` 仍分割左侧区域且无候选。原代码允许追加第二个目标，选择函数优先最后添加的目标，因此误点可替换抓取对象。日志没有记录鼠标事件，不能断言具体哪次鼠标操作导致变化。后续 RTDE 控制脚本已停止，IK 查询失败；现有日志不足以确定控制脚本停止的来源。

修复：单次只允许一个 ZED 目标，选定后锁定；重新选择须先按 r。空格在推理期间不再取走异步发布的 READY 候选；无有效目标掩膜时禁止退回整幅点云猜测。无候选时取消自动降低 10 cm 重试，按正常失败回位。控制脚本已停止时明确报错，不将其误报成 Home 超出关节限位。

Esc 在任一 OpenCV 相机窗口或 Open3D 抓取窗口聚焦时停止机器人程序并退出，不回 Home、不主动松爪；退出标志阻止后台线程再发起运动、回位或发布 READY。q 保留空闲时正常回 Home 退出。软件停止通过独立 Dashboard 连接发送，避免等待阻塞的 RTDE 运动调用。

验证：语法检查及模拟停止/后台动作阻断/目标锁定检查通过；修复后未重启真机测试。


## READY 后无动作与 D435i 选区追加（本轮修复）

用户本机运行产生的 ROS 日志 `/home/ros2/.ros/log/python_40866_1792397500148.log` 记录：16:12 那次视觉保留 2 个候选，按空格后报 `Grasp/lift pose exceeds controller safety limits`，在打开夹爪或执行接近运动之前退出。该次 TCP 抓取位置为 [-0.446895,-0.211232,-0.011648] m，接近位置 [-0.463566,-0.196340,0.065166] m，40 cm 抬升位置 [-0.463566,-0.196340,0.388352] m。复用标定 URDF 做离线多初值 IK：接近和抓取找到解，保持该 XY 与朝向的 40 cm 抬升未找到解；不应将此配置直接当成可执行计划。40 cm 参数保留，没有擅自降低高度或移动机器人。

READY 原先只保证视觉候选通过，运动可达性检查太晚。现将候选的接近、抓取、抬升端点 IK 和控制器安全检查前移，仅通过者可显示 READY；执行使用同一份计划并再次检查。拒绝原因指出具体阶段，保存到已有 `grasp_filter_info.json` 中。端点检查仍不等同于完整轨迹避障。

D435i 原回调仍允许两个目标，上次只锁定了 ZED。现 D435i 在鼠标完成第一次选择时立即锁定，按 w 后清空跟踪缓存也不会解锁；只有显式 r 或返回 Home 开启新轮次才解锁，画面显示 LOCKED。禁止运动期间选区。

已做最小离线验证：D435i 连续误点不会新增目标、w 后仍锁定、显式重置可重选；无 IK 的抬升在运动前被拒绝。未重启真机测试。16:11 数据为非小黄鸭掩膜；16:13 唯一方向合格的 68.5 mm 候选被指尖碰撞检查拒绝；16:14 候选最小倾角约30.31度，均未通过原30度限制。本次未放宽碰撞或方向阈值。

## 无解 IK 查询与 20 cm 抬升（16:23 运行后修复）

16:23 运行第一次观察得到 18 个 NMS、方向合格 0 个；第二次得到 603 个网络原始候选、64 个 NMS、15 个方向合格、实体筛选后 1 个，该候选 40 cm 抬升无 IK 解，故未显示 READY、未发出运动。随后回 Home 报 `RTDE control script is not running`。

原因：ur_rtde 上传的 URScript 用 `get_inverse_kin()` 查询不可达位姿时会终止控制脚本。现新增 `RobotControllerNode.inverse_kinematics()`，所有 RTDE IK 查询（抓取计划预检查、观察位）统一经它调用；查询无解且检测到脚本已停时调用 `reuploadScript()` 恢复，避免失败后无法回位。未做真机验证。

`config/handover_motion.json` 同时从 0.40 m 改为 0.20 m；对 16:23 的同一候选离线复核，抓取、接近与 20 cm 抬升端点 IK 残差均为 0。

## 16:45 运行：恢复时序问题

16:45 运行在按 W 后记录 `Moving to dynamic start pose [...]`，约 20 秒后出现 `No-solution IK query stopped the RTDE control script; re-uploaded it.`，紧接着回 Home 仍报脚本未运行。该观察位姿用标定 URDF 复核 IK 残差为 0，故失败不是位姿不可达；`getInverseKinematics` 的 20 秒耗时说明查询时脚本已停或查询期间停止，具体外部触发仍未确定。

新恢复代码的缺陷是 `reuploadScript()` 后立即回位，脚本尚未变为 PLAYING。现改为 `ensure_control_script()`：查询 IK 前先确认脚本，必要时重传并轮询 `isProgramRunning()` 最多 10 秒；IK 查询后若脚本被停也会恢复；回 Home 前同样先恢复。语法检查通过，未真机验证。

## 16:51 运行：候选开口超限与脚本就绪时序

第一次观察 `raw=135, nms=18, orient=1, selected=0`。离线复现 `captured_data/20261019_165138/`：唯一方向合格候选预测开口 **85.3 mm**，大于允许的 `85 − 8 = 77 mm`，被宽度检查剔除；不是碰撞或 IK 问题。`filter_grasps()` 现记录 `kept`、`width_rejected`、`clearance_rejected` 数量。

第二次、第三次观察都在按 W 后出现 `RTDE control script is not running!`。`ensure_control_script()` 已重传成功，但紧接着的 `getInverseKinematics` 立即返回无效值，被误报为观察位不可达。对同一姿态用标定 URDF 枚举出 4 组精确 IK 解，均在关节范围内。反汇编确认 ur_rtde 1.6.3 的 `reuploadScript()` 只发送脚本，不等待控制循环就绪。

修复：

- 重传后等待 `isProgramRunning()` 为 true，再额外等 1 秒；
- `inverse_kinematics()` 改用 `getInverseKinematicsHasSolution()` 先探测；机器人 PolyScope 5.12.4 支持该接口（要求 ≥5.10）。无解时不再调用会终止脚本的 `get_inverse_kin()`；
- `ensure_running()` 在脚本停止且未被 Esc 停止时自动恢复，覆盖运动、夹爪和回位路径；
- 观察位失败信息区分“无 IK 解”和“关节安全限位拒绝”，并打印解值。

语法检查通过；以上均未真机验证。

## 16:58 抓取：夹爪闭合判定与夹持力

16:58 运行在 READY 后完成了接近、抓取位和闭合。日志随后报 `Gripper close failed`，但夹爪实际已夹住物体：adapter 只在“到达指令开口”时返回 `reached_goal=True`，被物体挡住时返回 `stalled=True, reached_goal=False`。原代码只检查 `reached_goal`，于是把成功夹持误判为失败并回 Home（保持夹持）。

修复：`send_gripper_command()` 增加 `allow_stall`，闭合调用改为 `allow_stall=True`，`stalled` 视为夹持成功，并记录最终 `gap/effort/stalled`。夹持力默认值从 140 N 调整为 **60 N**（adapter 允许范围 20–235 N）；若实测滑落可再调高。

大物体仍受 2F-85 行程限制：实际允许的预测开口上限为 85 − 8 = 77 mm，超过即被筛选；这是硬件限制，未放宽。

未做真机验证。

## 手动松爪回 Home（0 键）与抓取失败自动松爪

有些抓取失败只有人眼能发现（例如夹爪闭合但没夹住或物体滑脱），程序会误判为成功。现增加：

- 在任一 OpenCV 相机窗口按 `0`：张开夹爪（85 mm）后回 Home。允许状态为 `IDLE`、`ERROR`、`GRASP_TEST_HOLD`、`WAITING_HANDOVER_START_POSE`；正在准备抓取或其他运动阶段会拒绝并提示。
- `WAITING_HANDOVER_START_POSE` 下按 `0` 会设置 `handover_abort`，等待右手的线程不会继续移动机械臂；若已开始前往交接位，先等当前运动停止再执行松爪回位。
- `execute_grasp` 捕获到抓取执行异常时，先尝试张开夹爪，再回 Home（原先保留夹爪状态）。右手等待 30 秒超时仍保留夹持回 Home，不自动松开。

`handover_abort` 在下一次 `execute_grasp` 开始时清除。语法检查通过，未真机验证。
