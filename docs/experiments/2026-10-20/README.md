# 2026-10-20 真机实验记录（精选）

本目录只保留 4 组代表场景、压缩截图、少量候选数组和日志摘录。补充保留各组目标区域原始点云，以及最新 H2R 的完整场景点云；其他完整点云、深度、数据集和完整运行目录留在本地。当前使用的 GraspNet 权重另随仓库保留（见下文）。截图 JPEG 为缩小后的观察材料，不用于精确像素或标定计算；候选数组保留原始数值。样本是人工挑选，不能用于统计成功率。

## 今日结论

R2H 桌面抓取成功率据操作者观察有所改善，但完整递送仍不稳定。H2R 已能无 API 启动，使用物理左手引导，人工选择手和物体，并生成候选；完整可靠接物尚未验收。今天停止继续修改功能，保留现场问题以便后续处理。

## 精选记录

| 目录 | 观测结果 |
| --- | --- |
| [r2h_155343_grasp_and_handover_stop](r2h_155343_grasp_and_handover_stop) | 网络 1024 → NMS 31 → 方向 11 → 物理朝向 8 → 端点筛选 4；日志记录夹爪接触、抬升并进入手势等待。随后递送触发关节分支跳变/步幅保护，未完成交接。不能仅凭夹爪接触日志认定稳定抓取成功。 |
| [r2h_155252_ik_rejected](r2h_155252_ik_rejected) | 8 个物理朝向全部在接近位姿 IK 检查被排除，未执行抓取。具体原因见 JSON。 |
| [h2r_175155_no_candidates](h2r_175155_no_candidates) | 手握瓶子；NMS 后 31 → 方向 5 → 物理筛选 0（宽度排除 3、间隙排除 2），未执行抓取。掩膜大致覆盖瓶身可见区域。 |
| [h2r_180035_ready_gripper_timeout](h2r_180035_ready_gripper_timeout) | 最新一轮：NMS 后 98 → 方向 33 → 物理筛选 3，进入 READY；之前同轮一次计算为 80 → 33 → 2。按确认后夹爪打开请求 8 秒未获 Action 接收确认，报 Gripper open failed，随后回 Home。此次最终中止不能归因于候选全部被排除。 |

原始日志摘录：[H2R 最新一轮](h2r_latest_excerpt.txt)、[R2H 抓取/交接](r2h_155343_excerpt.txt)。历史日志中的 0.7 秒手势阈值是当时配置，当前代码已改为 2 秒。`raw_count` 在这些 JSON 中表示 NMS 后候选数；R2H 另存 `network_raw_count` 表示网络原始数量。

## 操作者观察与待解决问题

- H2R 最后点选/框选明显卡顿，选择顺序容易混乱，SAM2 有时把人体也包含进掩膜。现场反馈与最新图像应结合看，不能认为所有样本都误分割。
- 最新平放掌心的呈现方式较手握物体更容易观察，但腕部 ZED 只能看到部分目标。相机位于夹爪上方，存在“相机看到物体、夹爪却在手掌下方”的现象。尚未据此证明外参错误，不能凭经验改标定或 TCP。
- 引导取 MediaPipe 第 9 号关键点（中指根部，并非严格掌心中心）对应 D435i 深度。握物遮挡时可能采到物体或背景；当前还持续跟手，TCP 目标与手同高，XY 向底座偏 15cm。IK 检查不等于完整肘部/底座碰撞检查。此前操作者实际观察到手太低导致肘部碰底座。
- 今日其他跟手轮次在 Y 约 -0.67～-0.69m 处出现路径 IK 无解，发生在生成抓取位姿之前，不能与夹爪抓取失败混为一谈。
- 夹爪 Action 接收确认超时仍未查明；RTDE 脚本停止与夹爪 Action 属于不同控制链路，需要结合示教器及驱动日志定位。
- 最新一轮连续生成两次候选并两次 READY，窗口操作和计算排队仍需梳理；本次不继续修改。

## 当前代码行为

H2R：不依赖 API 的 manual 模式；启动采用共享 Home；工作区几何、30cm 高度/10cm 向底座偏移及关节引导参数与 R2H 对齐。工作区完成后按空格启动左手引导；跟手时可直接在 ZED 左键单击/拖框，停止追加跟手运动，等当前运动完成后接收选择。先手、后物体，二者齐全才初始化分割；框和点保留显示。READY 后空格执行。

H2R 的 0 为显式打开夹爪并回 Home；夹爪未确认时提示可能仍夹着物体，继续尝试回位。仅控制器正常且机械臂静止才允许显式重载停止的 RTDE 脚本；不自动解除保护停止。恢复请求一次消费，避免每帧重试。

R2H：右手握拳/张掌保持时间均为 2 秒；握拳后等待张掌，再使用一次手部位置进行递送，不再连续追手。完整递送准确性仍需后续验证。

本次归档只做 Python 语法、差异检查及此前最小模拟检查，没有为归档启动相机、机械臂、夹爪或重新执行 GPU 推理。以上真机结果来自操作者当天运行的记录。

## 点云与 GraspNet 权重补充

- 每组 `pointcloud_workspace_used.ply`：原始掩膜与有效深度筛选后的 XYZ/RGB 点云，未降采样；这是送入 GraspNet 采样前的目标区域点集，并非网络实际抽取的固定数量点。
- 最新 H2R 组另有 `pointcloud_full.ply`：同帧全部有效深度点，可查看物体、手和人体的空间关系。其余三组仅保留局部点云以控制体积。
- 点云和 `grasp_before_filter.npy` / `grasp_after_filter.npy` 都在该帧 ZED 光学相机坐标系，XYZ 单位为米，不应再施加机械臂基座变换后相互叠加。候选数组是 GraspNet 的 Nx17 格式，可由 `graspnetAPI.GraspGroup` 读取。before 是 NMS 后、方向/物理筛选前；after 是最终筛选结果（R2H 还包含端点可达性筛选）。空 after 表示全部被排除。
- 当前实际使用的权重：[checkpoint-rs.tar](../../../src/graspnet-baseline/logs/log_rs/checkpoint-rs.tar)，原文件 12,468,415 字节，保留原路径，可直接被 H2R/R2H 的 checkpoint 解析函数找到。SHA256：`60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`。仅提交这一份 GraspNet checkpoint，未提交 SAM2/FFS 权重。

在项目根目录激活环境后，可离线叠加查看（不运行交接主程序、不连接机器人）：

```bash
source activate_handover.bash
python - <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, str(Path('src/graspnetAPI').resolve()))
import numpy as np
import open3d as o3d
from graspnetAPI import GraspGroup

sample = Path('docs/experiments/2026-10-20/h2r_180035_ready_gripper_timeout')
cloud = o3d.io.read_point_cloud(str(sample / 'pointcloud_full.ply'))
# 查看筛选前候选时，改成 grasp_before_filter.npy；默认只画前 20 个，避免遮挡。
grasps = GraspGroup(np.load(sample / 'grasp_after_filter.npy'))
o3d.visualization.draw_geometries([cloud, *grasps[:20].to_open3d_geometry_list()])
PY
```

这些局部点云可用于候选可视化、物理筛选及 GraspNet 点云输入的离线测试；完整复现 FFS 深度重建或 SAM2 分割仍需本地原始双目图像、深度及对应依赖，精选附件不代表完整回放包。
