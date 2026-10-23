# 已知 CAD 物体的真实 RGB-D / 点云采集

本工具是**只采数据**的独立入口：不连接 UR RTDE Control、不打开夹爪、不执行 GraspNet、不发送 ROS 运动命令。实验员自己使用示教器调整 UR3e 或手动摆放物体；程序从 **ZED 2i** 读取校正后的双目图像，按空格后使用现有 FFS 和 SAM2 生成数据。默认不要求机器人、ROS2 或 D435i 在线。

## 启动

在项目根目录：

```bash
source activate_handover.bash

# 常规 CAD 物体：在 ZED 左图单击/框选物体，空格采集
python tools/collect_cad_samples.py --mode object --object-id linemod_cat \
  --cad /path/to/obj_06.ply --cad-unit mm

# 左手握持 CAD 物体：自动手提示 + 手动物体提示
python tools/collect_cad_samples.py --mode handover --object-id linemod_cat \
  --cad /path/to/obj_06.ply --cad-unit mm
```

有实际已经在运行的 ROS2 UR3e driver 和 TF 标定发布时，可以附加 `--with-tf`，将 `base_link -> zed_inhand_camera_frame_optical` 变换记录在每个 `meta.json`。也可附加 `--robot-ip 192.168.1.10` **只读** RTDE 关节角与当前控制器 TCP（无需启动交接主程序）。这两个参数都是可选的；未开启就保留相机系点云。不要同时运行会占用 ZED 的交接程序。

需要机器人多视角数据的**统一基座坐标系**时，建议启用 `--with-tf` 并检查已经生效的 ZED Eye-in-Hand 标定。变换使用 TF 的最新值，未完成硬件时间同步，不构成严格同步的高精度位姿 GT。没有物体在基座下的独立真实位姿标注，因此即使有 TF，也不能将它当作 CAD→相机的真实 6D 位姿标签。

## 操作

1. 自己摆好相机/机械臂，等待 ZED 画面稳定。
2. 在 `ZED CAD Capture` **左图（640×360）** 单击物体，或拖框紧贴物体；在 `handover` 模式下只框物体，不要把手掌一起框进去。
3. `SPACE`：冻结当前左右图，顺序运行 FFS 与 SAM2，保存该帧全部数据。`Last capture Mask` 窗口显示分割覆盖结果供人工检查。
4. `r` 清除选区；每次成功采样后也自动清除提示，避免换姿态后沿用旧位置。`q` 或 `ESC` 退出。
5. `handover` 模式默认使用 MediaPipe 检测**物理左手**：该仓库未镜像 ZED 输入下的标签是 `Right`。如关键点错选或检测失败，按 `h`，再**点击手部露出的区域**，即可覆盖本轮自动手部提示；按 `o` 重新切换到物体选择。手部点应在物体框外。如果物体框包括所有手部候选点，将提示错误并保留原始双目数据。

此采集器每个样本独立初始化 SAM2，因此不依赖跨帧的视频跟踪状态，适合人工改变视角、记录不同遮挡程度的数据；不意味着 Mask 是真实的人工标注 GT。特别是手物接触时应人工检查 `mask_preview.png`。若失败，该次原始左右图仍会保存，方便以后重新分割。没有自动物体识别、抓取执行或 CAD 配准。

## 数据组织

默认输出位于仓库被 Git 忽略的 `captured_data/cad_samples/`。通过 `--output-root` 可以改为独立数据盘。每次运行新建一个 session：

```text
captured_data/cad_samples/
└── linemod_cat/
    └── 20261024_103000/
        ├── sample_0001_20261024_103015_123456/
        │   ├── left_full.png, right_full.png  # 1280×720 校正双目图
        │   ├── left.png, right.png            # 640×360，FFS 和 SAM2 用图
        │   ├── depth_mm.png                   # 16-bit 深度，0 为无效
        │   ├── disparity_px.npy               # float32 视差，像素
        │   ├── pointcloud_full.ply            # 全部有效深度点，ZED 左光学系，m
        │   ├── mask_object.png                # 物体二值 Mask，0/255
        │   ├── pointcloud_object.ply          # Mask 内的原始物体点云，m
        │   ├── mask_hand.png                  # handover 模式，成功时才存在
        │   ├── pointcloud_hand.ply            # handover 模式，成功时才存在
        │   ├── mask_preview.png               # 人工检查叠加图（不要当 GT）
        │   └── meta.json                      # 采集参数、提示、统计及坐标变换
        └── sample_0002_.../
```

所有 640×360 深度、Mask、视差和 `left.png` 像素一一对应；`left_full.png` 不可以直接与这些 640 掩膜作逐像素与操作。点云未做体素降采样、重复采样或 GraspNet 的降噪，因此可独立用于后续配准网络，不会将 2,000 个真实点重复填充成 20,000 个“新点”。`pointcloud_full.ply` 和 Mask 点云均保存 RGB 颜色和真实尺寸（米）。

`meta.json` 记录本次对象 ID、CAD 文件路径及单位、采集时间、ZED 图像时间戳、左右校正图内参、双目基线、FFS 参数、实际分割提示（正/负点/框）、Mask 和点云点数、可选 TF / 关节信息。**CAD 仅建立物体身份关联，不会自动标注 CAD 的物体位姿。** 如果需要 6D 位姿 GT，后续需另建经验证的标注/配准程序，不可将推断位姿冒充 GT。

如果某个样本分割错误，原始双目、深度、完整场景点云依然保留；`segmentation.status` 和 `error` 描述是否成功，不会将错误结果静默标记为真值。模型推理失败也会尽量保存当次采集图像；具体错误记录在 `meta.json`。

## 实验记录建议

建议对同一 CAD 物体分别采集无遮挡和手持遮挡的样本；每个对象保存不同方位、距离及接触位置，并根据 `mask_preview.png` 人工区分可用和错误分割。采集时不要在机械臂运动中按空格，以免双目时间同步、TF 位姿及运动模糊造成不必要的误差。

依赖沿用仓库已有 `urho` 环境及本地 FFS/SAM2 权重，**无需升级或重装**。代码只做必要的离线语法/数据写入验证，ZED/FFS/SAM2 的真实运行须在实验室完成。
