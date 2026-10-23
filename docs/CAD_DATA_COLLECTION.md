# CAD 物体与人手遮挡点云采集

这是独立采集入口，不导入 UR RTDE、机器人运动控制或夹爪控制模块；**运行采集程序不会命令机械臂运动**。机械臂和相机的位置由操作员在程序外调整。脚本只读取 ZED 2i 左右图像。与已有 `zed_ffs_pipeline_dump.py` 一样，沿用当前 FFS 权重、ZED 内参和相机设置；SAM2 使用当前仓库的 SAM2.1 Small。

## 运行

在仓库根目录执行：

```bash
source activate_handover.bash
python tools/collect_cad_samples.py --object-id cat_06 --mode object
python tools/collect_cad_samples.py --object-id cat_06 --mode handover
```

可用 `--output /path/to/dataset` 指定数据集存放位置。默认目录是 `collected_cad_data/`，建议将它放到 Git 忽略规则中或指定外部存储盘；不要上传大体量原始数据或私人人手照片到 GitHub。

## 采集键位

1. 手动调整相机/机器人到合适的静止观察姿态。确保不会与相机取帧程序同时打开 ZED。
2. 在相机预览窗口按 **空格**，冻结一对同步左右图像；之后的点击/保存都处理这同一帧。
3. **左键**单击物体可见表面作为物体正提示。选择 `--mode handover` 时，还需 **右键**单击清晰可见的手部表面作为手正提示。手和物体互为负提示，复用 `h2r_interaction.manual_segmentation_prompts`。目前是手动点击，不依赖自动识别手部身份；也尚不支持鼠标矩形框选。
4. **S** 运行 SAM2/FFS、保存当前样本。**R** 清除提示重选，**C** 取消冻结继续预览，**Q** 或 **Esc** 退出。
5. 每次保存完程序恢复实时预览；调整位置后继续空格采集。

采集失败不会保存为有效样本；成功保存也不等于分割一定正确。建议现场打开 `mask_object.png` 与 `cloud_object.ply` 抽查。

## 文件组织

```text
collected_cad_data/
  cat_06/
    object/                         # 无人手遮挡
      20261024_120000/             # 一次运行的 session
        20261024_120015_123456/   # 一次按 S 保存的 sample
          rgb_left.png
          rgb_right.png
          disparity_px.npy          # float32 像素视差，包含无效值
          depth_mm.png              # 16-bit 毫米深度，0=无效
          mask_object.png           # 0/255 掩膜
          cloud_scene.ply           # 所有有效深度，ZED 左相机光学坐标系
          cloud_object.ply          # 仅物体掩膜内有效点
          metadata.json
    handover/                       # 人手持物
      <session>/
        <sample>/
          ...上述文件...
          mask_hand.png
          cloud_hand.ply
```

`metadata.json` 记录物体标识、采集模式、时间、提示点、图像分辨率、相机内参矩阵 `K`、双目基线、点数、坐标系与单位。每组原始图像、深度、掩膜、点云来自一次冻结帧，不会混用相邻时刻的图像。

**精度与标注边界：**
- 点云均位于 ZED 左相机光学坐标系，以米计。没有把手眼标定变换强行写成当前真实外参；未读取机械臂关节角或末端位姿。
- 已知 CAD 模型的 ID 由 `--object-id` 记录；CAD 文件路径和 CAD→相机的 **6D 位姿真值未自动生成**。只凭 RGB-D 快照不能保证获得真实位姿 GT。未来可在附加标定或人工标注后扩展。
- 采集的 SAM2 Mask 是模型预测，**不是人工 GT**；被手遮挡的物体背面不能从该帧得到真实点云。
- 遮挡模式请将左键点选在物体可见表面，右键点选在物体外的可见手部区域，避免手物接触边缘。
- 当前程序对每次按 S 的静止图像运行 FFS 和 SAM2，可能有短暂停顿，不适合高速连续录制。

## 与主项目关系

本采集器不调用 GraspNet、不运行 H2R/R2H 状态机、不执行夹爪动作，避免采集逻辑与实际交接混合。已有的 `tools/collect_cad_samples.py` 可继续扩展不同物体 ID 和采集模式，不需要复制整份脚本。
