# LINEMOD Cat CAD：最小几何配准测试

此工具仅检验 **已知 CAD → 局部/遮挡点云配准**。不会连接相机、机器人、夹爪，不进行 GraspNet 推理，也不能单独证明抓取成功率提高。

## 输入

- 官方 LINEMOD / BOP 模型（通常 Cat 对应 `models/obj_000006.ply`；以你的文件夹和 `models_info.json` 为准）。
- 网格可以纯白、无纹理。**FPFH + RANSAC + ICP 使用 XYZ/法线，不需要 RGB 颜色**。
- `--cad-scale 0.001` 是 BOP 常见毫米转米设置。必须按实际文件单位校验模型尺寸。

## 运行（本地已安装 Open3D 的环境）

```bash
python -m pip install numpy open3d
python cat_registration_probe.py --cad /path/to/obj_000006.ply --show
```

如果已经拍摄到**与该 CAD 几何完全一致**的实体 Cat，且有只包含 Cat 的局部点云（XYZ，单位米），可运行：

```bash
python cat_registration_probe.py --cad /path/to/obj_000006.ply \
  --observed /path/to/cat_object_only.ply --show
```

生成 `cat_probe_out/observation.ply`、`aligned_cat.ply`、`cad_to_observation.txt`。可查看覆盖率、几何拟合误差；合成模式还有已知位姿的角度/平移误差。部分遮挡严重时，传统 FPFH 匹配可能失败，不代表你的未来学习式配准网络必然失败。

## 判断 CAD 是否有助于抓取（再往下一步）

1. 同一真实场景中固定相机、物体及 GraspNet 权重，记录目标物体的原始部分点云和真实 CAD。
2. 采用同一候选生成及夹爪筛选管线，比较原始点云、配准 CAD 点云（建议还比较仅用 CAD 校核预测开口）产生的候选。保持相同采样预算、碰撞检查、77 mm 有效开口和执行约束。
3. 观察候选位置、预测开口、碰撞后剩余数和 CAD 接触几何，不把「点数更多」当成必然更好。最后若要比较真机成功率，再设计独立受控测试。

## 真实物体验证的前提

- 需要 **几何与 Cat 网格一致** 的真实物体，例如按正确尺度打印的同一模型。单纯也是「猫形」但造型不同的玩具不能当成匹配 CAD。
- CAD 无颜色不影响点云几何匹配；但均匀白色或无纹理实体在 ZED 双目深度上可能存在空洞/噪声，仍需测量其点云质量。可以先通过 D435i 获取几何验证样例。
- 配准初值、模型对称性及遮挡会影响 FPFH/RANSAC；报告误差和失败样例，不只报告一次视觉上重合。
