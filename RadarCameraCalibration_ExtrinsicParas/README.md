# 雷达相机外参标定（纯 C++）

该目录提供一个独立的 C++ 外参标定工具，不依赖 ROS2，流程为：

1. 通过海康 MVS SDK 连接相机并读取配置参数（曝光、增益、帧率等）
2. 手动 n 点选取图像坐标（运行时输入 n）
3. 调用 `solvePnPRansac + solvePnP(ITERATIVE)` 计算外参
4. 写入 `config/out_matrix.yaml`

## 配置文件

- `config/camera_params.yaml`
  - 相机内参 `camera_matrix`、畸变 `dist_coeffs`
  - 建议填写 `image_width` / `image_height`（用于运行时分辨率一致性检查）
  - 海康运行参数 `hik_camera`（曝光、增益、触发、设备索引等）
- `config/world_points.yaml`
  - 世界坐标点，按顺序写入（程序会在运行时询问 n，并使用前 n 个点）
- `config/out_matrix.yaml`
  - 输出外参（`world_rvec` / `world_tvec`）

## 编译

```bash
cd /home/robot/Hikcamera_calibration/RadarCameraCalibration_ExtrinsicParas
mkdir -p build
cd build
cmake ..
make -j
```

如果 MVS SDK 不在默认路径（如 `/opt/MVS`），可先设置：

```bash
export HIK_MVS_ROOT=/path/to/MVS
```

## 运行

先做配置检查：

```bash
cd /home/robot/Hikcamera_calibration/RadarCameraCalibration_ExtrinsicParas
./build/extrinsic_calibrator --check-only
```

启动标定：

```bash
cd /home/robot/Hikcamera_calibration/RadarCameraCalibration_ExtrinsicParas
./build/extrinsic_calibrator
```

启动后终端会提示输入 `n`（`4 <= n <= world_points.yaml` 中点数），随后按 `world_points.yaml` 的前 `n` 个点顺序采点并求解外参。

重投影误差可视化验收：

```bash
cd /home/robot/Hikcamera_calibration/RadarCameraCalibration_ExtrinsicParas
./build/reprojection_error_viewer
```

启动后终端会提示输入 `n`（`4 <= n <= world_points.yaml` 中点数），验收时同样只使用前 `n` 个点，确保与标定一致。

## 离线图片外参标定（推荐）

如果你想直接使用 `images` 目录中的图片离线标定（不连相机），运行：

```bash
cd /home/robot/Hikcamera_calibration/RadarCameraCalibration_ExtrinsicParas
python3 offline_extrinsic_calibrator.py
```

可选参数：

- `--image-dir images`（图片目录）
- `--single-image images/Image_01.jpg`（只标定单张图）
- `--camera-params config/camera_params.yaml`
- `--world-points config/world_points.yaml`
- `--output config/offline_extrinsics.yaml`（每张图的结果）
- `--write-best-to config/out_matrix.yaml`（将 RMS 最小的一组写成最终外参）

交互键位（离线程序）：

- 鼠标左键：选当前点
- `W/A/S/D`：像素级微调当前待确认点
- `n`：确认当前点
- `u`：撤销上一个已确认点
- `r`：重置当前图片全部点
- `x`：跳过当前图片
- `q` / `Esc`：退出

说明：离线程序会同时弹出一个 `*_roi` 放大窗口（带十字线）用于微调观察。

离线程序会按 `config/world_points.yaml` 的点位顺序进行采点，并在结束后输出：

- `config/offline_extrinsics.yaml`：每张图的 `rvec/tvec`、重投影误差与 `inlier_count`
- `config/out_matrix.yaml`：自动写入误差最小（RMS 最低）的外参

可选参数：

- `--camera-params config/camera_params.yaml`
- `--world-points config/world_points.yaml`
- `--output config/out_matrix.yaml`
- `--display-width 1536 --display-height 1125`
- `--timeout-ms 1000`
- `--extrinsic config/out_matrix.yaml`（重投影验收程序使用）

## 交互说明

1. 按 `Enter` 进入标定模式
2. 按顺序点击当前目标点
3. 使用 `W/A/S/D` 对当前点像素级微调
4. 按 `n` 确认该点
5. 所有点确认后自动求解并写入 `config/out_matrix.yaml`

快捷键：

- `q` / `Esc`：退出
- `r`：重置当前选点

## 重投影误差可视化说明

`reprojection_error_viewer` 会实时显示：

- 绿色点：由 `world_points + out_matrix` 计算得到的重投影点
- 红色点：你在画面上手动点击的实际点
- 青色连线：每个点的偏差方向与大小
- 文本统计：`mean / rms / max` 像素误差

操作键位：

- `Enter`：开始按顺序选点
- 鼠标左键 + `W/A/S/D` + `n`：精调并确认当前点
- `r` 或 `c`：清空当前验收结果，重新测
- `q` / `Esc`：退出

## n点顺序

必须严格按照 `config/world_points.yaml` 的前 `n` 个点顺序点击
