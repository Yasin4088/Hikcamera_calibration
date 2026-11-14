# 雷达相机外参标定（纯 C++）

该目录提供一个独立的 C++ 外参标定工具，不依赖 ROS2，流程为：

1. 通过海康 MVS SDK 连接相机并读取配置参数（曝光、增益、帧率等）
2. 手动 n 点选取图像坐标（运行时输入 n）
3. 调用 `solvePnP(EPNP + ITERATIVE)` 计算外参
4. 写入 `config/out_matrix.yaml`

## 配置文件

- `config/camera_params.yaml`
  - 相机内参 `camera_matrix`、畸变 `dist_coeffs`
  - 海康运行参数 `hik_camera`（曝光、增益、触发、设备索引等）
- `config/world_points.yaml`
  - 世界坐标点，按顺序写入（程序会在运行时询问 n，并使用前 n 个点）
- `config/out_matrix.yaml`
  - 输出外参（`world_rvec` / `world_tvec`）及重投影误差（`mean_error` / `rms_error` / `max_error`，单位 px）

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

重投影验收程序也会在启动后询问 `n`，并按 `world_points.yaml` 的前 `n` 个点进行对比，确保与标定点集一致。

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
