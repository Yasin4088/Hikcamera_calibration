# 雷达相机外参标定（纯 C++）

该目录提供一个独立的 C++ 外参标定工具，不依赖 ROS2，流程为：

1. 通过海康 MVS SDK 连接相机并读取配置参数（曝光、增益、帧率等）
2. 手动五点选取图像坐标
3. 调用 `solvePnP` 计算外参
4. 写入 `config/out_matrix.yaml`

## 配置文件

- `config/camera_params.yaml`
  - 相机内参 `camera_matrix`、畸变 `dist_coeffs`
  - 海康运行参数 `hik_camera`（曝光、增益、触发、设备索引等）
- `config/world_points.yaml`
  - 世界坐标点，按顺序写入（当前为 5 点）
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

可选参数：

- `--camera-params config/camera_params.yaml`
- `--world-points config/world_points.yaml`
- `--output config/out_matrix.yaml`
- `--display-width 1536 --display-height 1125`
- `--timeout-ms 1000`

## 交互说明

1. 按 `Enter` 进入标定模式
2. 按顺序点击当前目标点
3. 使用 `W/A/S/D` 对当前点像素级微调
4. 按 `n` 确认该点
5. 所有点确认后自动求解并写入 `config/out_matrix.yaml`

快捷键：

- `q` / `Esc`：退出
- `r`：重置当前选点

## 五点顺序（T-DT标点顺序）

必须严格按照 `config/world_points.yaml` 顺序点击：

1. `self_FORTRESS`
2. `self_Tower`
3. `enemy_Base`
4. `enemy_Tower`
5. `enemy_High`
