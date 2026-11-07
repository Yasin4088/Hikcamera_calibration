# 雷达相机外参标定（独立版）

该目录包含从 `T-DT_Radar` 中拆分出的**不依赖 ROS2** 的相机外参标定模块。

## 1）拆分内容

- 核心方法：手动选择图像点 + `solvePnP` 解算外参。
- 世界坐标点与 `src/tdt_vision/calibrate` 使用的 5 点一致。
- 输出格式与 OpenCV YAML 兼容：
  - `world_rvec`
  - `world_tvec`

## 2）推荐环境

- 推荐 Python 3.9 及以上。
- 推荐使用 `venv` 虚拟环境运行，避免污染系统环境。

## 3）配置文件说明

- `config/camera_params.yaml`
  - 相机内参矩阵与畸变系数。
- `config/world_points.yaml`
  - 标定使用的世界坐标点（带顺序）。
- `config/out_matrix.yaml`
  - 外参输出文件（每次标定成功后会覆盖写入）。

如果你使用了不同相机，请先替换 `config/camera_params.yaml`。

## 4）在 venv 中运行（推荐）

### A. 进入目录

```bash
cd /home/robot/Hikcamera_calibration/RadarCameraCalibration_ExtrinsicParas
```

### B. 创建虚拟环境

```bash
python3 -m venv .venv
```

### C. 激活虚拟环境

```bash
source .venv/bin/activate
```

激活后命令行前通常会出现 `(.venv)` 前缀。

### D. 安装依赖

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### E. 先做配置检查

```bash
python calibrate_extrinsic.py --check-only
```

### F. 启动标定

1. 使用实时相机（通常是 0 号设备）：

```bash
python calibrate_extrinsic.py --input 0
```

2. 使用视频文件：

```bash
python calibrate_extrinsic.py --input /path/to/video.mp4
```

3. 使用单张图片：

```bash
python calibrate_extrinsic.py --input /path/to/image.png
```

### G. 退出虚拟环境

```bash
deactivate
```

## 5）交互操作说明

1. 按 `Enter` 进入标定模式。
2. 按顺序点击当前目标点。
3. 使用 `W/A/S/D` 对当前点进行像素级微调。
4. 按 `n` 确认并保存当前点。
5. 所有点确认后自动执行外参解算并写入 `config/out_matrix.yaml`。

快捷键：

- `q` 或 `Esc`：退出程序
- `r`：重置当前已选点并重新开始

## 6）点位顺序（必须一致）

必须严格按照 `config/world_points.yaml` 的顺序点击：

1. `self_FORTRESS`
2. `self_Tower`
3. `enemy_Base`
4. `enemy_Tower`
5. `enemy_High`

如果顺序错乱，输出外参会明显异常。
