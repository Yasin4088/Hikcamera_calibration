# 单目标定模块（MonocularCalibration）

本目录提供海康单相机内参标定流程，核心可执行程序是 `demarcate`，支持：

- 在线采集 BMP（相机实时预览 + 自动/手动存图）
- 离线 ROI 框选（逐图检查角点与覆盖率）
- OpenCV 求解与结果导出（`intrinsics.yaml` / `review.png`）

建议按“构建 -> 采集 -> 离线标定 -> 导出结果”执行。

## 目录与脚本

- `build_demarcate.sh`：构建 `demarcate`（支持 `--build-type` / `--clean` / `--mvs-root`）
- `run_collect.sh`：采集入口（自动构建缺失的 `demarcate`）
- `run_calibration.sh`：离线标定入口（自动构建缺失的 `demarcate`）
- `sub_module/Calibration/demarcate.cpp`：命令行参数与流程入口
- `sub_module/Calibration/monocular_workflow.hpp`：采集、离线交互、求解逻辑
- `config/dart.yaml`：默认采集配置（只读取 `cameras`）

## 环境依赖

1. 海康 MVS SDK（默认 `/opt/MVS`）
2. OpenCV：`core/imgproc/imgcodecs/calib3d/highgui`
   - `freetype` 可选（有则显示更完整中文）
   - `aruco` 可选（本流程不依赖 ArUco）
3. 中文字体（推荐 `fonts-noto-cjk`）
   - 或设置 `DART_CALIBRATION_FONT=/绝对路径/xxx.ttc`

## 快速开始（推荐）

在仓库根目录执行：

```bash
cd /home/robot/Hikcamera_calibration/MonocularCalibration_OpenCV
./build_demarcate.sh
```

### 1) 采集 BMP

```bash
./run_collect.sh
```

默认输入目录（自动按时间戳新建）：

- `sub_module/Calibration/Input/camera_YYYYMMDD_HHMMSS/`

采集时目录必须是“新目录或空目录”，程序不会覆盖已有图片。

常见示例：

```bash
./run_collect.sh ./sub_module/Calibration/Input/camera_test
./run_collect.sh ./sub_module/Calibration/Input/camera_test --interval=0.8 --auto=true
CONFIG=/path/to/dart.yaml ./run_collect.sh
```

### 2) 离线框选与标定

```bash
./run_calibration.sh
```

默认读取输入目录：

- `sub_module/Calibration/Input/camera`

默认输出目录（自动按时间戳新建）：

- `sub_module/Calibration/Results/camera_YYYYMMDD_HHMMSS/`

常见示例：

```bash
./run_calibration.sh ./sub_module/Calibration/Input/camera_test
./run_calibration.sh ./sub_module/Calibration/Input/camera_test ./sub_module/Calibration/Results/my_result --model=full --min-views=12 --max-views=40
```

## 直接调用 demarcate（高级）

不通过脚本时可直接运行：

```bash
./build/demarcate --collect --config=./config/dart.yaml --input=/path/to/new_or_empty_dir
./build/demarcate --calibration --input=/path/to/bmps --output=/path/to/new_result_dir
```

常用参数：

- `--interval=<秒>`：采集自动保存间隔（默认 `1.0`）
- `--auto=<true|false>`：采集启动时是否自动保存
- `--model=<auto|full|k1|narrow>`：标定模型
- `--cols=<N> --rows=<N> --square-mm=<毫米>`：棋盘规格（默认 `11x8`，`30mm`）
- `--min-views=<N> --max-views=<N>`：质量评估最小/最大视图数量

说明：

- `--calibration` 若未给 `--input`，会尝试弹窗选择目录；取消会退出
- `--output` 目录必须不存在，避免覆盖历史结果

## 界面快捷键

采集界面：

- `A`：自动采样/暂停
- `空格`：立即保存一帧
- `Q` / `Esc`：退出

离线界面：

- 左键拖拽：框选棋盘 ROI
- 滚轮 / `+` / `-` / `0`：缩放与复位
- 右键拖动：平移
- `空格` / `N`：接受 / 跳过并下一张
- `B` / `P`：上一张 / 下一张
- `R` / `D`：清除 ROI / 从标定集中移除
- `S`：保存当前求解结果
- `Q` / `Esc`：退出

## 输出文件

标定输出目录（`Results/camera_时间戳`）主要包含：

- `intrinsics.yaml`：内参、畸变、RMS、逐图误差、质量检查状态
- `review.png`：离线界面截图

输入目录下会维护：

- `annotations.yaml`：ROI、角点与图片状态缓存（用于恢复标注）

## 配置说明（`config/dart.yaml`）

采集只读取 `cameras`：

- `device_ip` / `net_ip`（USB 相机可留空）
- `exposure_us` / `gain`
- `image_width` / `image_height`（需与实际采集一致）

`input.mode` 不影响本模块采集/离线流程，离线以 `--input` 指定的 BMP 目录为准。
