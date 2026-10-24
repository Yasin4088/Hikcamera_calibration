# 单目标定独立模块（MonocularCalibration）

这个目录是从 `DartAutoAim` **复制**出来的单目标定链路，原仓库文件未移动、未删除。  
模块包含：

- 单目标定可执行程序：`demarcate`
- 实时采集界面（海康相机取流 + BMP保存）
- 离线可视化标注界面（ROI框选、角点检查、覆盖率可视化）
- OpenCV 交互标定求解与结果导出

## 目录说明

- `CMakeLists.txt`：独立构建脚本
- `sub_module/Calibration/demarcate.cpp`：标定主程序入口
- `sub_module/Calibration/monocular_workflow.hpp`：单目标定采集/离线流程与UI交互
- `sub_module/calibration_ui.hpp`：界面字体、文本与鼠标输入兼容
- `sub_module/opencv_interactive_calibration/`：OpenCV 交互标定核心
- `include/` + `src/`：相机采集、队列、配置读取等依赖
- `config/dart.yaml`：默认配置文件

## 环境依赖

1. 海康 MVS SDK（默认路径 `/opt/MVS`）
2. OpenCV 组件：`core/imgproc/imgcodecs/calib3d/highgui`  
   - `freetype` 为可选：存在时显示完整中文；缺失时自动降级为 OpenCV 原生字体
   - `aruco` 为可选：单目标定流程（棋盘）不依赖它
3. 中文字体（用于界面文字）  
   - 推荐安装：`fonts-noto-cjk`  
   - 或设置环境变量：`DART_CALIBRATION_FONT=/绝对路径/xxx.ttc`

## 编译

在工作空间根目录执行：

```bash
cd /home/robot/Hikcamera_calibration
cmake -S MonocularCalibration -B MonocularCalibration/build -DMVS_ROOT=/opt/MVS
cmake --build MonocularCalibration/build --target demarcate -j
```

> 如果你的 MVS 不在 `/opt/MVS`，把 `-DMVS_ROOT=...` 改成实际路径。

## 标定操作流程

建议严格按“采集 -> 离线标定 -> 导出结果”执行。

### 1) 采集单相机 BMP

```bash
./MonocularCalibration/build/demarcate --collect
```

默认保存路径：

- `MonocularCalibration/sub_module/Calibration/Input`

采集界面快捷键：

- `A`：自动采样/暂停
- `空格`：立即保存一帧
- `Q` / `Esc`：退出

采集建议：

- 棋盘格保持清晰、曝光稳定
- 多采“位置、角度、远近、边缘区域”不同姿态
- 建议至少采集 20~30 张可用图像

### 2) 离线可视化标定（ROI框选）

```bash
./MonocularCalibration/build/demarcate --calibration
```

如果图片目录自定义过，可指定：

```bash
./MonocularCalibration/build/demarcate --calibration --input=/你的BMP目录
```

离线界面常用操作：

- 左键拖拽：框选棋盘 ROI
- 滚轮：缩放
- 右键拖动：平移
- `空格`：接受当前图并进入下一张
- `N`：跳过当前图
- `B/P`：上一张/下一张
- `R`：清除当前 ROI
- `D`：从标定集中移除当前图
- `S`：保存当前求解结果
- `Q` / `Esc`：退出并结束流程

### 3) 结果输出

默认输出到新目录（按时间戳命名）：

- `MonocularCalibration/sub_module/Calibration/Results/camera_YYYYMMDD_HHMMSS/`

主要文件：

- `intrinsics.yaml`：内参、畸变、RMS、每张图误差、标定状态
- `review.png`：离线标定界面截图
- `annotations.yaml`：标注过程数据（保存在输入图目录）

## 配置文件说明

默认使用：`MonocularCalibration/config/dart.yaml`

采集模式读取 `cameras` 单相机配置：

- `device_ip`：相机 IP
- `net_ip`：本机网口 IP
- `exposure_us` / `gain`
- `image_width` / `image_height`（必须与实际采集尺寸一致）

可通过参数切换配置文件：

```bash
./MonocularCalibration/build/demarcate --collect --config=/path/to/dart.yaml
```

## 常见问题

1. **提示找不到 MVS 库**
   - 检查 `-DMVS_ROOT` 是否正确
   - 确认 `${MVS_ROOT}/include/MvCameraControl.h` 和 `${MVS_ROOT}/lib/64/libMvCameraControl.so` 存在

2. **界面中文乱码或报字体错误**
   - 安装 `fonts-noto-cjk`
   - 或配置 `DART_CALIBRATION_FONT`

3. **离线阶段无可用图片**
   - 确认 `--input` 路径下有 `.bmp` 文件
   - 检查图片是否可被 OpenCV 正常读取

