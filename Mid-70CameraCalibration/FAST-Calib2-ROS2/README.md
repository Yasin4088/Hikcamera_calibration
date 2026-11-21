# FAST-Calib2(ROS2)

## 目录说明

- `config/qr_params.yaml`: 标定参数（相机内参、标定板几何、ROI、输入输出路径）
- `launch/calib.launch.py`: 单场景标定启动文件
- `launch/multi_calib.launch.py`: 多场景联合标定启动文件
- `src/main.cpp`: 单场景标定主程序（输出 `T_cam_lidar`）
- `src/multi_scene.cpp`: 多场景联合求解程序
- `test/lidar_center_test.cpp`: LiDAR 圆心提取测试程序
- `scripts/distance_filter_tool.py`: ROI 参数辅助工具

## 依赖环境

- ROS2 Humble（或兼容版本）
- PCL >= 1.8
- OpenCV >= 4.0
- Eigen3

## 编译

```bash
cd /home/robot/Hikcamera_calibration/Mid-70CameraCalibration/FAST-Calib2-ROS2
source /opt/ros/humble/setup.bash

# 如 OpenCV 未被自动找到，可按下行方式指定 OpenCV_DIR
OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 colcon build --symlink-install --base-paths .

source install/setup.bash
```

## 参数配置

编辑 `config/qr_params.yaml`，至少确认以下内容：

1. **相机内参**  
   `camera_width/camera_height/fx/fy/cx/cy/k1/k2/p1/p2`

2. **标定板几何参数**（必须与实物一致）  
   `marker_size`、`delta_width_qr_center`、`delta_height_qr_center`、`delta_width_circles`、`delta_height_circles`、`circle_radius`、`annulus_half_width`

3. **LiDAR 安装轴定义**  
   `lidar_forward_axis`、`lidar_up_axis`  
   例如 Mid-70 常见配置为：
   ```yaml
   lidar_forward_axis: "+x"
   lidar_up_axis: "+z"
   ```

4. **输入路径与话题**
   - `bag_path`: rosbag2 目录路径
   - `image_path`: 与该场景对应的图像路径
   - `lidar_topic`: 点云 topic（如 `/pc_raw`）

5. **输出路径**
   - `output_path`: 标定结果输出目录

## 单场景标定

```bash
cd /home/robot/Hikcamera_calibration/Mid-70CameraCalibration/FAST-Calib2-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_calib calib.launch.py rviz:=false
```

## 多场景联合标定

1. 先在不同位置采集并运行多次单场景标定，累计写入 `output/circle_center_record.txt`。  
2. 再执行联合求解：

```bash
cd /home/robot/Hikcamera_calibration/Mid-70CameraCalibration/FAST-Calib2-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_calib multi_calib.launch.py rviz:=false
```

多场景结果输出为 `output/multi_calib_result.txt`（含 FAST-LIVO2 使用格式的 `Rcl`、`Pcl`）。

## LiDAR 圆心提取测试（可选）

用于先验证 LiDAR 侧圆心提取质量，再做完整相机-LiDAR 联合标定：

```bash
cd /home/robot/Hikcamera_calibration/Mid-70CameraCalibration/FAST-Calib2-ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_calib lidar_center_test <bag_path> <lidar_topic> [auto|solid|mech] [forward_axis up_axis]
```

示例：

```bash
ros2 run fast_calib lidar_center_test /path/to/mid70_bag /pc_raw solid +x +z
```

> 说明：`config/qr_params.yaml` 推荐使用 `$(find fast_calib)` 形式填写 `bag_path`、`image_path`、`output_path`，
> 这样即使项目目录改名，launch 也能自动解析到当前包路径。

## 输出文件说明

单场景标定完成后，`output_path` 下常见文件：

- `single_calib_result.txt`: 单场景外参结果（`T_cam_lidar`）
- `circle_center_record.txt`: 每次场景的 LiDAR/QR 圆心记录（多场景输入）
- `colored_cloud.pcd`: 使用外参投影上色后的点云
- `qr_detect.png`: 相机侧检测可视化结果

多场景流程额外输出：

- `multi_calib_result.txt`: 联合标定结果（`Rcl`、`Pcl`）
