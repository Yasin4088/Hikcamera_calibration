# Mid-70 Driver (ROS2)

该工程从 [`Mid-70_perception_module`](https://github.com/Yasin4088/Mid-70_perception_module.git) 中独立出 Mid-70 驱动能力，只保留驱动相关包，便于单独驱动与录包。

## 目录结构

- `livox_lidar`: Mid-70 UDP 收包并发布 `sensor_msgs/PointCloud2`（topic: `/pc_raw`）
- `radar_interface`: 驱动依赖的数据结构与接口定义
- `config/mid70_default.yaml`: 与感知模块一致的驱动参数
- `launch/mid70_driver_launch.py`: 仅启动 Mid-70 驱动
- `record.sh`: ROS2 录包脚本

## 编译

```bash
cd /home/robot/Hikcamera_calibration/Mid-70_driver
colcon build --symlink-install
```

## 启动驱动

```bash
cd /home/robot/Hikcamera_calibration/Mid-70_driver
source install/setup.bash
ros2 launch ./launch/mid70_driver_launch.py
```

## 录包

先启动驱动，再开新终端执行:

```bash
cd /home/robot/Hikcamera_calibration/Mid-70_driver
source install/setup.bash
./record.sh mid70_$(date +%Y%m%d_%H%M%S)
```

默认录制 `/pc_raw /tf /tf_static`，也可以追加自定义 topic，例如:

```bash
./record.sh mid70_test /pc_raw /livox_lidar/diagnostics
```
