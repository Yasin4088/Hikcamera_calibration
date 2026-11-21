#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace radar_interface {

#pragma pack(push, 1)
typedef struct {
  float x;
  float y;
  float z;
  float reflectivity;
  uint8_t tag;
  uint8_t resv;
  double timestamp;
} LivoxPointXyzrtlt;
#pragma pack(pop)

inline bool check_lidar_msg(const sensor_msgs::msg::PointCloud2& msg)
{
  if (msg.point_step != sizeof(radar_interface::LivoxPointXyzrtlt)) {
    RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "point_step error: %d", msg.point_step);
    return false;
  }
  if (msg.row_step != msg.point_step * msg.width) {
    RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "row_step error: %d", msg.row_step);
    return false;
  }
  if (msg.data.size() != msg.row_step * msg.height) {
    RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "data size error: %lu", msg.data.size());
    return false;
  }
  if (msg.fields.size() != 7) {
    RCLCPP_ERROR(rclcpp::get_logger("check_lidar_msg"), "fields size error: %lu", msg.fields.size());
    return false;
  }
  return true;
}

}  // namespace radar_interface
