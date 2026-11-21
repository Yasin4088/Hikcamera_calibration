#pragma once

#include "protocal.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <thread>

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_struct.hpp>

namespace livox_lidar {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
constexpr size_t pc_msg_size = 4096;

class LidarPublisher : public rclcpp::Node {
private:
  std::string frame_id;
  boost::asio::io_context ctx;
  boost::asio::ip::address_v4 local_ip;
  boost::asio::ip::address_v4 dest_ip;
  uint16_t local_port;
  std::optional<boost::asio::ip::udp::socket> socket;
  std::atomic_bool need_reconnect = true;

  size_t batch_dot_num;
  rclcpp::Publisher<PointCloud2>::SharedPtr pc_pub;
  std::thread recv_thread;
  std::thread heartbeat_thread;
  PointCloud2::SharedPtr pc_msg;

  void recv_spin();
  void heartbeat_spin();

  void process_type2(const protocal::data_header& header, const protocal::type2_span& data)
  {
    for (size_t i = 0; i < protocal::dot_num; ++i) {
      pc2_write({
        data[i].x.value() / 1000.0F,
        data[i].y.value() / 1000.0F,
        data[i].z.value() / 1000.0F,
        static_cast<float>(data[i].reflectivity.value()),
        data[i].tag.value(),
        0x00,
        static_cast<double>(header.timestamp.value()),
      });
    }
  }

  void process_type4(const protocal::data_header& header, const protocal::type4_span& data)
  {
    for (size_t i = 0; i < protocal::dot_num_dual; ++i) {
      pc2_write({
        data[i].point1.x.value() / 1000.0F,
        data[i].point1.y.value() / 1000.0F,
        data[i].point1.z.value() / 1000.0F,
        static_cast<float>(data[i].point1.reflectivity.value()),
        data[i].point1.tag.value(),
        0x00,
        static_cast<double>(header.timestamp.value()),
      });
    }
  }

  void pc2_init()
  {
    pc_msg = std::make_shared<PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
    modifier.setPointCloud2Fields(
      7,
      "x", 1, PointField::FLOAT32,
      "y", 1, PointField::FLOAT32,
      "z", 1, PointField::FLOAT32,
      "intensity", 1, PointField::FLOAT32,
      "tag", 1, PointField::UINT8,
      "resv", 1, PointField::UINT8,
      "timestamp", 1, PointField::FLOAT64);
    pc_msg->header.frame_id.assign(frame_id);
    pc_msg->height = 1;
    pc_msg->width = batch_dot_num;
    pc_msg->row_step = pc_msg->width * pc_msg->point_step;
    pc_msg->is_bigendian = false;
    pc_msg->is_dense = true;
    pc_msg->data.resize(pc_msg->row_step);
  }

  void pc2_write(const radar_interface::LivoxPointXyzrtlt& pt)
  {
    auto data = reinterpret_cast<radar_interface::LivoxPointXyzrtlt*>(pc_msg->data.data());
    static size_t it = 0;
    data[it++] = pt;
    if (it == pc_msg->width) {
      pc_msg->header.stamp = now();
      pc_pub->publish(*pc_msg);
      it = 0;
    }
  }

public:
  explicit LidarPublisher(const rclcpp::NodeOptions& options);
  ~LidarPublisher() override;
};

}  // namespace livox_lidar
