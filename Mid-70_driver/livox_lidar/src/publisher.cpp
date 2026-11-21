#include "publisher.hpp"

using namespace livox_lidar;

void LidarPublisher::recv_spin()
{
  std::array<unsigned char, pc_msg_size> recv_buf;
  boost::system::error_code error;
  std::size_t recv_length = 0;
  while (rclcpp::ok() && socket->is_open()) {
    socket->async_receive(
      boost::asio::buffer(recv_buf),
      [&](const boost::system::error_code& error_, std::size_t length_) {
        error = error_;
        recv_length = length_;
      });
    ctx.restart();
    ctx.run_for(std::chrono::milliseconds(get_parameter("timeout_ms").as_int()));
    if (!ctx.stopped()) {
      socket->cancel();
      ctx.run();
    }
    if (error && error != boost::asio::error::message_size) {
      if (error != boost::asio::error::operation_aborted) {
        RCLCPP_ERROR(get_logger(), "Receiver error: %s", error.message().c_str());
      }
      need_reconnect = true;
      continue;
    }

    if (recv_length < sizeof(protocal::data_header)) {
      continue;
    }
    auto header = reinterpret_cast<protocal::data_header*>(recv_buf.data());
    if (!protocal::check_header(*header)) {
      continue;
    }
    if (header->data_type.value() == 2) {
      auto data = reinterpret_cast<protocal::type2_span*>(recv_buf.data() + sizeof(protocal::data_header));
      process_type2(*header, *data);
    } else if (header->data_type.value() == 4) {
      auto data = reinterpret_cast<protocal::type4_span*>(recv_buf.data() + sizeof(protocal::data_header));
      process_type4(*header, *data);
    }
  }
}

void LidarPublisher::heartbeat_spin()
{
  std::array<uint8_t, 256> buf;
  using boost::asio::buffer;

  while (rclcpp::ok() && socket->is_open()) {
    if (need_reconnect) {
      protocal::handshake_data hs;
      std::copy(local_ip.to_bytes().begin(), local_ip.to_bytes().end(), hs.user_ip);
      hs.data_port = local_port;
      hs.cmd_port = local_port;
      hs.imu_port = local_port;
      socket->send_to(
        buffer(buf, protocal::write_frame_buffer(
                      buf.data(),
                      reinterpret_cast<const uint8_t*>(&hs),
                      sizeof(hs),
                      buf.size())),
        boost::asio::ip::udp::endpoint(dest_ip, protocal::dest_port));

      socket->send_to(
        buffer(protocal::start_data, sizeof(protocal::start_data)),
        boost::asio::ip::udp::endpoint(dest_ip, protocal::dest_port));
      need_reconnect = false;
    }
    socket->send_to(
      buffer(protocal::heartbeat_raw, sizeof(protocal::heartbeat_raw)),
      boost::asio::ip::udp::endpoint(dest_ip, protocal::dest_port));
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

LidarPublisher::LidarPublisher(const rclcpp::NodeOptions& options)
: rclcpp::Node("livox_lidar", options)
{
  declare_parameter("batch_dot_num", 9600);
  declare_parameter("local_ip", "192.168.1.50");
  declare_parameter("dest_ip", "192.168.1.166");
  declare_parameter("udp_port", 50010);
  declare_parameter("timeout_ms", 1000);

  frame_id = std::string(get_name()) + "_frame";
  batch_dot_num = static_cast<size_t>(get_parameter("batch_dot_num").as_int());
  local_ip = boost::asio::ip::make_address_v4(get_parameter("local_ip").as_string());
  dest_ip = boost::asio::ip::make_address_v4(get_parameter("dest_ip").as_string());
  local_port = static_cast<uint16_t>(get_parameter("udp_port").as_int());

  RCLCPP_INFO(get_logger(), "frame_id: %s", frame_id.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Params: batch_dot_num=%lu, local_ip=%s, dest_ip=%s, udp_port=%d, timeout_ms=%ld",
    batch_dot_num,
    local_ip.to_string().c_str(),
    dest_ip.to_string().c_str(),
    local_port,
    static_cast<long>(get_parameter("timeout_ms").as_int()));

  pc_pub = create_publisher<PointCloud2>("pc_raw", rclcpp::QoS(rclcpp::KeepLast(10)));
  pc2_init();

  try {
    socket.emplace(ctx, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_port));
  } catch (const boost::system::system_error& e) {
    RCLCPP_FATAL(
      get_logger(),
      "Bind UDP port %d failed: %s. Check if another process is occupying it.",
      local_port, e.what());
    std::exit(1);
  }

  heartbeat_thread = std::thread(&LidarPublisher::heartbeat_spin, this);
  recv_thread = std::thread(&LidarPublisher::recv_spin, this);
}

LidarPublisher::~LidarPublisher()
{
  if (socket.has_value() && socket->is_open()) {
    boost::system::error_code ec;
    socket->close(ec);
  }
  if (heartbeat_thread.joinable()) {
    heartbeat_thread.join();
  }
  if (recv_thread.joinable()) {
    recv_thread.join();
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(livox_lidar::LidarPublisher)
