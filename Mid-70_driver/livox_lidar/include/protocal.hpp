#pragma once

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include <boost/endian/arithmetic.hpp>
#include <rclcpp/rclcpp.hpp>

namespace livox_lidar {
namespace protocal {

using namespace boost::endian;

constexpr size_t dot_num = 96;
constexpr size_t dot_num_dual = 48;
constexpr uint16_t dest_port = 65000;

constexpr uint8_t heartbeat_raw[] {
  0xaa, 0x01, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x04, 0xd7, 0x00, 0x03, 0x38, 0xba, 0x8d, 0x0c
};
constexpr uint8_t start_data[] {
  0x00, 0x04, 0x01, 0xab, 0xb2, 0x3b, 0x4b
};

constexpr uint16_t livox_crc16_init = 0x4c49;
constexpr uint32_t livox_crc32_init = 0x564f580a;

#pragma pack(push, 1)
struct data_header {
  little_uint8_t version;
  little_uint8_t slot_id;
  little_uint8_t lidar_id;
  little_uint8_t reserved;
  little_uint32_t status_code;
  little_uint8_t timestamp_type;
  little_uint8_t data_type;
  little_uint64_t timestamp;
};

struct pc_type2 {
  little_int32_t x;
  little_int32_t y;
  little_int32_t z;
  little_uint8_t reflectivity;
  little_uint8_t tag;
};

using type2_span = std::array<struct pc_type2, dot_num>;

struct pc_type4 {
  pc_type2 point1;
  pc_type2 point2;
};

using type4_span = std::array<struct pc_type4, dot_num_dual>;

struct frame_header {
  little_uint8_t sof = 0xaa;
  little_uint8_t version = 1;
  little_uint16_t length;
  little_uint8_t cmd_type = 0;
  little_uint16_t seq_num;
  little_uint16_t crc_16;
};

struct handshake_data {
  little_uint8_t cmd_set = 0x00;
  little_uint8_t cmd_id = 0x01;
  little_uint8_t user_ip[4];
  little_uint16_t data_port;
  little_uint16_t cmd_port;
  little_uint16_t imu_port;
};
#pragma pack(pop)

inline uint16_t crc16_mcrf4xx(const uint8_t* data, size_t len, uint16_t init)
{
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001U) {
        crc = static_cast<uint16_t>((crc >> 1U) ^ 0x8408U);
      } else {
        crc = static_cast<uint16_t>(crc >> 1U);
      }
    }
  }
  return crc;
}

inline uint32_t crc32_livox(const uint8_t* data, size_t len, uint32_t init)
{
  uint32_t crc = init ^ 0xffffffffU;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 1U) {
        crc = (crc >> 1U) ^ 0xedb88320U;
      } else {
        crc >>= 1U;
      }
    }
  }
  return crc ^ 0xffffffffU;
}

inline bool check_header(const data_header& header)
{
  if (header.version.value() == 0xaa) {
    return false;
  }
  if (header.version.value() != 5) {
    RCLCPP_ERROR(rclcpp::get_logger("header"), "version is not 5, got %u", header.version.value());
    return false;
  }
  if (header.data_type.value() != 2 && header.data_type.value() != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("header"), "data_type is not 2 or 4, got %u", header.data_type.value());
    return false;
  }
  if (header.lidar_id.value() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("header"), "lidar_id is not 1, got %u", header.lidar_id.value());
    return false;
  }
  return true;
}

inline size_t write_frame_buffer(uint8_t* buf, const uint8_t* data, size_t size, size_t max_size)
{
  static uint16_t seq_num = 0;
  size_t length = sizeof(frame_header) + size + sizeof(uint32_t);
  if (length > max_size) {
    throw std::out_of_range("Out of buffer size.");
  }

  frame_header header;
  header.length = static_cast<uint16_t>(length);
  header.seq_num = seq_num++;

  const uint8_t* header_data = reinterpret_cast<const uint8_t*>(&header);
  header.crc_16 = crc16_mcrf4xx(header_data, sizeof(header) - sizeof(header.crc_16), livox_crc16_init);
  std::copy(header_data, header_data + sizeof(frame_header), buf);

  std::copy(data, data + size, buf + sizeof(frame_header));

  little_uint32_t crc32_val = crc32_livox(buf, sizeof(header) + size, livox_crc32_init);
  const uint8_t* crc32_data = reinterpret_cast<const uint8_t*>(&crc32_val);
  std::copy(crc32_data, crc32_data + sizeof(crc32_val), buf + sizeof(frame_header) + size);

  return length;
}

}  // namespace protocal
}  // namespace livox_lidar
