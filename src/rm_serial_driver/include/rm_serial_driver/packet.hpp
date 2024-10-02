#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <sys/types.h>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{

// 定义接收数据的数据包结构
struct ReceivePacket
{
  uint8_t header = 0x5A;  // 帧头
  float pitch;
  float yaw;
  // uint16_t checksum = 0; // CRC校验位，未开启
} __attribute__((packed));  // 保证结构体被连续编译，不插入对齐填充物

// 定义发送数据的数据包结构
struct SendPacket
{
  uint8_t header = 0xA5;  // 帧头
  float pitch;
  float yaw;
  float target_x;
  float target_y;
  float target_z;
  // uint16_t checksum = 0; // CRC校验位，未开启
} __attribute__((packed));  // 保证结构体被连续编译，不插入对齐填充物

/**
 * @brief 将电控发的多个连续字节转化为视觉接受的数据
 */
inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

/**
 * @brief 将视觉发送的数据转化多个连续字节，发给电控
 */
inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
