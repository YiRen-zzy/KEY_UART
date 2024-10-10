#include "rm_serial_driver/rm_serial_driver.hpp"

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
  getParams();
  // Create Publisher
  serial_msg_pub_ = this->create_publisher<auto_aim_interfaces::msg::ReceiveData>(
    "/serial_msg", rclcpp::QoS(rclcpp::KeepLast(1)));

  try {
    // 初始化串口，设置设备名称和配置
    serial_driver_->init_port(device_name_, *device_config_);
    // 检查串口是否打开，如果没有则打开，并启动接收数据的线程
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    // 捕获异常，记录错误日志，并抛出异常
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // Create Subscription
  send_data_sub_ = this->create_subscription<auto_aim_interfaces::msg::SendData>(
    "send_data", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  // 检查接收线程是否正在运行，如果是，则等待其结束
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  // 检查串口是否打开，如果是，则关闭它
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }
  // 如果拥有的上下文对象存在，则等待其退出
  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);
      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);
        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        // bool crc_ok =
        //   crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        // 取消crc校验, 若要使用，将注释取消，并将if(1)改为if(crc_ok)
        if (1) {
          auto_aim_interfaces::msg::ReceiveData receive_data_msg;
          receive_data_msg.pitch = packet.pitch;
          receive_data_msg.yaw = packet.yaw;
          serial_msg_pub_->publish(receive_data_msg);
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::SendData::SharedPtr msg)
{
  try {
    SendPacket packet;
    packet.pitch = msg->pitch;
    packet.yaw = msg->yaw;
    packet.position_x = msg->position_x;
    packet.position_y = msg->position_y;
    packet.shot = msg->shot;

    // 取消crc校验, 若要使用，将注释取消
    //crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;  // 表示串口驱动中的流控类型
  using Parity = drivers::serial_driver::Parity;      // 表示串口驱动中的奇偶校验类型
  using StopBits = drivers::serial_driver::StopBits;  // 表示串口驱动中的停止位类型

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

/**
 * @brief 尝试重新打开串口
 */
void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    // 如果端口是打开的，尝试关闭它
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    // 重新打开端口
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    // 如果ros节点还在运行，则等待一秒后再次尝试打开端口
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
