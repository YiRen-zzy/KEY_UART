#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <auto_aim_interfaces/msg/detail/receive_data__struct.hpp>
#include <auto_aim_interfaces/msg/detail/send_data__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/receive_data.hpp"
#include "auto_aim_interfaces/msg/send_data.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendData(auto_aim_interfaces::msg::SendData::SharedPtr msg);

  void reopenPort();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Publisher<auto_aim_interfaces::msg::ReceiveData>::SharedPtr serial_msg_pub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::SendData>::SharedPtr send_data_sub_;
  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
