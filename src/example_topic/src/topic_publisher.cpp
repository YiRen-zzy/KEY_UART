#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/send_data.hpp"
#include <rclcpp/logging.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class TopicPublisher01 : public rclcpp::Node
{
public:
    TopicPublisher01(std::string name) : Node(name)
    {
        // 创建发布者
        send_data_publisher_ = this->create_publisher<auto_aim_interfaces::msg::SendData>("send_data", 10);
        // 创建定时器，100ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TopicPublisher01::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        auto_aim_interfaces::msg::SendData message;
        message.header.stamp = this->now();

        // 每次回调重置所有变量
        float x = 0.0;
        float y = 0.0;
        float pitch = 0.0;
        float yaw = 0.0;

        // 检查键盘输入并调整 pitch, yaw, position
        char key = getch(); // 获取键盘输入

        if (key == 'w') {
            x += 1;  // 前进
        } else if (key == 's') {
            x -= 1;  // 后退
        } else if (key == 'a') {
            y += 1;  // 左移
        } else if (key == 'd') {
            y -= 1;  // 右移
        } else if (key == 'i') {
            pitch += 1;  // 向上俯仰
        } else if (key == 'k') {
            pitch -= 1;  // 向下俯仰
        } else if (key == 'j') {
            yaw += 1;  // 向左旋转
        } else if (key == 'l') {
            yaw -= 1;  // 向右旋转
        }

        // 更新消息内容
        message.position.x = x;
        message.position.y = y;
        message.pitch = pitch;
        message.yaw = yaw;

        // 实时输出当前的移动和俯仰角旋转数据
        RCLCPP_INFO(this->get_logger(), "Current Data -> Position: [x: %.2f, y: %.2f], Pitch: %.2f, Yaw: %.2f", x, y, pitch, yaw);

        // 发布消息
        send_data_publisher_->publish(message);
    }

    // 获取键盘输入的函数
    char getch()
{
    char buf = 0;
    struct termios old;
    if (tcgetattr(0, &old) < 0)
        perror("tcgetattr()");
    
    // 使用 memset 初始化结构体
    struct termios new_attr;
    memset(&new_attr, 0, sizeof(new_attr));
    
    new_attr = old;
    new_attr.c_lflag &= ~ICANON;
    new_attr.c_lflag &= ~ECHO;
    new_attr.c_cc[VMIN] = 1;
    new_attr.c_cc[VTIME] = 0;

    if (tcsetattr(0, TCSANOW, &new_attr) < 0)
        perror("tcsetattr ICANON");

    if (read(0, &buf, 1) < 0)
        perror("read()");

    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");

    return buf;
}


    // 声明定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者
    rclcpp::Publisher<auto_aim_interfaces::msg::SendData>::SharedPtr send_data_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 创建对应节点的共享指针对象
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    // 运行节点，并检测退出信号
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
