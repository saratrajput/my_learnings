#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisherNode : public rclcpp::Node
{
public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher")
    {
        hw_status_publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                    std::bind(&HardwareStatusPublisherNode::publishHwStatus, this));
        RCLCPP_INFO(this->get_logger(), "Hardware status publisher has been started...");

    }

private:
    void publishHwStatus()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 50;
        msg.are_motors_ready = true;
        msg.debug_message = "Nothing special";
        hw_status_publisher_->publish(msg);
    }
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr hw_status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
