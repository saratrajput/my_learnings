#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"
 
using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10,
                        std::bind(&NumberCounterNode::callbackNumberCounter, this, std::placeholders::_1));
        // Reset counter service
        server_ = this->create_service<example_interfaces::srv::SetBool>(
                "reset_counter",
                std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Number Counter node has started.");
    }
 
private:
    void callbackNumberCounter(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        // Subscriber part
        counter_ += msg->data;
        RCLCPP_INFO(this->get_logger(), "%d", counter_);
        // Publisher Part
        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = counter_;
        publisher_->publish(new_msg);
    }

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                              const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            response->message = "Counter has been reset";
        }
        else
        {
            response->success = false;
            response->message = "Counter has not been reset";
        }
    }
    int counter_;
    // Declare subscriber
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    // Declare Publisher
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    // Declare Service
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
