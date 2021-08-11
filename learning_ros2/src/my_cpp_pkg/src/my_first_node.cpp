#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
    public:
        MyNode(): Node("cpp_test"), counter_(0)
        {
            RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");

            timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                             std::bind(&MyNode::timerCallback, this));
        }

    private:
        
        void timerCallback()
        {
            counter_++;
            RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
        }
        
        rclcpp::TimerBase::SharedPtr timer_;
        int counter_;

};


int main(int argc, char **argv)
{
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);

    // Create a node of type "rclcpp::Node"
    // std::make_shared to create a shared pointer
    // If the shared pointer goes out of scope it automatically destroys the node
    //auto node = std::make_shared<rclcpp::Node>("cpp_test");
    auto node = std::make_shared<MyNode>();

    //RCLCPP_INFO(node->get_logger(), "Hello Cpp Node");

    // To keep the node alive
    rclcpp::spin(node);

    // Shutdown all ROS2 communication
    rclcpp::shutdown();
    return 0;
}
