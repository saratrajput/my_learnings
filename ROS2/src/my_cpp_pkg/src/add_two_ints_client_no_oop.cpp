#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
 
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Create node
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop"); // MODIFY NAME

    //Create client directly from the create client method with service type and service
    //name. Service name should match exactly that from the server
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    
    // Wait for the service to be up
    while(!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for the server to be up...");
    }

    // Create a request which is a shared pointer
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 3;
    request->b = 8;

    // Send the request asynchronously
    // In fact we'll receive a shared future of a shared ptr
    auto future = client->async_send_request(request);
    // Spin the node until future is complete
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "%d + %d = %d", request->a, request->b, future.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Error while calling service");
    }
    rclcpp::shutdown();
    return 0;
}
