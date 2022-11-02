#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        //callAddTwoIntsService(1, 4);

        // If you call the function directly: the program starts -> we start the node
        // -> we create the client -> we wait for service -> if we start the server on
        // another terminal it passes -> we send the request -> we'll wait for the
        // response at future.get (problem) -> This will block the thread here
        // -> which means the function will not exit -> so the constructor will
        // not exit -> so we'll still be in this line:
        // "auto node = std::make_shared<AddTwoIntsClientNode>();" -> and spin(node)
        // will not be executed. -> But we need spin(node) in order to get the result
        // from the future.
        // So we need to start this in a different thread so that we can continue the
        // execution, and the constructor can exit and we can call spin on the node.
        // So we create a thread object in private
        thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4));

        // If you'd like to send multiple requests you can use a thread pool
        //threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4)));
        //threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 2, 5)));

    }

    // This function needs to be called on a different thread
    void callAddTwoIntsService(int a, int b)
    {
        // Create client with type and name
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        // Wait for service to be up
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        // We fill the request with what we need and send it
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        // This will block the thread
        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    std::thread thread1_;
    // Example solution for multiple requests
    //std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
