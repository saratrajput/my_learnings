#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <my_robot_msgs/ResetCounter.h>

int counter = 0;

void callback_number(const std_msgs::Int64 &msg)
{
    counter += msg.data;
    ROS_INFO("Counter: %d", counter);
}

bool callback_reset_counter(my_robot_msgs::ResetCounter::Request &req,
                            my_robot_msgs::ResetCounter::Response &res)
{
    if (req.reset_value >= 0)
    {
        counter = req.reset_value;
        res.success = true;
        ROS_INFO("Counter: %d", counter);
    }
    else
    {
        res.success = false;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("number", 1000, callback_number);

    ros::ServiceServer reset_service = nh.advertiseService("reset_counter",
                                                           callback_reset_counter);

    ROS_INFO("Number counter has been started.");

    ros::spin();
    return 0;
}
