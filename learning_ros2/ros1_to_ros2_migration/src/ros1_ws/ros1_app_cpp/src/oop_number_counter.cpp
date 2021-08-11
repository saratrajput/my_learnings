#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <my_robot_msgs/ResetCounter.h>

class NumberCounter
{

public:
    NumberCounter(ros::NodeHandle *nh)
    {
        counter_ = 0;

        number_subscriber_ = nh->subscribe("number", 1000,
                                           &NumberCounter::callback_number, this);

        reset_service_ = nh->advertiseService("reset_counter",
                                              &NumberCounter::callback_reset_counter, this);

        ROS_INFO("Number counter has been started");
    }

    void callback_number(const std_msgs::Int64 &msg)
    {
        counter_ += msg.data;
        ROS_INFO("Counter: %d", counter_);
    }

    bool callback_reset_counter(my_robot_msgs::ResetCounter::Request &req,
                                my_robot_msgs::ResetCounter::Response &res)
    {
        if (req.reset_value >= 0)
        {
            counter_ = req.reset_value;
            res.success = true;
            ROS_INFO("Counter: %d", counter_);
        }
        else
        {
            res.success = false;
        }

        return true;
    }

private:
    int counter_;
    ros::Subscriber number_subscriber_;
    ros::ServiceServer reset_service_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    NumberCounter nc = NumberCounter(&nh);
    ros::spin();
    return 0;
}
