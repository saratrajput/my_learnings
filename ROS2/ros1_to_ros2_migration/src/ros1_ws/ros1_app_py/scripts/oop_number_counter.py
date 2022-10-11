#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from my_robot_msgs.srv import ResetCounter


class NumberCounter:

    def __init__(self):
        self.counter_ = 0
        self.number_subscriber_ = rospy.Subscriber(
            "number", Int64, self.callback_number)
        self.reset_service_ = rospy.Service(
            "reset_counter", ResetCounter, self.callback_reset_counter)
        rospy.loginfo("Number counter has been started.")

    def callback_number(self, msg):
        self.counter_ += msg.data
        rospy.loginfo("Counter: " + str(self.counter_))

    def callback_reset_counter(self, req):
        if req.reset_value >= 0:
            self.counter_ = req.reset_value
            rospy.loginfo("Counter: " + str(self.counter_))
            return True
        return False


if __name__ == '__main__':
    rospy.init_node('number_counter')
    NumberCounter()
    rospy.spin()
