#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from my_robot_msgs.srv import ResetCounter

counter = 0


def callback_number(msg):
    global counter
    counter += msg.data
    rospy.loginfo("Counter: " + str(counter))


def callback_reset_counter(req):
    if req.reset_value >= 0:
        global counter
        counter = req.reset_value
        rospy.loginfo("Counter: " + str(counter))
        return True
    return False


if __name__ == '__main__':
    rospy.init_node('number_counter')

    sub = rospy.Subscriber("number", Int64, callback_number)

    reset_service = rospy.Service(
        "reset_counter", ResetCounter, callback_reset_counter)

    rospy.loginfo("Number counter has been started.")

    rospy.spin()
