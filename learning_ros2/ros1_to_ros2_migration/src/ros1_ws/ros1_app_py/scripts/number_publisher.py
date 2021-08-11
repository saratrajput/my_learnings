#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64

if __name__ == '__main__':
    rospy.init_node("number_publisher")

    pub = rospy.Publisher("number", Int64, queue_size=10)

    # Get the frequency from a parameter
    publish_frequency = rospy.get_param("/number_publish_frequency")
    rate = rospy.Rate(publish_frequency)

    # Get the number to publish from a parameter
    number = rospy.get_param("/number_to_publish")

    rospy.loginfo("Number publisher has been started.")

    while not rospy.is_shutdown():
        msg = Int64()
        msg.data = number
        pub.publish(msg)
        rate.sleep()
