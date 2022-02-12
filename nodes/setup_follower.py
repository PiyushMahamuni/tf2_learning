#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from math import atan2

# CONSTANTS
NODE_NAME = "follower"


def chase(follower, carrot):
    tfBuffer = tf2_ros.Buffer()
    tfl = tf2_ros.TransformListener(tfBuffer)
    vel_pub = rospy.Publisher(f"{follower}/cmd_vel", Twist, queue_size=1)
    vel_msg = Twist()
    vel_msg.linear.z = vel_msg.linear.y = 0.0
    vel_msg.angular.x = vel_msg.angular.y = 0.0
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            t = tfBuffer.lookup_transform(follower, carrot, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            loop_rate.sleep()
            continue
        vel_msg.angular.z = 4 * atan2(t.transform.translation.y, t.transform.translation.x)
        vel_msg.linear.x = 0.5 * (t.transform.translation.x ** 2 + t.transform.translation.y ** 2) ** 0.5
        vel_pub.publish(vel_msg)
        loop_rate.sleep()


def setup():
    rospy.init_node(NODE_NAME)
    try:
        carrot_frame = rospy.get_param("~carrot_frame")
        follower_frame = rospy.get_param("~follower_frame")
    except KeyError:
        rospy.logerr("Couldn't find both or one of `carrot_frame` and `follower_frame` parameter!")
        exit(1)
    chase(follower_frame, carrot_frame)


if __name__ == "__main__":
    setup()