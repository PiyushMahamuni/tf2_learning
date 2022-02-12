#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import pi, sin, cos

#CONSTANTS
NODE_NAME = "moving_carrot"

class CircularTFBroadcaster:

    def __init__(self, parent, child, radius: float = 1.5):
        tfb = TransformBroadcaster()
        tf = TransformStamped()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.z = 0
        tf.transform.rotation.x = tf.transform.rotation.y = tf.transform.rotation. z = 0.0
        tf.transform.rotation.w = 1

        loop_rate = rospy.Rate(25)
        dtheta = pi / 100
        # 25 freq and dtheta = pi/ 100 -> 1/8 rps
        theta = 0
        pi_2 = 2 * pi
        while not rospy.is_shutdown():
            tf.header.stamp = rospy.Time.now()
            tf.transform.translation.x = radius * cos(theta)
            tf.transform.translation.y = radius * sin(theta)
            tfb.sendTransform(tf)
            theta += dtheta
            if theta > pi_2:
                theta -= pi_2
            loop_rate.sleep()
            

def setup():
    rospy.init_node(NODE_NAME)

    try:
        parent_frame = rospy.get_param("~parent_frame")
        carrot_frame = rospy.get_param("~carrot_frame")
        radius = rospy.get_param("~radius")
    except KeyError:
        rospy.logerr("Couldn't get `parent_frame` and/or `carrot_frame` and/or `radius` parameter")
        exit(1)
    
    carrot = CircularTFBroadcaster(parent_frame, carrot_frame)
    rospy.spin() # very important, the class created and the periodic broadcast of transform
    # is happened by a seperate thread which also stops when this main thread stops


if __name__ == "__main__":
    setup()