#!/usr/bin/env python

import rospy
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, ConnectivityException, LookupException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

# CONSTANTS
NODE_NAME = "time_travel"


def setup():
    rospy.init_node(NODE_NAME)
    fixed_frame = rospy.get_param("~fixed_frame")
    target_frame = rospy.get_param("~target_frame")
    source_frame = rospy.get_param("~source_frame")
    si_past = rospy.Duration(rospy.get_param("~seconds_in_past"))

    tfb = Buffer()
    tfl = TransformListener(tfb)
    tb = TransformBroadcaster()

    loop_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        current = rospy.Time.now()
        try:
            t: TransformStamped = tfb.lookup_transform_full(
                target_frame, current, source_frame, current-si_past, fixed_frame, rospy.Duration(0.3))
        except (ConnectivityException, LookupException, ExtrapolationException):
            loop_rate.sleep()
            continue
        
        t.header.stamp = current
        t.header.frame_id = target_frame
        t.child_frame_id = f"past_{source_frame}"
        tb.sendTransform(t)
        loop_rate.sleep()


if __name__ == "__main__":
    setup()
