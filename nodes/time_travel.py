#!/usr/bin/env python

import rospy
from tf2_ros import TransformBroadcaster, TransformListener, Buffer

# CONSTANTS
NODE_NAME = "time_travel"


def setup():
    rospy.init_node(NODE_NAME)
    fixed_frame = rospy.get_param("~fixed_frame")
    target_frame = rospy.get_param("~target_frame")
    source_frame = rospy.get_param("~source_frame")
    si_past = rospy.get_param("~seconds_in_past")
    
    tfb = Buffer()
    tfl = TransformListener(tfb)
    while not rospy.is_shutdown():
        current = rospy.Time.now()
        tfb.lookup_transform_full(target_frame=target_frame, target_time=current, source_frame=source_frame, source_time=current-si_past, fixed_frame=fixed_frame, timeout=rospy.Duration(0.5))
