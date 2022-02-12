#!/usr/bin/env python

from os import stat
import rospy

# to get command line arguments
import sys

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

NODE_NAME = "static_turtle_tf2_broadcaster"

def setup():
    if len(sys.argv) < 9:
        rospy.logerr(f"""Invalid number of parameters
        Usage: {sys.argv[0]}/static_turtle_tf2_broadcaster.py child_frame_name x y z roll pitch yaw""")
        sys.exit(1)
    elif sys.argv[2] == "world":
        rospy.logerr("Your static turtle name can't be `world`")
        sys.exit(1)
    elif sys.argv[1] == sys.argv[2]:
        rospy.logerr("Your parent and child frames can't be the same")
        sys.exit(1)
    
    rospy.init_node(NODE_NAME)
    broadcaster = StaticTransformBroadcaster()
    static_transform_stamped = TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = sys.argv[1]
    static_transform_stamped.child_frame_id = sys.argv[2]

    static_transform_stamped.transform.translation.x = float(sys.argv[3])
    static_transform_stamped.transform.translation.y = float(sys.argv[4])
    static_transform_stamped.transform.translation.z = float(sys.argv[5])
    q = quaternion_from_euler(float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]))
    static_transform_stamped.transform.rotation.x = q[0]
    static_transform_stamped.transform.rotation.y = q[1]
    static_transform_stamped.transform.rotation.z = q[2]
    static_transform_stamped.transform.rotation.w = q[3]

    broadcaster.sendTransform(static_transform_stamped)
    rospy.spin()


if __name__ == "__main__":
    setup()
