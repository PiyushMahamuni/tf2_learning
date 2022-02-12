#!/usr/bin/env python

# spawns the turtle and adds frame for it

import rospy
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_conversions import transformations

# CONSTANTS
NODE_NAME = "setup_turtle"
SPAWN_SERVICE = "/spawn"

# GLOBALS
turtle = None # will be initialized to proper value by setup()
POSE_TOPIC = None # will be initialized to proper value by setup()
pose_sub = None
tf_br : TransformBroadcaster= None


def publish_transform(pose: Pose, transform: TransformStamped):
    transform.header.stamp = rospy.Time.now()
    transform.transform.translation.x = pose.x
    transform.transform.translation.y = pose.y
    transform.transform.translation.z = 0.0
    q = transformations.quaternion_from_euler(0.0, 0.0, pose.theta)
    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]
    tf_br.sendTransform(transform)
    


def setup():
    rospy.init_node(NODE_NAME)
    global turtle
    try:
        turtle = rospy.get_param("~turtle")
    except KeyError:
        rospy.logerr("Didn't find `turtle` parameter to setup a turtle!")
        exit(1)
    rospy.wait_for_service(SPAWN_SERVICE)
    try:
        rospy.ServiceProxy(SPAWN_SERVICE, Spawn).call(5.0, 5.0, 0.0, turtle)
    except rospy.ServiceException:
        pass

    # setup a tranform publisher
    global tf_br
    tf_br = TransformBroadcaster()
    # attach frame to spawned turtle
    global POSE_TOPIC, pose_sub
    POSE_TOPIC = f"{turtle}/pose"
    # setup transform metadata except for time
    tf = TransformStamped()
    tf.header.frame_id = "world"
    tf.child_frame_id = turtle
    pose_sub = rospy.Subscriber(POSE_TOPIC, Pose, publish_transform, tf)
    rospy.spin()


if __name__ == "__main__":
    setup()
