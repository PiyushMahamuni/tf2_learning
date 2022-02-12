#!/usr/bin/env python
import rospy

import tf_conversions

import tf2_ros
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose

# CONSTANTS
NODE_NAME = "tf2_turtle_broadcaster"

# GLOBALS
POSE_TOPIC = None # initialized to proper value in main()

# callback function to broadcast transform (from world to this frame) whenever we receive pose data
def handle_turtle_pose(msg : Pose, turtlename : str):
    # Now, we create a Transform object and give it the appropriate metadata.
    # We need to give the transform being published a timestamp, we'll just stamp it with
    # the current time, ros::Time::now().
    # Then, we need to set the name of the parent frame of the link we're creating,
    # in this case "world"
    # Finally, we need to set the name of the child node of the link we're creating,
    # in this case this is the name of the turtle itself.
    
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


def main():
    rospy.init_node(NODE_NAME)
    turltename = rospy.get_param('~turtle')
    # This node takes a single parameter "turtle", which specifies a turtle name, e.g. "turtle1" or "turtle2".
    POSE_TOPIC = f"{turltename}/pose"
    rospy.Subscriber(POSE_TOPIC, Pose, handle_turtle_pose, turltename)
    rospy.spin()


if __name__ == "__main__":
    main()

