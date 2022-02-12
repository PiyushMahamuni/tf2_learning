#!/usr/bin/env python

import rospy
import tf2_msgs.msg
import geometry_msgs.msg

# CONSTANTS
NODE_NAME = "fixed_tf2_broadcaster"

class FixedTFBroadcaster:
    TF_TOPIC = "/tf"
    def __init__ (self, frame_name: str="carrot", parent_frame: str="turtle1"):
        self.pub_tf = rospy.Publisher(FixedTFBroadcaster.TF_TOPIC, tf2_msgs.msg.TFMessage, queue_size=1)

        loop_rate = rospy.Rate(20)
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent_frame
        t.child_frame_id = frame_name
        t.transform.translation.x = 0.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = t.transform.rotation.y = t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1
        tfm = tf2_msgs.msg.TFMessage([t])
        while not rospy.is_shutdown():
            loop_rate.sleep()
            t.header.stamp = rospy.Time.now()
            self.pub_tf.publish(tfm)


def setup():
    rospy.init_node(NODE_NAME)
    fixed_frame = rospy.get_param("~carrot_frame", "carrot")
    parent_frame = rospy.get_param("~parent_frame", "turtle1")
    tfb = FixedTFBroadcaster(fixed_frame, parent_frame)
    rospy.spin()


if __name__ == "__main__":
    setup()
