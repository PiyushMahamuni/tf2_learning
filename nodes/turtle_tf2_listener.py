#!/usr/bin/env python
import rospy
import tf2_ros
import math
import geometry_msgs.msg
import turtlesim.srv

# CONSTANTS
NODE_NAME = "tf2_turtle_listener"
SPAWN_SERVICE = "spawn"

# GLOBALS
turtle_vel = None # will be initialized by main
VEL_TOPIC = None # will be initialized by main
turtle_name = None # will be initialized by main
turtle_to_follow = None # will be initialized by main

def main():
    rospy.init_node(NODE_NAME)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.wait_for_service(SPAWN_SERVICE)
    spawner = rospy.ServiceProxy(SPAWN_SERVICE, turtlesim.srv.Spawn)
    global turtle_name, turtle_to_follow
    turtle_name = rospy.get_param("~turtle", "turtle2")
    # returns the second arg if failed to retrieve the parameter
    turtle_to_follow = rospy.get_param("~turtle_to_follow")
    spawner(4, 2, 0, turtle_name)

    global VEL_TOPIC
    VEL_TOPIC = f"{turtle_name}/cmd_vel"
    global turtle_vel
    turtle_vel = rospy.Publisher(VEL_TOPIC, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10)
    vel_cmd = geometry_msgs.msg.Twist()
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(turtle_name, turtle_to_follow, rospy.Time(), rospy.Duration(0.5))
            # the fourt duration argument is optional and is a timeout period, it will wait for that many seconds
            # to see if the required transform is available
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        vel_cmd.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        vel_cmd.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        turtle_vel.publish(vel_cmd)
        rate.sleep()


if __name__ == "__main__":
    main()