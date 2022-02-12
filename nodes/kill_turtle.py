#!/usr/bin/env python

import rospy
from turtlesim.srv import Kill

#CONSTANTS
NODE_NAME = "kill_turtle"
SERVICE = "kill"

# GLOBALS
turtle_to_kill = None # will be initialzed to proper value in main()

def main():
    rospy.init_node(NODE_NAME)
    rospy.wait_for_service(SERVICE)
    global turtle_to_kill
    turtle_to_kill = rospy.get_param("turtle_to_kill", "turtle1")
    killer = rospy.ServiceProxy(SERVICE, Kill)
    killer.call(turtle_to_kill)


if __name__ == "__main__":
    main()