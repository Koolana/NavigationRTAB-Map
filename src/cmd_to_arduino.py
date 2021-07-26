#!/usr/bin/env python3.6

import rospy
from geometry_msgs.msg import Twist, Vector3

def callback(data):
    msgToArduino = "v" + str(data.linear.x) + " " + str(data.angular.z)
    print(msgToArduino)

if __name__ == '__main__':
    rospy.init_node('cmd_listener')
    rospy.Subscriber("cmd_vel", Twist, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
