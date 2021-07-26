#!/usr/bin/env python3.6

import rospy
from geometry_msgs.msg import Twist, Vector3

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('arduino_cmd_listener')
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    vx = -0.1
    vy = 0
    vth = 1
    twistMsg = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    pub.publish(twistMsg)
    r.sleep()
