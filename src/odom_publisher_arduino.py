#! /usr/bin/env python3.6

import serial
import time

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))

    if x[0] == 'v':
        time.sleep(1.0)
    else:
        time.sleep(0.1)

    data = arduino.readline()
    return data

if __name__ == '__main__':
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=57600, timeout=.1)

    topic = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.init_node('test_topic_publisher')
    rate = rospy.Rate(10) # 10hz

    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0
    vy = 0
    vth = 0

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    write_read("v1 0")

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        num = "o"
        data = write_read(num)
        data = data.decode("utf-8").split("; ")

        if num is 'o' and len(data) >= 4:
            print("V = ", data[0])
            print("Yaw = ", data[1])
            print("x = ", data[2])
            print("y = ", data[3])
        else:
            print(data)
            continue

        x = float(data[2])
        y = float(data[3])
        th = float(data[1])

        vx = float(data[0])
        vy = 0
        vth = 0

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        topic.publish(odom)

        last_time = current_time

        rate.sleep()

    write_read('p')
