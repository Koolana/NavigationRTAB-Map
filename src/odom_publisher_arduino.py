#! /usr/bin/env python3.6

import serial
import time

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

current_msg = "p"
input_msg_avaible = True

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    # arduino.flush();

    if x[0] == 'v':
        time.sleep(1.0)
    else:
        time.sleep(0.1)

    data = arduino.readline()
    return data

def callback(data):
    global current_msg
    global input_msg_avaible

    current_msg = "v" + str(data.linear.x) + " " + str(data.angular.z)
    print("Update msg: ", current_msg)
    input_msg_avaible = True


if __name__ == '__main__':
    arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
    print(serial.VERSION)

    topic = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.init_node('test_topic_publisher')
    rate = rospy.Rate(1) # 10hz

    rospy.Subscriber("cmd_vel", Twist, callback)

    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0
    vy = 0
    vth = 0

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    time.sleep(1)  # waiting arduino

    # write_read("v0.1 0")

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        num = "o"
        data = write_read(num)
        data = data.decode("utf-8").split("; ")
        print(data)

        if num is 'o' and len(data) >= 5:
            print("V = ", data[0])
            print("Omega = ", data[1])

            print("Yaw = ", data[2])
            print("x = ", data[3])
            print("y = ", data[4])
        else:
            # print(data)
            continue

        x = float(data[3])
        y = float(data[4])
        th = float(data[2])

        vx = float(data[0])
        vy = 0
        vth = float(data[1])

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

        if input_msg_avaible:
            arduino.write(bytes(current_msg, 'utf-8'))
            time.sleep(1)
            input_msg_avaible = False

        last_time = current_time

        rate.sleep()

    write_read('p')
