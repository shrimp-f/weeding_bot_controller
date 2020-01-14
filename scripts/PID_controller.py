#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
import math


Kp = 5.
Ki = 0.01
Kd = 1
ratio = 0.001

target_y = 0.0

# ロボットの前進速度
CONST_X_LINEAR = 0.2

# publishするトピック
#PUBLISH_TOPIC = '/my_robo_two/diff_drive_controller/cmd_vel'
PUBLISH_TOPIC = '/cabbage1/cmd_vel_after'


# delta_t 実行時間間隔dt rostopic hz /estimator_linear/center_distance から
delta_t = 1. / 21

P, I, D, pre_P, duty = 0,0,0,0,0

def pid_control(y):
    global P, I, D, pre_P, duty

    P = target_y - y
    I += P * delta_t
    D = (P-pre_P) / delta_t
    pre_P = P

    duty = Kp * P + Ki * I + Kd * D
    output = duty * ratio
    rospy.loginfo("output: %f", output)
    return output




def callback_distance(msg):
    global center_distance
    center_distance = msg.data
#    rospy.loginfo("center_distance: %f", center_distance)


def callback_angle(msg):
    global ridge_angle
    ridge_angle = msg.data
    rospy.loginfo("ridge_angle: %f", ridge_angle)

    rospy.loginfo("center_distance: %f", center_distance)

    # 速度を格納する変数
    move_cmd = Twist()

    # 入力を計算
    u = pid_control(center_distance)
    
    # 速度をセット
    move_cmd.linear.x = CONST_X_LINEAR
    move_cmd.angular.z = u
#    rospy.loginfo("u0: %f", u[0,0])


    # twist型をpublish
    pub = rospy.Publisher(PUBLISH_TOPIC, Twist, queue_size=3)
    pub.publish(move_cmd)
#    rospy.loginfo("x_l: %f", move_cmd.linear.x)
#    rospy.loginfo("z_a: %f", move_cmd.angular.z)


def controller():
    rospy.init_node('controller')
    rospy.Subscriber("/estimator_linear/center_distance", Float32, callback_distance)
    rospy.Subscriber("/estimator_linear/ridge_angle", Float32, callback_angle)

    rospy.spin()


if __name__ == '__main__':

    controller()

