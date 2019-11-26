#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
20191126
after modelization

"""

import rospy
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
import math


# ロボットの前進速度
CONST_X_LINEAR = 0.2

# publishするトピック
#PUBLISH_TOPIC = '/my_robo_two/diff_drive_controller/cmd_vel'
PUBLISH_TOPIC = '/cabbage1/cmd_vel'


# delta_t 実行時間間隔dt rostopic hz /estimator_linear/center_distance から
# delta_t = 1. / 21
B_para = 0.05

#### LQR 関係 #################
# x[k+1] = Ax[k] + Bu[k]
A = np.matrix([[1.0, 0.0084], 
                [0, 1.0]])
B = np.matrix([[0.0, -0.0160],
                [0.0, 0.0421]])
C = np.matrix([1.0, 0.0])


#    Q = np.eye(2)
Q = np.matrix([[1.0, 0], 
                [0, 1.0]])
#    R = 5*np.eye(2)
R = np.matrix([[10.0, 0], 
                [0, 10.0]])

Kopt = None
#### LQR 関係 おわり#################


def dlqr(A, B, Q, R):
    """
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    # solve the ricatti equation ############
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T * X * A - A.T * X * B * np.linalg.inv(R + B.T * X * B) * B.T * X * A + Q
        if (abs(Xn - X)).max() < eps:
            X = Xn
            break
        X = Xn
    ##########################################

    # LQR gain
    K = np.matrix(np.linalg.inv(B.T * X * B + R) * (B.T * X * A))
    return K, X


def lqr_control(x):
    global Kopt
#    B = np.matrix([[x[1,0]*delta_t + 1.0, 0.0],
#                    [0.0, -1.0]])

#    Kopt, X, ev = dlqr(A, B, C.T * np.eye(2) * C, np.eye(2))
    # Bが固定になったので、Koptを1回だけ計算する
    if Kopt is None:
        Kopt, X = dlqr(A, B, Q, R)
    
    u = -Kopt * x
    return u


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

    # 状態変数
    x = np.matrix([center_distance, ridge_angle]).T # [y, theta]
    u = np.matrix([0.0, 0.0]).T # [x_l, z_a]

    # LQRで最適な入力を計算
    u = lqr_control(x)
    
    # 速度をセット
    move_cmd.linear.x = u[0,0]*10 + CONST_X_LINEAR
    move_cmd.angular.z = 1.* u[1,0]
#    rospy.loginfo("u0: %f", u[0,0])


    # twist型をpublish
    pub = rospy.Publisher(PUBLISH_TOPIC, Twist, queue_size=3)
    pub.publish(move_cmd)
    rospy.loginfo("x_l: %f", move_cmd.linear.x)
    rospy.loginfo("z_a: %f", move_cmd.angular.z)


def controller():
    rospy.init_node('controller')
    rospy.Subscriber("/estimator_linear/center_distance", Float32, callback_distance)
    rospy.Subscriber("/estimator_linear/ridge_angle", Float32, callback_angle)

    rospy.spin()


if __name__ == '__main__':

    controller()

