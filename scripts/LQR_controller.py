#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
# import scipy.linalg as la
import math

# ロボットの前進速度
#CONST_X_LINEAR = 0.5
TARGET_X_LINEAR = 1.0
TREAD = 0.61 # m
pre_x_l = 0.0 # 一個前の速度

# delta_t 実行時間間隔dt rostopic hz /estimator_linear/center_distance から
delta_t = 1. / 21

# センサーデータ
#center_distance = 0.
#ridge_angle = 0.

#### LQR 関係 #################
# x[k+1] = Ax[k] + Bu[k]
'''
A = np.matrix([[1.0, 0], 
                [0, 1.0]])
B = np.matrix([[0., 0.],
                [1./TREAD, -1./TREAD]])
#C = np.matrix([1.0, 0.0])
'''
A = np.matrix([[1.0, 0, 0], 
                [0, 1.0, 0],
                [0, 0, 1.0]])
B = np.matrix([[0., 0., 0],
                [1./TREAD, -1./TREAD, 0],
                [0.5, 0.5, 0]])

Kopt = None
Q = np.matrix([[1.0, 0, 0], 
                [0, 1.0, 0],
                [0, 0, 1.0]])
R = np.matrix([[5.0, 0, 0], 
                [0, 5.0, 0],
                [0, 0, 5.0]])

'''
Q = np.matrix([[1.0, 0], 
                [0, 1.0]])
R = np.matrix([[5.0, 0], 
                [0, 5.0]])
'''
#### LQR 関係 おわり#################


def dlqr(A, B, Q, R):
    """
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    """
    print("def dlqr")
    print("A\n", A)
    print("B\n", B)
    print("Q\n", Q)
    print("R\n", R)
    """

    # solve the ricatti equation ############
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
#        Xn = A.T * X * A - A.T * X * B * la.inv(R + B.T * X * B) * B.T * X * A + Q
        ababa =  B.T * X * B
        Xn = A.T * X * A - A.T * X * B * np.linalg.inv(R + B.T * X * B) * B.T * X * A + Q
        if (abs(Xn - X)).max() < eps:
            X = Xn
            break
        X = Xn
    ##########################################

    # LQR gain
#    K = np.matrix(la.inv(B.T * X * B + R) * (B.T * X * A))
    K = np.matrix(np.linalg.inv(B.T * X * B + R) * (B.T * X * A))
    return K, X


def lqr_control(x):
    global Kopt
#    B = np.matrix([[x[1,0], 0.0],
#                    [0.0, 1.0]])
    '''
    B = np.matrix([[-0.5*x[1,0] * delta_t, -0.5*x[1,0] * delta_t],
                    [1./TREAD, -1./TREAD]])
    '''
#    '''
    B = np.matrix([[-0.5*x[1,0] * delta_t, -0.5*x[1,0] * delta_t, 0],
                    [1./TREAD, -1./TREAD, 0],
                    [0.5,      0.5,       0]])
#    '''

    # Kopt毎回計算する
#    Kopt, X, ev = dlqr(A, B, C.T * np.eye(2) * C, np.eye(2))
#    Q = np.eye(2)
    Kopt, X = dlqr(A, B, Q, R)
#    print("Kopt")
#    print(Kopt)
    
    u = -Kopt * x
    return u


######################


def callback_distance(msg):
    global center_distance
    center_distance = msg.data
    rospy.loginfo("center_distance: %f", center_distance)


def callback_angle(msg):
    ridge_angle = msg.data
    rospy.loginfo("ridge_angle: %f", ridge_angle)

    # 状態変数
    x = np.matrix([center_distance, ridge_angle, TARGET_X_LINEAR - pre_x_l]).T # [y, theta]
    rospy.loginfo("x0: %f", x[0,0])
    rospy.loginfo("x1: %f", x[1,0])
    u = np.matrix([0.0, 0.0, 0]).T # [x_l, z_a, 0]
#    B = np.matrix([[x[1,0], 0.0],
#                    [0.0, 1.0]])

    # LQRで最適な入力を計算。このuはv_L,v_R
    u = lqr_control(x)
    rospy.loginfo("v_L: %f", u[0,0])
    rospy.loginfo("v_R: %f", u[1,0])
    v_ratio = 0.01
    v_L = u[0,0] * v_ratio
    v_R = u[1,0] * v_ratio

    # 速度をセット
    move_cmd = Twist()
    move_cmd.linear.x = (v_L + v_R)/2.
    pre_x_a = (v_L + v_R)/2.
    move_cmd.angular.z = (v_R - v_L)/TREAD *10.

    # twist型をpublish
    pub = rospy.Publisher('/my_robo_two/diff_drive_controller/cmd_vel', Twist, queue_size=3)
    pub.publish(move_cmd)
    rospy.loginfo("x_l: %f", move_cmd.linear.x)
    rospy.loginfo("z_a: %f", move_cmd.angular.z)


def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/estimator_linear/center_distance", Float32, callback_distance)
    rospy.Subscriber("/estimator_linear/ridge_angle", Float32, callback_angle)

    rospy.spin()


if __name__ == '__main__':
    listener()

