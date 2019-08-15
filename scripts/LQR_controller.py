#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

center_distaince = 0.
ridge_angle = 0.


def callback_distance(msg):
    center_distaince = msg.data
    rospy.loginfo("center_distace: %f", center_distaince)


def callback_angle(msg):
    ridge_angle = msg.data
    rospy.loginfo("ridge_angle: %f", ridge_angle)

    pub = rospy.Publisher('/my_robo_two/diff_drive_controller/cmd_vel', Twist, queue_size=3)
    
    
    move_cmd = Twist()


    # 速度をセット
    move_cmd.linear.x = 1.0
    move_cmd.angular.z = 0.1

    # twist型をpublish
    pub.publish(move_cmd)
    rospy.loginfo("publish: %f", move_cmd.linear.x)


def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/estimator_linear/center_distance", Float32, callback_distance)
    rospy.Subscriber("/estimator_linear/ridge_angle", Float32, callback_angle)

    rospy.spin()


if __name__ == '__main__':
    listener()