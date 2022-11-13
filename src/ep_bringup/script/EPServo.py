#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File   :   EPServo.py
@Time   :   2022/10/21 14:48:50
@Author :   Cao Zhanxiang 
@Version:   1.0
@Contact:   caozx1110@163.com
@License:   (C)Copyright 2022
@Desc   :   EP servo ros class
'''

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../../config"))
from config import config

from robomaster import robot
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import rospy
# from ep_bringup.msg import servo_cmd


class EPServo(object):
    def __init__(self, ep: robot) -> None:
        self.servo = ep.servo
        
        # ros communication
        # self.cmd_subscriber = rospy.Subscriber(config.servo.cmd_topic_name, servo_cmd, self.cmd_callback, queue_size=10)
        self.angle_publisher = rospy.Publisher(config.servo.angle_topic_name, Float32MultiArray, queue_size=10)
        self.angle_msg = Float32MultiArray()
    
    def pub_angles(self):
        """pub the servos' angles
            msg.data:   the angles of the servos
            data[0]:    the angle of the servo 1 in rad
            data[1]:    the angle of the servo 2 in rad
            data[2]:    the angle of the servo 3 in rad
            NOTE:       servo2 is shoulder servo, servo1 is elbow servo
        """
        angle = [self.servo.get_angle(index=i) * np.pi / 180 for i in range(1, 4)]
        self.angle_msg.data = angle
        
        self.angle_publisher.publish(self.angle_msg)

    # def cmd_callback(self, msg: servo_cmd):
    #     """callback function of the servo command subscriber

    #     :param servo_cmd msg: the servo command
    #                             msg.index:   servo id     [1, 3]    int
    #                             msg.angle:   servo angle  [-pi, pi] rad
    #     """     
    #     idx = msg.index
    #     angle = int(msg.angle * 180 / np.pi)
    #     self.servo.moveto(index=idx, angle=angle).wait_for_completed()
        
        
if __name__ == '__main__':
    pass
