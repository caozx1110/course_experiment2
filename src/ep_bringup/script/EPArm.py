#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File   :   EPArm.py
@Time   :   2022/10/21 17:10:52
@Author :   Cao Zhanxiang 
@Version:   1.0
@Contact:   caozx1110@163.com
@License:   (C)Copyright 2022
@Desc   :   Arm control
'''

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../../config"))
from config import config

from robomaster import robot
import numpy as np
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
import rospy
import time


class EPArm(object):
    """ ep arm """
    def __init__(self, ep: robot) -> None:
        """init

        :param robot ep: the robomaster.robot object
        """
        # actuator
        self.arm = ep.robotic_arm
        self.gripper = ep.gripper
        # ros communication
        self.end_subscriber = rospy.Subscriber(config.arm.end_topic_name, Point, self.end_callback, queue_size=10)
        self.gripper_subscriber = rospy.Subscriber(config.arm.gripper_topic_name, Int8, self.gripper_callback, queue_size=10)

    def end_callback(self, msg: Point):
        """callback function of the end point subscriber

        :param Point msg: the end point position
                            msg.x:   x position m
                            msg.y:   y position m
                            msg.z:   0
        """ 
        self.arm.moveto(x=msg.x * 1000, y=msg.y * 1000).wait_for_completed(timeout=1)
        time.sleep(1)
        
    def gripper_callback(self, msg: Int8):
        """callback function of the gripper subscriber

        :param Int8 msg: the gripper state
                            msg.data:   +: close    [1, 100]    power
                                        -: open     [-100, -1]  power
                                        0: pause
        """ 
        if msg.data > 0:
            self.gripper.pause()
            self.gripper.close(power=msg.data)
            # time.sleep(1)
        elif msg.data < 0:
            self.gripper.pause()
            self.gripper.open(power=-msg.data)
            # time.sleep(1)
        else:
            self.gripper.pause()
    

if __name__ == '__main__':
    pass
