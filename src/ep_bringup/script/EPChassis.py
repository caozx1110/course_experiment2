#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File   :   EPChassis.py
@Time   :   2022/10/21 14:12:18
@Author :   Cao Zhanxiang 
@Version:   1.0
@Contact:   caozx1110@163.com
@License:   (C)Copyright 2022
@Desc   :   ep chassis control
'''

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../../config"))
from config import config

from robomaster import robot
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
import rospy

class EPChassis(object):
    """ ep chassis """
    def __init__(self, ep: robot) -> None:
        """init

        :param robot ep: the robomaster.robot object
        """
        self.chassis = ep.chassis
        
        # ros communication
        self.vel_subscriber = rospy.Subscriber(config.chassis.cmd_topic_name, Twist, self.vel_callback, queue_size=10)
        
    def vel_callback(self, msg: Twist):
        """callback function of the velocity subscriber

        :param Twist msg: the velocity command
                            msg.linear.x:   forward velocity        [-3.5,3.5]          m/s 
                            msg.linear.y:   right velocity          [-3.5,3.5]          m/s
                            msg.angular.z:  clockwise yaw velocity  [-10/3 pi, 10/3 pi] rad/s
        """     
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z * 180 / np.pi
        self.chassis.drive_speed(x=x, y=y, z=z, timeout=config.chassis.timeout)
    
    #TODO: pub the chassis state
    

if __name__ == '__main__':
    pass
