#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File   :   ep_bringup.py
@Time   :   2022/10/19 20:24:14
@Author :   Cao Zhanxiang 
@Version:   1.0
@Contact:   caozx1110@163.com
@License:   (C)Copyright 2022
@Desc   :   None
'''
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../../config"))
from config import config

from EPCamera import EPCamera
from EPChassis import EPChassis
from EPArm import EPArm
from EPServo import EPServo
from robomaster import conn, robot
from MyQR import myqr
import cv2
import rospy
import time


# 连接方式为STA组网模式
class EPRobot(object):
    def __init__(self):
        """init function
        """
        # connect
        self.ep_robot = robot.Robot()
        try:
            self.ep_robot.initialize(conn_type="sta")
        except:
            self.connect_wifi()
            self.ep_robot.initialize(conn_type="sta")
        
        rospy.loginfo("EP Robot connected!")
        
        # sensors & actuator
        self.camera = EPCamera(self.ep_robot)
        self.chassis = EPChassis(self.ep_robot)
        self.arm = EPArm(self.ep_robot)
        self.servo = EPServo(self.ep_robot)
        
        self.SN = self.ep_robot.get_sn()
        rospy.loginfo("Robot SN: " + self.SN)
        # print("Robot SN:", self.SN)
        
        # start the camera thread
        self.camera.pub_img()
        
        
    def connect_wifi(self):
        """generate the QR code & connect the wifi
        """
        helper = conn.ConnectionHelper()
        # build qr code
        info = helper.build_qrcode_string(ssid=config.wifi.ssid, password=config.wifi.password)
        _, _, qr_name = myqr.run(words=info)
        
        # show the QR code
        cv2.imshow("QR Code", cv2.imread(qr_name))
        cv2.waitKey(0)
        if helper.wait_for_connection():
            print("Connected!")
            cv2.destroyAllWindows()
        else:
            print("Connect failed!")
            
    def publish(self):
        """publish the data
        """
        self.camera.pub_info()
        self.servo.pub_angles()
        self.camera.pub_tf()
            
    def close(self):
        """close the robot
        """
        self.ep_robot.close()
    
if __name__ == '__main__':
    rospy.init_node('ep_bringup')
    rate = rospy.Rate(config.rate)

    ep = EPRobot()
    
    while not rospy.is_shutdown():
        ep.publish()
        rate.sleep()
    
    rospy.loginfo("Closing the robot...")
    ep.close()
