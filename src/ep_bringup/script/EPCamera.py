#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File   :   EPCamera.py
@Time   :   2022/10/21 13:13:50
@Author :   Cao Zhanxiang 
@Version:   1.0
@Contact:   caozx1110@163.com
@License:   (C)Copyright 2022
@Desc   :   Camera class
'''

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../../config"))
from config import config

from robomaster import robot
import threading
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header, Float32MultiArray
from geometry_msgs.msg import TransformStamped
import rospy
import tf2_ros, tf_conversions
from cv_bridge import CvBridge
import math

class EPCamera(object):
    """ ep camera """
    def __init__(self, ep: robot) -> None:
        """init

        :param robot ep: the robomaster.robot object
        """
        self.camera = ep.camera
        
        # ros communication
        self.camera_img_publiser = rospy.Publisher(config.camera.img_topic_name, Image, queue_size=10)
        self.img_msg_header = Header()
        
        self.camera_info_publisher = rospy.Publisher(config.camera.info_topic_name, CameraInfo, queue_size=10)
        self.camera_info_msg = CameraInfo()
        
        # the camera tf related to the arm frame
        # sub the servos' angle
        self.angle_subscriber = rospy.Subscriber(config.servo.angle_topic_name, Float32MultiArray, self.angle_callback, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_msg = TransformStamped()
        
        # camera thread
        self.thread_camera = threading.Thread(target=self.pub_img_task)
        
    def angle_callback(self, msg: Float32MultiArray):
        """callback of the servo angle
        param: msg Float32MultiArray: the msg of the servo angle
        """
        # NOTE: servo2 is shoulder servo, servo1 is elbow servo
        agl1 = config.arm.agl1_base - msg.data[1]
        agl2 = msg.data[0] - config.arm.agl2_base - agl1    # NOTE：几何关系 舵机角度不等于肘关节角度
        
        # msg
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.header.seq += 1
        self.tf_msg.header.frame_id = config.arm.frame_id
        self.tf_msg.child_frame_id = config.camera.frame_id
        
        self.tf_msg.transform.translation.x = 0
        self.tf_msg.transform.translation.y = config.arm.l1 * math.cos(agl1)
        self.tf_msg.transform.translation.z = config.arm.l1 * math.sin(agl1)
        
        # rospy.logerr("agl1: %f, agl2: %f", agl1, agl2)
        rot_x = agl1 + agl2 - math.pi / 12  # TODO: add to config & measure the real value
        # the frame of the camera is different from the frame of the arm , roate 180 degree around z axis
        q = tf_conversions.transformations.quaternion_from_euler(rot_x, 0, math.pi, 'rxyz')     # 绕自身转轴旋转
        self.tf_msg.transform.rotation.x = q[0]
        self.tf_msg.transform.rotation.y = q[1]
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]
        
        # pub tf
        print(self.tf_msg)
        self.tf_broadcaster.sendTransform(self.tf_msg)
    
    def pub_img_task(self):
        """publish the camera image
        Frequency not fixed
        open a single thread
        once getting a img from the sdk, pub the img
        """
        self.camera.start_video_stream(display=config.camera.display, resolution=config.camera.resolution)
        bridge = CvBridge()
        
        while not rospy.is_shutdown():
            try:
                frame = self.camera.read_cv2_image()
                # msg
                self.img_msg_header.stamp = rospy.Time.now()
                self.img_msg_header.seq += 1
                self.img_msg_header.frame_id = config.camera.frame_id
                
                msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8", header=self.img_msg_header)
                # publish
                self.camera_img_publiser.publish(msg)
            except:
                pass
    
    def pub_info(self):
        """publish the camera info
        Frequency fixed
        """
        self.camera_info_msg.header.stamp = rospy.Time.now()
        self.camera_info_msg.header.seq += 1
        self.camera_info_msg.header.frame_id = config.camera.frame_id
        self.camera_info_msg.D = config.camera.info.D
        self.camera_info_msg.K = config.camera.info.K
        self.camera_info_msg.R = config.camera.info.R
        self.camera_info_msg.P = config.camera.info.P
        
        self.camera_info_publisher.publish(self.camera_info_msg)
    
    def pub_tf(self):
        """publish the camera tf
        """
        pass
    
    def pub_img(self):
        """start the img_publish thread
        """
        self.thread_camera.start()
    
    def stop(self):
        """stop the camera
        """
        self.camera.stop_video_stream()
        self.thread_camera.join()

        
if __name__ == '__main__':
    pass
