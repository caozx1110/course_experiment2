#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
'''
@File   :   visual_servo.py
@Time   :   2022/10/23 08:59:47
@Author :   Cao Zhanxiang 
@Version:   1.0
@Contact:   caozx1110@163.com
@License:   (C)Copyright 2022
@Desc   :   视觉伺服 抓取物块
'''

from operator import pos
import sys, os
from turtle import update
sys.path.append(os.path.join(os.path.dirname(__file__), "../../config"))
from config import config

import cv2
import numpy as np
import time
import traceback
import rospy
import tf2_ros
from tf2_geometry_msgs import PoseStamped
import tf_conversions
from std_msgs.msg import Int8, Float32MultiArray
from geometry_msgs.msg import Point, Twist, Pose
from marker_location.msg import markers

class grabTask:
    def __init__(self) -> None:
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message(config.marker.topic_name, markers, timeout=5.0)
            except KeyboardInterrupt:
                traceback.print_exc()
                quit()
            except:
                rospy.logwarn("Waiting for topic")
                continue
            try:
                rospy.wait_for_message(config.servo.angle_topic_name, Float32MultiArray, timeout=5.0)
            except KeyboardInterrupt:
                traceback.print_exc()
                quit()
            except:
                rospy.logwarn("Waiting for topic")
                continue
            break
        
        # ros communication
        
        self.end_publisher = rospy.Publisher(config.arm.end_topic_name, Point, queue_size=10)
        self.gripper_publisher = rospy.Publisher(config.arm.gripper_topic_name, Int8, queue_size=10)
        self.chassis_publisher = rospy.Publisher(config.chassis.cmd_topic_name, Twist, queue_size=10)
        
        # init
        self.tgt_id = 0
        self.tgt_pose = Pose()
        self.tgt_end_msg = Point()
        self.is_tgt = False
        self.is_done = False
        self.times_count = 0
        
        self.marker_subscriber = rospy.Subscriber(config.marker.topic_name, markers, self.marker_callback, queue_size=10)
        self.servo_angle_subscriber = rospy.Subscriber(config.servo.angle_topic_name, Float32MultiArray, self.servo_angle_callback, queue_size=10)

        # transform
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        while not rospy.is_shutdown():
            try:
                trans = self.buffer.lookup_transform(config.camera.frame_id, config.arm.frame_id, rospy.Time())
            except KeyboardInterrupt:
                traceback.print_exc()
                quit()
            except:
                rospy.logwarn_once("Waiting for topic")
                continue
            break
        
        self.init_arm()
        
    def chassis_setvel(self, vx: float=0, vy: float=0, vw: float=0):
        """set the chassis velocity

        :param float vx: x velocity m/s, defaults to 0
        :param float vy: y velocity m/s, defaults to 0
        :param float vw: angular velocity rad/s, defaults to 0
        """
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = vw
        self.chassis_publisher.publish(msg)
        
    def arm_moveto(self, x: float, y: float):
        """move the arm to the target position

        :param float x: x position m
        :param float y: y position m
        """
        self.tgt_end_msg.x = x
        self.tgt_end_msg.y = y
        self.tgt_end_msg.z = 0.0
        print("move to: x: {:.3f}, y: {:.3f}".format(x, y))
        self.end_publisher.publish(self.tgt_end_msg)
        
    def gripper_open(self, power: int=50):
        """open the gripper

        :param int power: power, defaults to 50
        """
        msg = Int8()
        msg.data = -power
        self.gripper_publisher.publish(msg)
    
    def gripper_close(self, power: int=50):
        """close the gripper

        :param int power: power, defaults to 50
        """
        msg = Int8()
        msg.data = power
        self.gripper_publisher.publish(msg)
        
    def servo_angle_callback(self, msg: Float32MultiArray):
        """callback function of the servo angle subscriber
        
        :param Float32MultiArray msg: the servo angle
        """
        # 相机所在关节角度
        angle = msg.data[0]
        
    def marker_callback(self, msg: markers):
        """callback function of the marker subscriber
        
        :param markers msg: the marker location
        """
        idx_list = msg.ids.data
        pose_list = msg.poses
        
        # 找最上面的marker
        # self.tgt_idx = -1 means the target has been grabed
        if len(idx_list) > 0:
            idx = idx_list.index(idx_list[-1])
            self.tgt_id = idx_list[idx]
            
            pose_stamp = PoseStamped()
            pose_stamp.header = msg.header
            pose_stamp.pose = pose_list[idx]
            # transform the pose to arm frame
            try:
                self.tgt_pose = self.buffer.transform(pose_stamp, config.arm.frame_id, timeout=rospy.Duration(1)).pose
                self.times_count += 1
                # 等到稳定
                if self.times_count > 10:
                    self.is_tgt = True
            except Exception as e:
                rospy.logerr(e)
            
    
    def update(self):
        def _delta(current, target, tol=0.01):
            if abs(current - target) > tol:
                return current - target
            else:
                return 0
            
        def _restrict_angle(angle):
            if angle > np.pi:
                angle -= 2 * np.pi
            elif angle < -np.pi:
                angle += 2 * np.pi
            return angle
            
        if self.is_tgt and (not self.is_done):
            # TODO: strategy can be changed here
            delta_x = -_delta(self.tgt_pose.position.x, config.chassis.x_target, config.chassis.x_tolerance)
            delta_z = _delta(self.tgt_pose.position.z, config.chassis.z_target, config.chassis.z_tolerance)
            r, p, y = tf_conversions.transformations.euler_from_quaternion([self.tgt_pose.orientation.x, 
                                                                self.tgt_pose.orientation.y, 
                                                                self.tgt_pose.orientation.z, 
                                                                self.tgt_pose.orientation.w])
            # 由于相机坐标系方向和机械臂坐标系方向不同
            delta_w = _restrict_angle(_delta(p, config.chassis.yaw_target, config.chassis.yaw_tolerance))

            rospy.loginfo_once("x: {:.3f}, y: {:.3f}, z: {:.3f}, r: {:.3f}, p: {:.3f}, y: {:.3f}, delta_x: {:.4f}, delta_z: {:.4f}, delta_w: {:.4f}".format(self.tgt_pose.position.x, 
                                                                                            self.tgt_pose.position.y, 
                                                                                            self.tgt_pose.position.z, 
                                                                                            r, p, y,
                                                                                            delta_x, delta_z, delta_w))
            
            if delta_x != 0 or delta_w != 0:
                delta_x = np.clip(delta_x, -0.1, 0.1)
                # delta_z = np.clip(delta_z, -0.1, 0.1)
                delta_w = np.clip(delta_w, -0.1, 0.1)
                self.chassis_setvel(vx=delta_z, vy=delta_x * 4, vw=delta_w) # TODO: add the gain to config
                pass
            elif delta_x != 0 or delta_z != 0 or delta_w != 0:
                delta_x = np.clip(delta_x, -0.1, 0.1)
                delta_z = np.clip(delta_z, -0.1, 0.1)
                delta_w = np.clip(delta_w, -0.1, 0.1)
                self.chassis_setvel(vx=delta_z, vy=delta_x * 4, vw=delta_w) # TODO: add the gain to config
                pass
            else:
                # return
                self.chassis_setvel(0, 0, 0)
                arm_y = self.tgt_pose.position.y - config.arm.y_offset
                arm_z = self.tgt_pose.position.z - config.arm.z_offset
                # move to the target
                time.sleep(1)
                self.arm_moveto(arm_z, arm_y)
                time.sleep(2)
                # grab the target
                self.gripper_close()
                time.sleep(1)
                self.chassis_setvel(-0.1, 0, 0)
                time.sleep(1)
                self.chassis_setvel(0, 0, 0)
                self.gripper_open()
                time.sleep(2)
                self.is_done = True
        elif not self.is_done:
            rospy.loginfo_once("marker not found")
        else:
            rospy.loginfo_once("target grabed")
            
    def init_arm(self):
        """init the arm end point position
        """
        self.gripper_open()
        self.arm_moveto(0.1, 0.1)
        time.sleep(2)
        self.arm_moveto(0.18, -0.08)
        time.sleep(1)
        

if __name__ == '__main__':
    rospy.init_node('grab_task')
    rate = rospy.Rate(config.rate)
    time.sleep(1)
    gt = grabTask()
    # gt.gripper_open()
    # gt.arm_moveto(0.1, 0.1)
    # gt.init_end()

    while not rospy.is_shutdown():
        gt.update()
        rate.sleep()
