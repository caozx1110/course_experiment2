#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../../config"))
from config import config

import cv2
import rospy
import traceback
import numpy as np
from threading import Thread
from cv_bridge import CvBridge

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image, CameraInfo
from detect import marker_detection, load_template
from marker_location.msg import markers

class Processor:
    def __init__(self, verbose=True):
        self.verbose = verbose
        self.bridge = CvBridge()
        self.image = None
        self.current_visualization_image = None

        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message(config.camera.img_topic_name, Image, timeout=5.0)
                rospy.loginfo("Get topic /ep/camera/color/image_raw.")
            except KeyboardInterrupt:
                traceback.print_exc()
                quit()
            except:
                rospy.logwarn("Waiting for message /ep/camera/color/image_raw.")
                continue
            try:
                rospy.wait_for_message(config.camera.info_topic_name, CameraInfo, timeout=5.0)
                rospy.loginfo("Get topic /ep/camera/color/camera_info.")
            except KeyboardInterrupt:
                traceback.print_exc()
                quit()
            except:
                rospy.logwarn("Waiting for message /ep/camera/color/image_raw.")
                continue
            break

        camera_info = rospy.wait_for_message(config.camera.info_topic_name, CameraInfo, timeout=1.0)
        self.camera_matrix = np.array(camera_info.K).reshape(3,3)

        try:
            if self.verbose:
                self.vis_thread = Thread(target=self.visualization)
                self.vis_thread.start()
        except:
            rospy.logerr("The visualization window has collapsed!")

        rospy.Subscriber(config.camera.img_topic_name, Image, self.imageCallback, queue_size=1)
        self.pub = rospy.Publisher(config.marker.topic_name, markers, queue_size=1)

    def imageCallback(self, image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(image, "bgr8")

            id_list, quads_list, pose_list = marker_detection(self.image, area_filter_size=1200, camera_matrix=self.camera_matrix)
            marker_msg = markers()
            marker_msg.header.seq = image.header.seq
            marker_msg.header.stamp = image.header.stamp
            marker_msg.header.frame_id = image.header.frame_id
            msg_id_lst = []
            for i in range(len(id_list)):
                msg_id_lst.append(id_list[i])
                marker_msg.poses.append(pose_list[i])
            marker_msg.ids = UInt8MultiArray(data=msg_id_lst)
            self.pub.publish(marker_msg)

            if self.verbose:
                cv2.drawContours(self.image, quads_list, -1, (0,255,0), 1)
                self.current_visualization_image = self.image
            id_cnd = [0] * 5
            color_lst = [(255, 0, 255), (0, 255, 255), (255, 255, 0), (0, 0, 255), (0, 255, 0), (255, 0, 0)]
            for i in range(len(id_list)):
                bbox = cv2.boundingRect(quads_list[i])
                cv2.putText(self.image, '{}'.format(id_list[i]), (bbox[0], (-20*id_cnd[id_list[i]-1]-20)+bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_lst[id_cnd[id_list[i]-1]], 2)
                cv2.putText(self.image, '({:.1f},{:.1f},{:.1f})'.format(pose_list[i].position.x*100, pose_list[i].position.y*100, pose_list[i].position.z*100), (bbox[0], -20*id_cnd[id_list[i]-1]+bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color_lst[id_cnd[id_list[i]-1]], 2)
                id_cnd[id_list[i]-1] += 1
                if id_cnd[id_list[i]-1] > 5:
                    id_cnd[id_list[i]-1] = 5
        except Exception as e:
            rospy.logerr_once(e)

    def visualization(self):
        while not rospy.is_shutdown():
            try:
                if self.current_visualization_image is not None:
                    cv2.imshow('frame', self.current_visualization_image)
                    cv2.waitKey(1)
            except:
                rospy.logerr("The visualization window has collapsed!")

if __name__ == "__main__":
    rospy.init_node('image_node', anonymous=True)
    load_template()

    pcer = Processor(verbose=True)
    rospy.loginfo("Image thread started")
    rospy.spin()
