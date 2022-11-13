#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import cv2
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

templates = []

def pose_aruco_2_ros(rvec, tvec):
    aruco_pose_msg = Pose()
    aruco_pose_msg.position.x = tvec[0,0]
    aruco_pose_msg.position.y = tvec[1,0]
    aruco_pose_msg.position.z = tvec[2,0]
    rotation_matrix = cv2.Rodrigues(rvec)
    r_quat = R.from_matrix(rotation_matrix[0]).as_quat()
    aruco_pose_msg.orientation.x = r_quat[0]
    aruco_pose_msg.orientation.y = r_quat[1]
    aruco_pose_msg.orientation.z = r_quat[2]
    aruco_pose_msg.orientation.w = r_quat[3]
    return aruco_pose_msg

def map_img77(img):
    segment = [6,14,22,30,37,44,]
    ass = np.split(img, segment, axis=0)
    all_subs = np.array([[np.sum(k)/k.size/255. for k in np.split(a, segment, axis=1)] for a in ass], dtype=np.float32)
    return (all_subs > 0.5).astype(np.uint8)*255

def load_template():
    tpl_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "template")
    rospy.loginfo(tpl_path)
    for i in range(1,9):
        tpl = cv2.imread(tpl_path + "/{}.png".format(i), 0)
        templates.append(map_img77(tpl))
    rospy.loginfo("size of templates: {}".format(len(templates)))

def sort_contour(cnt):

    if not len(cnt) == 4:
        assert False
    new_cnt = cnt.copy()

    cx = (cnt[0,0,0]+cnt[1,0,0]+cnt[2,0,0]+cnt[3,0,0])/4.0
    cy = (cnt[0,0,1]+cnt[1,0,1]+cnt[2,0,1]+cnt[3,0,1])/4.0

    x_left_n = 0
    for i in range(4):
        if cnt[i,0,0] < cx:
            x_left_n += 1
    if x_left_n != 2:
        return None
    lefts = np.array([c for c in cnt if c[0,0] < cx])
    rights = np.array([c for c in cnt if c[0,0] >= cx])
    if lefts[0,0,1] < lefts[1,0,1]:
        new_cnt[0, 0,0] = lefts[0,0,0]
        new_cnt[0, 0,1] = lefts[0,0,1]
        new_cnt[3, 0,0] = lefts[1,0,0]
        new_cnt[3, 0,1] = lefts[1,0,1]
    else:
        new_cnt[0, 0,0] = lefts[1,0,0]
        new_cnt[0, 0,1] = lefts[1,0,1]
        new_cnt[3, 0,0] = lefts[0,0,0]
        new_cnt[3, 0,1] = lefts[0,0,1]

    if rights[0,0,1] < rights[1,0,1]:
        new_cnt[1, 0,0] = rights[0,0,0]
        new_cnt[1, 0,1] = rights[0,0,1]
        new_cnt[2, 0,0] = rights[1,0,0]
        new_cnt[2, 0,1] = rights[1,0,1]
    else:
        new_cnt[1, 0,0] = rights[1,0,0]
        new_cnt[1, 0,1] = rights[1,0,1]
        new_cnt[2, 0,0] = rights[0,0,0]
        new_cnt[2, 0,1] = rights[0,0,1]
    return new_cnt

def preprocessing(frame):
    hsvImg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    boolImg = (np.logical_and(np.logical_and(np.logical_or(hsvImg[:,:,0] <= 10 , hsvImg[:,:,0] >= 150) , hsvImg[:,:,1] >= 60) , hsvImg[:,:,2] >= 75) * 255).astype(np.uint8)
    return boolImg, hsvImg

def square_detection(grayImg, camera_matrix, area_filter_size=30, height_range=(-10000.0,200000.0), block_size = 0.045):
    projection_points = True
    quads = []
    quads_f = []
    contours, hierarchy = cv2.findContours(grayImg,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    try:
        hierarchy = hierarchy[0]
        father_contours = []
        start_search = 0
        while start_search<len(contours):
            if hierarchy[start_search, -1] == -1:# -1 means father idx
                break
            start_search += 1
        saved_idx = [start_search]
        next_same_level_idx = hierarchy[start_search,0] # 0 means next brother
        while next_same_level_idx != -1:
            saved_idx.append(next_same_level_idx)
            next_same_level_idx = hierarchy[next_same_level_idx,0]
        for i in saved_idx:
            father_contours.append(contours[i])
        contours = father_contours
    except:
        print("Nothing detected in hierarchy")
        cv2.imwrite("./debug.png", grayImg)

    if area_filter_size<200:
        filter_len = 4
        projection_points = False
    elif area_filter_size<250:
        filter_len = 5
        projection_points = False
    elif area_filter_size<350:
        filter_len = 10
    else:
        filter_len = 15

    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= area_filter_size:
            approx = cv2.approxPolyDP(contour,filter_len,True)
            if len(approx) == 4:
                approx_sort = sort_contour(approx)
                if approx_sort is not None:
                    quads.append(approx_sort)
                    quads_f.append(approx_sort.astype(float))

    #############
    quads = []
    for contour in contours:
        approx = cv2.approxPolyDP(contour,filter_len,True)
        if len(approx) == 4:
            quads.append(approx_sort)
    #############

    #############
    # half_block_size = 0.05/2.0 # 物块边长的一半，单位是m
    # # objectPoints : 从左上点开始 顺时针排布 ↖ ↗ ↘ ↙
    # # 顺序不唯一,与imagePoints保持一致即可
    # objectPoints = np.array([
    #     (-half_block_size, -half_block_size, 0.0),
    #     (+half_block_size, -half_block_size, 0.0),
    #     (+half_block_size, +half_block_size, 0.0),
    #     (-half_block_size, +half_block_size, 0.0)])
    # dist_coeffs = np.zeros((1,4)) # 设置为零即可
    # for quad in quads_f:
    #     # imagePoints : 需要与objectPoints顺序保持一致
    #     imagePoints = np.array([
    #         (quad[0,0,0],quad[0,0,1]),
    #         (quad[1,0,0],quad[1,0,1]),
    #         (quad[2,0,0],quad[2,0,1]),
    #         (quad[3,0,0],quad[3,0,1])])
    #     ret, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, camera_matrix, dist_coeffs)
    #############

    if projection_points:
        rvec_list = []
        tvec_list = []
        quads_prj = []
        area_list = []
        model_object = np.array([(0-0.5*block_size,0-0.5*block_size, 0.0),
                                (block_size-0.5*block_size, 0-0.5*block_size, 0.0),
                                (block_size-0.5*block_size, block_size-0.5*block_size, 0.0),
                                (0-0.5*block_size, block_size-0.5*block_size, 0.0)])
        dist_coeffs = np.array([[0,0,0,0]], dtype="double")
        for quad in quads_f:
            model_image = np.array([(quad[0,0,0],quad[0,0,1]),
                                    (quad[1,0,0],quad[1,0,1]),
                                    (quad[2,0,0],quad[2,0,1]),
                                    (quad[3,0,0],quad[3,0,1])])
            ret, rvec, tvec = cv2.solvePnP(model_object, model_image, camera_matrix, dist_coeffs)
            projectedPoints,_ = cv2.projectPoints(model_object, rvec, tvec, camera_matrix, dist_coeffs)

            err = 0
            for t in range(len(projectedPoints)):
                err += np.linalg.norm(projectedPoints[t]-model_image[t])

            area = cv2.contourArea(quad.astype(np.int))
            if err/area < 0.005 and tvec[1] > height_range[0] and tvec[1] < height_range[1]:
                quads_prj.append(projectedPoints.astype(int))
                rvec_list.append(rvec)
                tvec_list.append(tvec)
                area_list.append(area)
        return quads_prj, tvec_list, rvec_list, area_list, quads
    else:
        return quads, [[0,0,0] for _ in quads], [[0,0,0] for _ in quads], [cv2.contourArea(quad.astype(np.int)) for quad in quads], quads

def classification(frame, quads,template_ids=range(1,9)):
    quads_ID = []
    minpoints_list = []
    wrapped_img_list = []
    for i in range(len(quads)):
        points_src = np.array([[(quads[i][0,0,0],quads[i][0,0,1])],
                                [(quads[i][1,0,0],quads[i][1,0,1])],
                                [(quads[i][2,0,0],quads[i][2,0,1])],
                                [(quads[i][3,0,0],quads[i][3,0,1])]],dtype = "float32")
        points_dst = np.array([[0,0],
                                [49,0],
                                [49,49],
                                [0,49]],dtype = "float32")
        out_img = cv2.warpPerspective(frame, cv2.getPerspectiveTransform(points_src,points_dst), (50,50))
        out_img = cv2.cvtColor(out_img, cv2.COLOR_BGR2GRAY)
        out_img = cv2.threshold(out_img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        wrapped_img_list.append(out_img)
        
        resize = False
        if resize:
            try:
                # resize trick from XL, adjust outimg just for classification
                out_img[:3, :] = 0
                out_img[47:, :] = 0
                out_img[:, :3] = 0
                out_img[:,47:] = 0
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(out_img)
                for label_i in range(1, num_labels):                
                    if stats[label_i, cv2.CC_STAT_AREA].astype(float) < 35: # 原50
                        out_img[labels==label_i] = 0
            
                nonzero_img = np.nonzero(out_img)
                left, right = np.min(nonzero_img[0]), np.max(nonzero_img[0])
                top, bottom = np.min(nonzero_img[1]), np.max(nonzero_img[1])
                right, bottom = min(right+1, 49), min(bottom+1, 49)
                nonzero_img = out_img[left:right, top:bottom]
                nonzero_img = cv2.resize(nonzero_img, (36, 36), interpolation=cv2.INTER_NEAREST)
                out_img = np.zeros((50,50),dtype=np.uint8)
                out_img[7:7+36, 7:7+36] = nonzero_img
            except:
                print("resize trick failed, back to original img as tempate")
        out_img = map_img77(out_img)


        match_candidate = []
        match_candidate.append(out_img)
        match_candidate.append(cv2.rotate(out_img, cv2.ROTATE_180))
        match_candidate.append(cv2.rotate(out_img, cv2.ROTATE_90_CLOCKWISE))
        match_candidate.append(cv2.rotate(out_img, cv2.ROTATE_90_COUNTERCLOCKWISE))

        min_diff = 10000
        min_diff_target = 0

        for tid in template_ids:
            for tt in range(4):
                diff_img = cv2.absdiff(templates[tid - 1], match_candidate[tt])
                sum = np.sum(diff_img) / 255.0 / diff_img.size
                if min_diff > sum:
                    min_diff = sum
                    min_diff_target = tid

        if min_diff < 0.2:
            quads_ID.append(min_diff_target)
            minpoints_list.append(min_diff)
        else:
            quads_ID.append(-1)
            minpoints_list.append(min_diff)

    return quads_ID, minpoints_list, wrapped_img_list


def marker_detection(frame, template_ids=range(1,9), area_filter_size=30, height_range=(-10000.0,200000.0), camera_matrix=np.array([(617.3054000792732, 0.0, 424.0),
                                                                                                                                 (0.0, 608.3911743164062, 243.64712524414062,),
                                                                                                                                 (0,0,1)], dtype="double")):
    boolImg, _ = preprocessing(frame)
    quads, tvec_list,rvec_list, area_list, ori_quads = square_detection(boolImg, camera_matrix, area_filter_size=area_filter_size, height_range=height_range)
    quads_ID, minpoints_list, wrapped_img_list = classification(frame, quads, template_ids=template_ids)
    ids = [i for i in range(len(quads_ID)) if 1 <= quads_ID[i] <= 5]
    pose_list = [pose_aruco_2_ros(r, t) for t,r in zip(tvec_list, rvec_list)]

    return [quads_ID[_] for _ in ids], [quads[_] for _ in ids], [pose_list[_] for _ in ids]
