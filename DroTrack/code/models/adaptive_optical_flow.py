# -*- coding: utf-8 -*-
"""
Adaptive Optical Flow (LKT) with safe initialization
Modified: fallback to bbox center if no corners are found
"""

import cv2
import utils.bbox_helper as bbox_helper
import numpy as np


def find_Shi_Tomasi_corners(first_frame, maxCorners=100, qualityLevel=0.2,
                            minDistance=7, blockSize=7):
    feature_params = dict(maxCorners=maxCorners,
                          qualityLevel=qualityLevel,
                          minDistance=minDistance,
                          blockSize=blockSize)
    corners = cv2.goodFeaturesToTrack(first_frame, mask=None, **feature_params)
    return corners


def LKT_intialization(first_frame, template, bbox,
                      maxCorners=100,
                      qualityLevel=0.5,
                      minDistance=1,
                      blockSize=3,
                      padding=0):

    qualityLevel = max(qualityLevel, 0.005)

    first_frame_corners = find_Shi_Tomasi_corners(
        first_frame,
        maxCorners=maxCorners,
        qualityLevel=qualityLevel,
        minDistance=minDistance,
        blockSize=blockSize
    )

    if first_frame_corners is None:
        print("[WARN] LKT_intialization: no corners found, using bbox center")
        cx, cy = bbox_helper.get_bbox_center(bbox)
        return np.array([[[cx, cy]]], dtype=np.float32)

    corners = []
    for i in first_frame_corners:
        x, y = i.ravel()
        corners.append(list(i.ravel()))

    points = []
    x1, y1, x2, y2 = bbox_helper.get_bbox_points(bbox)

    for i in range(len(corners)):
        p_x, p_y = corners[i][0], corners[i][1]
        if (p_x >= x1 and p_x < x2) and (p_y >= y1 and p_y < y2):
            points.append(corners[i])

    if len(points) > 0:
        return np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    else:
        # recursive retry with relaxed parameters
        maxCorners += 10
        qualityLevel -= 0.05
        minDistance += 1
        blockSize += 1
        padding += 1
        if qualityLevel <= 0:
            cx, cy = bbox_helper.get_bbox_center(bbox)
            return np.array([[[cx, cy]]], dtype=np.float32)
        return LKT_intialization(first_frame, template, bbox,
                                 maxCorners,
                                 qualityLevel,
                                 minDistance,
                                 blockSize,
                                 padding)


def otpical_flow_LKT(prev_frame, frame, prev_corners, mask, scale):
    s = int(100 * scale)
    if s < 15:
        s = 15
    if s > 30:
        s = 30

    m = int(s / 2) * 10

    lucas_kanade_params = dict(
        winSize=(s, s),
        maxLevel=10,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, m, m / 100)
    )

    current_corners, status, errors = cv2.calcOpticalFlowPyrLK(
        prev_frame, frame, prev_corners, None, **lucas_kanade_params)

    return current_corners, status, errors
