# -*- coding: utf-8 -*-
"""
BBox helper functions
Supports both classic datasets with groundtruth_rect.txt
and flat .txt annotation files (e.g. Animal1.txt)
"""

from collections import OrderedDict
import cv2
import numpy as np
import os


def intersection_over_union(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou


def data2bboxes(data):
    boxes = {}
    for title, dataset in data.items():
        boxes.update({title: {}})
        for class_item in dataset['dirs']:
            filepath = os.path.join(dataset['url'], class_item, "groundtruth_rect.txt")
            if not os.path.exists(filepath):
                filepath = os.path.join(dataset['url'], class_item + ".txt")
            if not os.path.exists(filepath):
                print("[WARN] Missing:", filepath)
                continue

            with open(filepath, "r") as f:
                first_line = f.readline().strip()

            points = []
            try:
                for point in first_line.split(','):
                    points.append(int(float(point)))
            except:
                for point in first_line.replace('\t', ' ').split():
                    try:
                        points.append(int(float(point)))
                    except:
                        continue

            if len(points) >= 4:
                bbox = tuple(points[:4])
                boxes[title][class_item] = bbox
            else:
                print("[WARN] Invalid bbox in:", filepath)
    return OrderedDict(sorted(boxes.items()))


def get_all_bboxes(filename):
    all_boxes = []
    if not os.path.exists(filename):
        print("[WARN] Missing:", filename)
        return all_boxes
    with open(filename, "r") as f:
        lines = f.readlines()
    for line in lines:
        box = []
        for point in line.replace('\t', ',').split(','):
            try:
                box.append(float(point))
            except:
                box.append(np.nan)
        if len(box) >= 4:
            all_boxes.append(box[:4])
    return all_boxes


def get_bbox_points(bbox):
    y1 = int(bbox[1])
    y2 = int(bbox[1]) + int(bbox[3])
    x1 = int(bbox[0])
    x2 = int(bbox[0]) + int(bbox[2])
    return x1, y1, x2, y2


def visualise_bbox(bbox, file, class_dir, s, dataset):
    filepath = os.path.join(dataset, class_dir, "img", file)
    frame_color = cv2.imread(filepath)
    if frame_color is None:
        print("[WARN] Could not read image:", filepath)
        return
    x1, y1, x2, y2 = get_bbox_points(bbox)
    x1, y1, x2, y2 = int(x1 * s), int(y1 * s), int(x2 * s), int(y2 * s)
    cv2.rectangle(frame_color, (x1, y1), (x2, y2), (0, 255, 255), 1)
    cv2.imshow(class_dir, frame_color)


def get_bbox_center(bbox):
    bbox_center_x = int(bbox[0] + (bbox[2] / 2))
    bbox_center_y = int(bbox[1] + (bbox[3] / 2))
    return (bbox_center_x, bbox_center_y)


def complement_point(point, ppoint):
    return point[0] - ppoint[0], point[1] - ppoint[1]
