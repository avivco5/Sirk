# ===== run.py =====
# ASCII-only; English comments
# Robust runner for DTB70 sequences using drone-tracking "experiments/anno"
# Loads real frames if present; otherwise skips the sequence.
# Requires:
#   - anno txt files at  datasets/experiments/anno/*.txt  (from flyers/drone-tracking)
#   - images at one of these roots (override with env var DTB70_IMG_ROOT):
#       datasets/data/DTB70/<Seq>/img/*.jpg
#       datasets/DTB70/<Seq>/img/*.jpg
#       drone-tracking/data/DTB70/<Seq>/img/*.jpg
# Produces tracking results via utils.results_saving if available.

import os
import glob
import time
import math
from collections import OrderedDict

import cv2
import numpy as np

# project imports
import utils.config_helper as config
import utils.bbox_helper as bbox_helper
import models.DroTrack as DroTrack

# optional saver (do not crash if module is missing)
try:
    import utils.results_saving as results_saving
    HAS_SAVER = True
except Exception:
    HAS_SAVER = False

# -----------------------------------------------------------------------------
# paths
# -----------------------------------------------------------------------------
HERE = os.path.abspath(os.path.dirname(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
ANNO_ROOT = os.path.abspath(os.path.join(ROOT, "datasets", "experiments", "anno"))

IMG_ROOT_CANDIDATES = [
    os.environ.get("DTB70_IMG_ROOT", "").strip(),
    os.path.abspath(os.path.join(ROOT, "datasets", "data", "DTB70")),
    os.path.abspath(os.path.join(ROOT, "datasets", "DTB70")),
    os.path.abspath(os.path.join(ROOT, "drone-tracking", "data", "DTB70")),
]

IMG_ROOT_CANDIDATES = [p for p in IMG_ROOT_CANDIDATES if p and os.path.isdir(p)]

if not os.path.isdir(ANNO_ROOT):
    raise FileNotFoundError("Missing anno root: %s" % ANNO_ROOT)

# -----------------------------------------------------------------------------
# helpers
# -----------------------------------------------------------------------------
def read_anno_txt(txt_path):
    """Return list of [x,y,w,h] per frame. Supports comma or tab separators."""
    boxes = []
    with open(txt_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            sep = "," if "," in line else "\t"
            parts = [p.strip() for p in line.split(sep)]
            vals = []
            for p in parts[:4]:
                try:
                    vals.append(float(p))
                except Exception:
                    vals.append(float("nan"))
            boxes.append(vals)
    return boxes

def find_seq_img_dir(seq_name):
    """Try common locations for <root>/<seq>/img."""
    for base in IMG_ROOT_CANDIDATES:
        cand = os.path.join(base, seq_name, "img")
        if os.path.isdir(cand):
            return cand
    return None

def detect_zero_padding(img_dir):
    """Infer zero padding from file names like 0001.jpg or 00001.jpg."""
    files = sorted(glob.glob(os.path.join(img_dir, "*.jpg")))
    if not files:
        return 4
    name = os.path.splitext(os.path.basename(files[0]))[0]
    # count leading zeros
    zc = len(name)
    # clamp to [4,6]
    return max(4, min(zc, 6))

def clamp_bbox_to_frame(b, H, W):
    """Clamp bbox [x,y,w,h] to image bounds and force min size 2x2."""
    x, y, w, h = b
    x = max(0, min(x, W - 2))
    y = max(0, min(y, H - 2))
    w = max(2, min(w, W - x))
    h = max(2, min(h, H - y))
    return [float(x), float(y), float(w), float(h)]

def draw_bbox(img, bbox, color=(0, 255, 255), thickness=2):
    x1, y1, x2, y2 = bbox_helper.get_bbox_points(bbox)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

# -----------------------------------------------------------------------------
# main
# -----------------------------------------------------------------------------
def main():
    # build list of sequences from anno txt files (skip "att" folder)
    txt_files = sorted(glob.glob(os.path.join(ANNO_ROOT, "*.txt")))
    if not txt_files:
        print("[WARN] No anno txt files under:", ANNO_ROOT)
        return

    # iterate sequences
    for txt_path in txt_files:
        seq_name = os.path.splitext(os.path.basename(txt_path))[0]
        img_dir = find_seq_img_dir(seq_name)

        if img_dir is None:
            print("[WARN] Skip %s: images not found. Set DTB70_IMG_ROOT or place frames under datasets/data/DTB70/%s/img" % (seq_name, seq_name))
            continue

        # frames and anno
        zc = detect_zero_padding(img_dir)
        gt_boxes = read_anno_txt(txt_path)
        n_frames = len(gt_boxes)

        if n_frames == 0:
            print("[WARN] Skip %s: empty anno." % seq_name)
            continue

        print("Sequence:", seq_name)

        # first frame for init
        first_path = os.path.join(img_dir, ("{0:0" + str(zc) + "}").format(1) + ".jpg")
        frame0 = cv2.imread(first_path, cv2.IMREAD_GRAYSCALE)
        if frame0 is None:
            print("[WARN] Skip %s: cannot read %s" % (seq_name, first_path))
            continue

        H, W = frame0.shape[:2]
        fbbox = clamp_bbox_to_frame(gt_boxes[0], H, W)

        # create tracker
        tracker = DroTrack.DroTrack(frame0, [int(fbbox[0]), int(fbbox[1]), int(fbbox[2]), int(fbbox[3])])

        # iterate frames
        for i in range(1, n_frames + 1):
            img_path = os.path.join(img_dir, ("{0:0" + str(zc) + "}").format(i) + ".jpg")
            frame = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if frame is None:
                # try next
                continue

            try:
                bbox, center, extime = tracker.track(frame)
            except Exception as e:
                # recover by re-initializing on current gt if available
                print("[WARN] Tracking error at %s #%d: %s" % (seq_name, i, str(e)))
                if i - 1 < len(gt_boxes):
                    cur_gt = clamp_bbox_to_frame(gt_boxes[i - 1], frame.shape[0], frame.shape[1])
                    tracker = DroTrack.DroTrack(frame, [int(cur_gt[0]), int(cur_gt[1]), int(cur_gt[2]), int(cur_gt[3])])
                    bbox = cur_gt
                    center = bbox_helper.get_bbox_center(bbox)
                    extime = 0.0
                else:
                    continue

            # visualization or saving
            if getattr(config, "save", 0) == 0:
                vis = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                draw_bbox(vis, bbox, (0, 255, 255), 2)
                cv2.imshow(seq_name, vis)
                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break
            else:
                if HAS_SAVER:
                    gt = gt_boxes[i - 1] if (i - 1) < len(gt_boxes) else [0, 0, 0, 0]
                    results_saving.save_tracking_results("DTB70", seq_name, os.path.basename(img_path), bbox, gt, 1, extime)

        cv2.destroyWindow(seq_name)

if __name__ == "__main__":
    main()
