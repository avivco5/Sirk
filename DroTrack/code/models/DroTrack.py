# ===== models/DroTrack.py =====
# ASCII-only; English comments
# DroTrack main tracker with safer corner handling and fuzzy branch guards.

import time
import math
import numpy as np
import cv2

import utils.bbox_helper as bbox_helper
import models.adaptive_optical_flow as adaptive_optical_flow
import utils.config_helper as config
import utils.cnn_features_extraction as cnn
import models.angular_scaling as angular_scaling
import models.FCM as FCM
from scipy.spatial import distance

def _ensure_gray_u8(img):
    """Ensure frame is single channel uint8."""
    if img is None:
        return None
    if img.ndim == 3 and img.shape[2] == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img.dtype != np.uint8:
        img = cv2.convertScaleAbs(img)
    return img

def _safe_crop(img, bbox):
    """Safe crop with clamping. Returns gray uint8."""
    img = _ensure_gray_u8(img)
    H, W = img.shape[:2]
    x, y, w, h = bbox
    x = max(0, min(int(x), W - 2))
    y = max(0, min(int(y), H - 2))
    w = max(2, min(int(w), W - x))
    h = max(2, min(int(h), H - y))
    roi = img[y:y + h, x:x + w]
    if roi.size == 0:
        # fallback to small center patch
        cx, cy = W // 2, H // 2
        r = max(8, min(W, H) // 16)
        roi = img[max(0, cy - r):min(H, cy + r), max(0, cx - r):min(W, cx + r)]
    return roi

def _to_int_bbox(b):
    return [int(round(b[0])), int(round(b[1])), int(round(b[2])), int(round(b[3]))]

class DroTrack:
    def __init__(self, frame, bbox):
        self.frame = _ensure_gray_u8(frame)
        self.bbox = _to_int_bbox(bbox)
        self.fbbox = list(self.bbox)
        self.iterator = 0

        # mask for drawing (not used by LK directly)
        self.mask = np.zeros_like(self.frame)

        # template and features
        self.template = _safe_crop(self.frame, self.bbox)
        try:
            self.template_vgg_features = cnn.features(self.template, config.VGG16_model, config.preprocess)
        except Exception as e:
            print("[WARN] cnn.features failed, using zeros:", e)
            self.template_vgg_features = np.zeros((512,), dtype=np.float32)

        # LKT initialization
        self.prev_corners = adaptive_optical_flow.LKT_intialization(self.frame, self.template, self.bbox)
        if self.prev_corners is None or len(self.prev_corners) == 0:
            print("[WARN] LKT_intialization: no corners found, using bbox center")
            cx, cy = bbox_helper.get_bbox_center(self.bbox)
            self.prev_corners = np.array([[cx, cy]], dtype=np.float32)
        else:
            self.prev_corners = np.array(self.prev_corners, dtype=np.float32).reshape(-1, 2)

        xs = self.prev_corners[:, 0]
        ys = self.prev_corners[:, 1]
        corners_center = bbox_helper.get_bbox_center([xs.min(), ys.min(), xs.max() - xs.min(), ys.max() - ys.min()])
        center = bbox_helper.get_bbox_center(self.bbox)
        self.complement_x, self.complement_y = bbox_helper.complement_point(corners_center, center)

        self.prev_bbox = list(self.bbox)
        self.prev_frame = self.frame.copy()
        self.prev_of_point = bbox_helper.get_bbox_center(self.bbox)

        self.first_scale = self.prev_bbox[3] / float(max(1, self.frame.shape[0]))

    def track(self, frame):
        self.iterator += 1
        start = time.time()

        frame = _ensure_gray_u8(frame)
        H, W = frame.shape[:2]
        scale = self.prev_bbox[3] / float(max(1, H))

        prev_c = self.prev_corners.reshape(-1, 2).astype(np.float32)

        # Optical flow
        corners, status, errors = adaptive_optical_flow.otpical_flow_LKT(
            self.prev_frame, frame, prev_c.reshape(-1, 1, 2), self.mask, scale
        )

        current_corners = []
        if corners is not None:
            for i in corners:
                x, y = i.ravel()
                current_corners.append([float(x), float(y)])

        # outlier filtering
        ccs = []
        if len(current_corners) > int(50 * scale) and len(prev_c) == len(current_corners):
            distances = []
            for i in range(len(prev_c)):
                px, py = prev_c[i]
                cx, cy = current_corners[i]
                dist = math.hypot(cx - px, cy - py)
                distances.append(dist)
            mu, sigma = np.mean(distances), np.std(distances)
            for i, d in enumerate(distances):
                if mu - 0.75 * sigma <= d <= mu + 0.75 * sigma:
                    ccs.append(current_corners[i])
        if len(ccs) > 0:
            current_corners = ccs

        # center from corners or fallback
        if len(current_corners) == 0:
            of_point_center = bbox_helper.get_bbox_center(self.prev_bbox)
        else:
            xs = np.array([c[0] for c in current_corners], dtype=np.float32)
            ys = np.array([c[1] for c in current_corners], dtype=np.float32)
            of_point_center = bbox_helper.get_bbox_center(
                [float(xs.min()), float(ys.min()), float(xs.max() - xs.min()), float(ys.max() - ys.min())]
            )

        corrected_x = int(of_point_center[0] - self.complement_x * (scale / max(1e-6, self.first_scale)))
        corrected_y = int(of_point_center[1] - self.complement_y * (scale / max(1e-6, self.first_scale)))
        center = (corrected_x, corrected_y)

        # angle based correction
        center, final_angle = angular_scaling.out_of_view_correction(frame, center, self.prev_of_point)
        bboxAng = angular_scaling.Angular_Relative_Scaling(final_angle, self.prev_bbox, center, H)

        bbox = [
            center[0] - (self.prev_bbox[2] / 2.0),
            center[1] - (self.prev_bbox[3] / 2.0),
            bboxAng[2],
            bboxAng[3],
        ]

        # clamp to frame
        bbox = bbox_helper.complement_point((0, 0), (0, 0)) and bbox  # no-op to keep style
        bbox = [float(b) for b in bbox]
        bbox = [
            max(0.0, min(bbox[0], W - 2.0)),
            max(0.0, min(bbox[1], H - 2.0)),
            max(2.0, min(bbox[2], W - bbox[0])),
            max(2.0, min(bbox[3], H - bbox[1])),
        ]

        # fuzzy refinement every few frames
        if self.iterator % 5 == 0:
            try:
                pad_x = int(scale * bbox[2])
                pad_y = int(scale * bbox[3])
                padded = [
                    max(0, int(bbox[0] - pad_x)),
                    max(0, int(bbox[1] - pad_y)),
                    int(min(W - int(bbox[0] - pad_x), int(bbox[2] + 2 * pad_x))),
                    int(min(H - int(bbox[1] - pad_y), int(bbox[3] + 2 * pad_y))),
                ]
                x1, y1, x2, y2 = bbox_helper.get_bbox_points(padded)
                fuzzyArea = frame[y1:y2, x1:x2]
                if fuzzyArea is None or fuzzyArea.size < 36:
                    raise ValueError("empty fuzzyArea")

                # ensure 8-bit single channel
                fuzzyArea = _ensure_gray_u8(fuzzyArea)

                n = 2
                cluster = FCM.FCM(
                    fuzzyArea, n, m=2, epsilon=0.05, max_iter=2 * n, kernel_shape="gaussian", kernel_size=9
                )
                cluster.form_clusters()
                cluster.calculate_scores()

                result = FCM.postFCM(np.float32(cluster.result))

                # choose segment closest to template features
                best = self.best_segment_coord(result, self.template_vgg_features, fuzzyArea, n)
                if best is not None:
                    x0, y0, x1s, y1s = best
                    bbox2 = (
                        int(padded[0] + x0 / 2),
                        int(padded[1] + y0 / 2),
                        int(x0 / 2 + x1s),
                        int(y0 / 2 + y1s),
                    )
                    center = bbox_helper.get_bbox_center(bbox2)
                    bbox3 = np.array(bbox, dtype=np.float32) - np.array(bbox2, dtype=np.float32)
                    bbox = (np.array(bbox, dtype=np.float32) - bbox3 / 10.0).tolist()
                    bbox = FCM.fcm_bbox_correction(bbox, frame)
            except Exception as e:
                print("[WARN] FUZZY correction failed:", e)

        extime = time.time() - start

        # update state
        self.prev_bbox = _to_int_bbox(bbox)
        self.prev_frame = frame.copy()
        if len(current_corners) == 0:
            self.prev_corners = np.array([self.prev_bbox[:2]], dtype=np.float32).reshape(-1, 1, 2)
        else:
            self.prev_corners = np.array(current_corners, dtype=np.float32).reshape(-1, 1, 2)
        self.prev_of_point = of_point_center

        return self.prev_bbox, center, extime

    def best_segment_coord(self, result, tf, fuzzyArea, n):
        """Return (x0, y0, x1, y1) for best segment, or None if not found."""
        best_key = None
        best_rect = None
        for i in range(n):
            mask = (result == i)
            if not np.any(mask):
                continue
            coords = np.argwhere(mask)
            if coords.size == 0:
                continue
            x0, y0 = coords.min(axis=0)
            x1, y1 = coords.max(axis=0) + 1
            if (x1 - x0) < 4 or (y1 - y0) < 4:
                continue
            try:
                sf = cnn.features(fuzzyArea[x0:x1, y0:y1], config.VGG16_model, config.preprocess)
                # guard against empty vectors
                if sf is None or tf is None or len(sf) == 0 or len(tf) == 0:
                    continue
                d = float(distance.cosine(tf, sf))
            except Exception:
                continue
            if best_key is None or d < best_key:
                best_key = d
                best_rect = (x0, y0, x1, y1)
        return best_rect
