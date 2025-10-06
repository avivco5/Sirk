#!/usr/bin/env python3
# ascii-only, english comments
# YOLOv8 + DeepSORT (Kalman + ReID) with multi-class filter by IDs or names
# Features:
# - --classes "0,2,3,5,7" (IDs) OR --classnames "person,car,cellphone"
# - Aliases: "cellphone|mobile|phone" -> "cell phone" (id=67), etc.
# - --list-classes prints all model classes (id -> name) and exits
# - Viewer: --show (OpenCV) OR --web HOST:PORT (MJPEG)
# - Stable colored boxes per (track_id, class_id)

import argparse, time, sys, threading, queue
import re
import cv2
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort

# ---------------- MJPEG web server (optional) ----------------
class MJPEGServer:
    def __init__(self, host="0.0.0.0", port=5000):
        self.host, self.port = host, port
        self.frame_q = queue.Queue(maxsize=2)
        self._srv_thread = None

    def start(self):
        from flask import Flask, Response
        app = Flask(__name__)

        @app.route("/")
        def index():
            return "<html><body><h3>YOLOv8 + DeepSORT stream</h3><img src='/video' /></body></html>"

        def gen():
            while True:
                jpg = self.frame_q.get()
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")

        @app.route("/video")
        def video():
            return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

        def run():
            app.run(host=self.host, port=self.port, debug=False, threaded=True, use_reloader=False)

        self._srv_thread = threading.Thread(target=run, daemon=True)
        self._srv_thread.start()
        print(f"[WEB] MJPEG on http://{self.host}:{self.port}")

    def push(self, bgr):
        ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        if self.frame_q.full():
            try: self.frame_q.get_nowait()
            except: pass
        self.frame_q.put(jpg.tobytes())

# ---------------- utils ----------------
def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=str, default="0", help="camera index or video path")
    ap.add_argument("--model", type=str, default="yolov8n.pt")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--classes", type=str, default="", help="IDs: '0,2,3,5,7'")
    ap.add_argument("--classnames", type=str, default="", help="names: 'person,car,cellphone'")
    ap.add_argument("--list-classes", action="store_true", help="print model classes and exit")
    ap.add_argument("--show", action="store_true", help="OpenCV window viewer")
    ap.add_argument("--web", type=str, default="", help="Start MJPEG server, e.g. '0.0.0.0:5000'")
    ap.add_argument("--save", type=str, default="", help="mp4 output path")
    ap.add_argument("--max-age", type=int, default=30)
    ap.add_argument("--n-init", type=int, default=3)
    ap.add_argument("--max-iou", type=float, default=0.7)
    ap.add_argument("--nn-budget", type=int, default=100)
    return ap.parse_args()

def get_names_map(model_or_res):
    # Returns {id:int -> name:str}
    names = None
    try:
        names = model_or_res.names if hasattr(model_or_res, "names") else None
    except:
        names = None
    if names is None:
        return None
    if isinstance(names, dict):
        return names
    # sometimes it's a list-like
    try:
        return {i: str(n) for i, n in enumerate(names)}
    except:
        return None

def norm(s):
    # normalize a class name: lowercase + remove non-alnum
    return re.sub(r"[^a-z0-9]", "", str(s).lower())

def build_alias_map(id2name):
    # Build alias->id map for robust name matching (spaces/underscores/hyphens removed)
    # Also inject common synonyms for COCO-style names
    alias_to_id = {}
    if not id2name:
        return alias_to_id

    # base: canonical names and no-space variants
    for i, nm in id2name.items():
        alias_to_id[norm(nm)] = i

    # add common synonyms
    syn = {
        "tv": "tv",
        "television": "tv",
        "tvmonitor": "tv",
        "aeroplane": "airplane",
        "aircraft": "airplane",
        "plane": "airplane",
        "motorbike": "motorcycle",
        "stoplight": "traffic light",
        "stopsign": "stop sign",
        "diningtable": "dining table",
        "sportsball": "sports ball",
        "pottedplant": "potted plant",
        "hairdryer": "hair drier",
        "fridge": "refrigerator",
        "cellphone": "cell phone",
        "mobile": "cell phone",
        "mobilephone": "cell phone",
        "phone": "cell phone",
        "smartphone": "cell phone",
        "handbag": "handbag",  # keep but normalize variants
        "backpack": "backpack",
        "remotecontrol": "remote",
        "firehydrant": "fire hydrant",
        "trafficlight": "traffic light",
        "parkingmeter": "parking meter",
        "tennisracket": "tennis racket",
        "wineglass": "wine glass",
        "hotdog": "hot dog",
        "hairdrier": "hair drier",
        "toothbrush": "toothbrush",
        "toothbrushes": "toothbrush",
        "sofa": "couch",
        "laptop": "laptop",
        "keyboard": "keyboard",
        "mouse": "mouse",
    }
    for alias, canon in syn.items():
        alias_n = norm(alias)
        canon_n = norm(canon)
        if canon_n in alias_to_id:
            alias_to_id[alias_n] = alias_to_id[canon_n]

    return alias_to_id

def resolve_class_filter(args, id2name):
    # Build a set of class IDs from --classes and/or --classnames
    ids = set()
    any_filter = False

    # IDs
    if args.classes.strip():
        any_filter = True
        for s in args.classes.split(","):
            s = s.strip()
            if not s: continue
            try:
                ids.add(int(s))
            except:
                print(f"[WARN] bad class id '{s}'")

    # Names with aliases
    if args.classnames.strip():
        any_filter = True
        alias_map = build_alias_map(id2name or {})
        for s in args.classnames.split(","):
            raw = s.strip()
            if not raw: continue
            key = norm(raw)
            if key in alias_map:
                ids.add(alias_map[key])
            else:
                print(f"[WARN] class name '{raw}' not found in model names")
    return (ids if any_filter else None)

def xyxy_to_ltrb_wh(xyxy):
    x1, y1, x2, y2 = xyxy
    w = x2 - x1
    h = y2 - y1
    return [float(x1), float(y1), float(w), float(h)]

def open_source(src_str):
    src = 0 if src_str.isdigit() else src_str
    cap = cv2.VideoCapture(int(src) if isinstance(src, int) else src)
    if not cap.isOpened():
        print("ERROR: cannot open source:", src_str)
        sys.exit(1)
    return cap

def color_for(track_id, cls_id):
    k = int(track_id) * 3 + int(cls_id or 0) * 13
    r = (k >> 16) & 255
    g = (k >> 8) & 255
    b = k & 255
    return (b, g, r)

def draw_legend(frame, names, cls_filter):
    txt = "ALL classes" if cls_filter is None else "Filter: " + ", ".join(
        [names.get(i, str(i)) if names else str(i) for i in sorted(cls_filter)]
    )
    cv2.putText(frame, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (10,10,10), 3, cv2.LINE_AA)
    cv2.putText(frame, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240,240,240), 1, cv2.LINE_AA)

# ---------------- main ----------------
def main():
    args = parse_args()

    cap = open_source(args.source)

    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        writer = cv2.VideoWriter(args.save, fourcc, fps, (w, h))

    model = YOLO(args.model)
    id2name = get_names_map(model) or {}

    if args.list_classes:
        print("Model classes:")
        for i in sorted(id2name.keys()):
            print(f"{i:2d}: {id2name[i]}")
        return

    cls_filter = resolve_class_filter(args, id2name)

    tracker = DeepSort(
        max_age=args.max_age,
        n_init=args.n_init,
        max_iou_distance=args.max_iou,
        nn_budget=args.nn_budget,
        nms_max_overlap=1.0,
        embedder="mobilenet",
        half=True
    )

    # viewers
    use_imshow = args.show
    web_srv = None
    if args.web:
        host, port = args.web.split(":")
        web_srv = MJPEGServer(host, int(port))
        web_srv.start()
    if use_imshow:
        try:
            cv2.namedWindow("YOLOv8 + DeepSORT", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("YOLOv8 + DeepSORT", 960, 540)
            print("[SHOW] Press 'q' to quit.")
        except Exception as e:
            print("[SHOW] OpenCV window failed:", e)
            use_imshow = False

    last_t = time.time()
    while True:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok:
            break
        t_cap = (time.time() - t0) * 1000.0

        t1 = time.time()
        res = model.predict(source=frame, imgsz=args.imgsz, conf=args.conf, verbose=False)
        det = res[0]
        # refresh names (some versions set per-result)
        id2name = get_names_map(det) or id2name
        t_infer = (time.time() - t1) * 1000.0

        detections = []
        if det.boxes is not None and det.boxes.xyxy is not None:
            xyxy = det.boxes.xyxy.cpu().numpy()
            confs = det.boxes.conf.cpu().numpy()
            clss  = det.boxes.cls.cpu().numpy().astype(int)
            for i in range(len(xyxy)):
                if cls_filter is not None and clss[i] not in cls_filter:
                    continue
                ltrbwh = xyxy_to_ltrb_wh(xyxy[i])
                detections.append([ltrbwh, float(confs[i]), int(clss[i])])

        t2 = time.time()
        tracks = tracker.update_tracks(detections, frame=frame)
        t_track = (time.time() - t2) * 1000.0

        for trk in tracks:
            if not trk.is_confirmed():
                continue
            l, t, r, b = trk.to_ltrb()
            l, t, r, b = int(l), int(t), int(r), int(b)
            tid = trk.track_id
            cls_id = trk.get_det_class()
            col = color_for(tid, cls_id)
            nm = id2name.get(cls_id, str(cls_id))
            label = f"{nm} ID {tid}"
            cv2.rectangle(frame, (l, t), (r, b), col, 2)
            cv2.putText(frame, label, (l, max(0, t-6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)

        draw_legend(frame, id2name, cls_filter)

        t_loop = (time.time() - t0) * 1000.0
        if (time.time() - last_t) > 1.0:
            print(f"[FPS] loop={1000.0/t_loop:.1f}/s cap={t_cap:.1f}ms infer={t_infer:.1f}ms track={t_track:.1f}ms")
            last_t = time.time()

        if use_imshow:
            try:
                cv2.imshow("YOLOv8 + DeepSORT", frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
            except Exception as e:
                print("[SHOW] imshow error, switching to web viewer:", e)
                use_imshow = False
                if web_srv is None:
                    web_srv = MJPEGServer("0.0.0.0", 5000)
                    web_srv.start()

        if web_srv is not None:
            web_srv.push(frame)

        if writer is not None:
            writer.write(frame)

    cap.release()
    if writer: writer.release()
    try: cv2.destroyAllWindows()
    except: pass

if __name__ == "__main__":
    main()
