#!/usr/bin/env python3
# Fast remote YOLO guard: threaded pipeline (grab -> infer -> serve),
# downscale for inference, MJPEG live annotated stream, optional MP4, image logs.
# ASCII-only comments

import os, time, csv, threading, queue
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np
from PIL import Image
from ultralytics import YOLO
from flask import Flask, Response

# Optional .env
try:
    from dotenv import load_dotenv
    load_dotenv()
except Exception:
    pass

# ---------------- CONFIG ----------------
STREAM_URL   = os.getenv("STREAM_URL", "http://10.0.0.25:8000/stream")

# Project logs under your repo:
PROJECT_ROOT = Path(r"C:\Users\Aviv\PycharmProjects\81\PTZ")
LOG_ROOT     = PROJECT_ROOT / "security_logs"
SAVE_DIR     = LOG_ROOT / "remote_images"
CSV_PATH     = LOG_ROOT / "remote_detections.csv"

# YOLO
YOLO_MODEL   = os.getenv("YOLO_MODEL", "yolov8n.pt")
CONF         = float(os.getenv("CONF", "0.40"))
IOU          = float(os.getenv("IOU", "0.45"))
INFER_SIZE   = int(os.getenv("INFER_SIZE", "512"))  # 320/416/512/640; lower = faster

# Performance knobs
TARGET_FPS   = float(os.getenv("TARGET_FPS", "12"))  # cap total pipeline FPS
WRITE_MP4    = os.getenv("WRITE_MP4", "0") == "1"    # optional recording
RESIZE_OUTPUT_W = int(os.getenv("FRAME_WIDTH", "0")) # 0 = keep source
RESIZE_OUTPUT_H = int(os.getenv("FRAME_HEIGHT", "0"))

# Save throttling
SAVE_COOLDOWN_SEC = int(os.getenv("SAVE_COOLDOWN_SEC", "45"))
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "80"))

# Live annotated MJPEG server (local)
MJPEG_HOST   = os.getenv("MJPEG_HOST", "0.0.0.0")
MJPEG_PORT   = int(os.getenv("MJPEG_PORT", "8088"))
MJPEG_PATH   = os.getenv("MJPEG_PATH", "/stream")

SHOW_WINDOW  = os.getenv("SHOW_WINDOW", "0") == "1"
# ----------------------------------------

# ---- prep paths ----
SAVE_DIR.mkdir(parents=True, exist_ok=True)
CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
if not CSV_PATH.exists():
    with open(CSV_PATH, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "score", "image_path"])

def normalize_stream_url(url: str) -> str:
    u = url.rstrip("/")
    if u.endswith(":8000"):
        return u + "/stream"
    if u.count("/") == 2:  # scheme://host
        return u + "/stream"
    return url
STREAM_URL = normalize_stream_url(STREAM_URL)

print("[INFO] Source:", STREAM_URL)
print("[INFO] Loading YOLO:", YOLO_MODEL)
model = YOLO(YOLO_MODEL)

# ---- Open capture (prefer FFMPEG) ----
cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    cap = cv2.VideoCapture(STREAM_URL)
if not cap.isOpened():
    raise SystemExit(f"Failed to open {STREAM_URL}. Try opening /frame in browser.")

ok, f0 = cap.read()
if not ok:
    raise SystemExit("Stream opened but first frame failed.")
src_h, src_w = f0.shape[:2]
out_w = RESIZE_OUTPUT_W if RESIZE_OUTPUT_W > 0 else src_w
out_h = RESIZE_OUTPUT_H if RESIZE_OUTPUT_H > 0 else src_h
print(f"[INFO] Frame size: src={src_w}x{src_h}, out={out_w}x{out_h}, infer_size={INFER_SIZE}")

# ---- Optional MP4 writer ----
OUT_MP4 = LOG_ROOT / "remote_labeled_fast.mp4"
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
video_writer = cv2.VideoWriter(str(OUT_MP4), fourcc, TARGET_FPS, (out_w, out_h)) if WRITE_MP4 else None

# ---- Shared state for MJPEG server ----
app = Flask(__name__)
_latest_jpeg = None
_jpeg_lock = threading.Lock()

@app.route("/")
def root():
    return f"Annotated MJPEG at {MJPEG_PATH}"

def gen_mjpeg():
    boundary = b"--frame"
    while True:
        with _jpeg_lock:
            data = _latest_jpeg
        if data is not None:
            yield (boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")
        time.sleep(1.0 / max(TARGET_FPS, 1))

@app.route(MJPEG_PATH)
def stream():
    return Response(gen_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

def serve():
    app.run(host=MJPEG_HOST, port=MJPEG_PORT, threaded=True)

threading.Thread(target=serve, daemon=True).start()
print(f"[INFO] Live annotated stream: http://127.0.0.1:{MJPEG_PORT}{MJPEG_PATH}")

# ---- Queues: latest-frame-wins (drop old) ----
grab_q   = queue.Queue(maxsize=1)  # raw frames from capture
infer_q  = queue.Queue(maxsize=1)  # results (annotated frame + best score + person_found)

stop_flag = threading.Event()

def put_latest(q: queue.Queue, item):
    # Drop old item if queue is full, then put
    if q.full():
        try:
            q.get_nowait()
        except queue.Empty:
            pass
    q.put_nowait(item)

def grab_thread():
    # grab frames and push to queue
    while not stop_flag.is_set():
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue
        if (out_w, out_h) != (src_w, src_h):
            frame = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_LINEAR)
        put_latest(grab_q, frame)

def infer_thread():
    # consume latest raw frame, run yolo on downscaled copy, draw on full frame
    last_save = 0.0
    while not stop_flag.is_set():
        try:
            frame = grab_q.get(timeout=0.2)
        except queue.Empty:
            continue

        # downscale for inference
        ih, iw = frame.shape[:2]
        scale_w = INFER_SIZE
        # keep aspect
        scale_h = int(ih * (scale_w / float(iw))) if iw != 0 else INFER_SIZE
        infer_img = cv2.resize(frame, (scale_w, scale_h), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(infer_img, cv2.COLOR_BGR2RGB)

        res = model.predict(source=rgb, conf=CONF, iou=IOU, imgsz=INFER_SIZE, verbose=False)[0]

        person_found = False
        best_score = 0.0

        # compute scale factors back to output frame
        sx = iw / float(scale_w) if scale_w else 1.0
        sy = ih / float(scale_h) if scale_h else 1.0

        if hasattr(res, "boxes") and len(res.boxes) > 0:
            for b in res.boxes:
                conf = float(b.conf.item()) if hasattr(b.conf, "item") else float(b.conf)
                cls  = int(b.cls.item())   if hasattr(b.cls, "item")  else int(b.cls)
                if cls != 0:  # person only
                    continue

                xyxy = b.xyxy
                if hasattr(xyxy, "cpu"): xyxy = xyxy.cpu().numpy()
                xyxy = np.array(xyxy).reshape(-1)
                x1i, y1i, x2i, y2i = [float(v) for v in xyxy]

                # map back to output size
                x1, y1, x2, y2 = int(x1i * sx), int(y1i * sy), int(x2i * sx), int(y2i * sy)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(iw - 1, x2), min(ih - 1, y2)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)
                cv2.putText(frame, f"person {conf:.2f}", (x1, max(15, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)

                person_found = True
                if conf > best_score:
                    best_score = conf

        # save image on cooldown
        if person_found:
            now = time.time()
            if now - last_save > SAVE_COOLDOWN_SEC:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                img_path = SAVE_DIR / f"yolo_person_{ts}.jpg"
                Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).save(img_path, quality=92)
                with open(CSV_PATH, "a", newline="") as f:
                    csv.writer(f).writerow([ts, f"{best_score:.3f}", str(img_path)])
                print(f"[DETECT] {ts} score={best_score:.2f} -> {img_path}")
                last_save = now

        put_latest(infer_q, (frame, person_found, best_score))

def serve_thread():
    # take annotated frame, emit MJPEG, write MP4 (optional), cap FPS
    frame_time = 1.0 / max(TARGET_FPS, 1)
    last_t = 0.0
    while not stop_flag.is_set():
        try:
            frame, person_found, best = infer_q.get(timeout=0.2)
        except queue.Empty:
            continue

        # Optional recording
        if video_writer is not None:
            video_writer.write(frame)

        # Update MJPEG
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            data = jpg.tobytes()
            with _jpeg_lock:
                global _latest_jpeg
                _latest_jpeg = data

        if SHOW_WINDOW:
            cv2.imshow("YOLO Guard (fast)", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                stop_flag.set()

        # FPS cap
        now = time.time()
        dt = now - last_t
        if dt < frame_time:
            time.sleep(frame_time - dt)
        last_t = time.time()

# ---- Start threads ----
t_grab  = threading.Thread(target=grab_thread,  daemon=True)
t_infer = threading.Thread(target=infer_thread, daemon=True)
t_serve = threading.Thread(target=serve_thread, daemon=True)
t_grab.start(); t_infer.start(); t_serve.start()

print("[INFO] Running... Press Ctrl+C to stop.")
try:
    while t_grab.is_alive() and t_infer.is_alive() and t_serve.is_alive():
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    stop_flag.set()
    time.sleep(0.3)
    try:
        cap.release()
    except Exception:
        pass
    if video_writer is not None:
        try:
            video_writer.release()
        except Exception:
            pass
    cv2.destroyAllWindows()
    print(f"[INFO] Live MJPEG at: http://127.0.0.1:{MJPEG_PORT}{MJPEG_PATH}")
    print(f"[INFO] Images dir: {SAVE_DIR}")
    print(f"[INFO] CSV log: {CSV_PATH}")
    if WRITE_MP4:
        print(f"[INFO] MP4 saved: {OUT_MP4}")
