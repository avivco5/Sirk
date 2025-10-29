#!/usr/bin/env python3
# ASCII-only comments

import os, time, csv, smtplib
from email.message import EmailMessage
from pathlib import Path
import cv2
import numpy as np
from PIL import Image

# Optional .env loader
try:
    from dotenv import load_dotenv
    load_dotenv()
except Exception:
    pass

# ====== CONFIG ======
STREAM_URL   = os.getenv("STREAM_URL", "http://127.0.0.1:8000/stream")
MODEL_PATH   = os.getenv("MODEL_PATH", str(Path.home() / "security_models" / "ssd_mobilenet_v2_320x320_coco.tflite"))
CONF_THRESH  = float(os.getenv("CONF", "0.45"))

LOG_DIR      = Path.home() / "security_logs"
IMAGES_DIR   = LOG_DIR / "remote_images"
CSV_PATH     = LOG_DIR / "remote_detections.csv"

SMTP_HOST    = os.getenv("SMTP_HOST", "smtp.gmail.com")
SMTP_PORT    = int(os.getenv("SMTP_PORT", "587"))
EMAIL_FROM   = os.getenv("EMAIL_FROM", "")
EMAIL_TO     = os.getenv("EMAIL_TO", "")
EMAIL_USER   = os.getenv("EMAIL_USER", EMAIL_FROM)
EMAIL_PASS   = os.getenv("EMAIL_PASS", "")
EMAIL_COOLDOWN = int(os.getenv("EMAIL_COOLDOWN_SEC", "120"))

SHOW_WINDOW  = os.getenv("SHOW_WINDOW", "0") == "1"
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "85"))
# =====================

IMAGES_DIR.mkdir(parents=True, exist_ok=True)
LOG_DIR.mkdir(parents=True, exist_ok=True)
if not CSV_PATH.exists():
    with open(CSV_PATH, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "score", "image_path"])

# TFLite or TF lite
try:
    from tflite_runtime.interpreter import Interpreter
except Exception:
    from tensorflow.lite import Interpreter

print("[INFO] Loading model:", MODEL_PATH)
interpreter = Interpreter(MODEL_PATH)
interpreter.allocate_tensors()
in_det = interpreter.get_input_details()
out_det = interpreter.get_output_details()
IN_H, IN_W = in_det[0]["shape"][1], in_det[0]["shape"][2]
IN_TYPE    = in_det[0]["dtype"]

def infer(rgb):
    x = cv2.resize(rgb, (IN_W, IN_H))
    if IN_TYPE == np.float32:
        x = x.astype(np.float32) / 255.0
    else:
        x = x.astype(np.uint8)
    x = np.expand_dims(x, 0)
    interpreter.set_tensor(in_det[0]["index"], x)
    interpreter.invoke()
    boxes  = interpreter.get_tensor(out_det[0]["index"])[0]
    classes= interpreter.get_tensor(out_det[1]["index"])[0]
    scores = interpreter.get_tensor(out_det[2]["index"])[0]
    count  = int(interpreter.get_tensor(out_det[3]["index"])[0])
    return boxes, classes, scores, count

def send_email_with_image(subject, body, image_path):
    if not (EMAIL_FROM and EMAIL_TO and EMAIL_USER and EMAIL_PASS):
        print("[WARN] Email not configured. Skipping send.")
        return False
    msg = EmailMessage()
    msg["From"] = EMAIL_FROM
    msg["To"] = EMAIL_TO
    msg["Subject"] = subject
    msg.set_content(body)
    with open(image_path, "rb") as f:
        data = f.read()
    msg.add_attachment(data, maintype="image", subtype="jpeg",
                       filename=Path(image_path).name)
    with smtplib.SMTP(SMTP_HOST, SMTP_PORT) as s:
        s.ehlo(); s.starttls(); s.login(EMAIL_USER, EMAIL_PASS)
        s.send_message(msg)
    return True

print("[INFO] Opening stream:", STREAM_URL)
cap = cv2.VideoCapture(STREAM_URL)
if not cap.isOpened():
    raise SystemExit("Failed to open stream. Check STREAM_URL and network.")

print("[INFO] Running... (Ctrl+C to stop)")
last_email_ts = 0

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue

        # frame from MJPEG usually BGR, ensure contiguous
        bgr = np.ascontiguousarray(frame)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape

        boxes, classes, scores, n = infer(rgb)

        person_found = False
        best_score = 0.0
        for i in range(n):
            score = float(scores[i]); cls = int(classes[i])
            if score >= CONF_THRESH and cls in (0, 1):  # 'person' in COCO often 0 or 1
                ymin, xmin, ymax, xmax = boxes[i]
                x1, y1 = int(xmin*w), int(ymin*h)
                x2, y2 = int(xmax*w), int(ymax*h)
                x1 = max(0, min(x1, w-1)); x2 = max(0, min(x2, w-1))
                y1 = max(0, min(y1, h-1)); y2 = max(0, min(y2, h-1))
                cv2.rectangle(bgr, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(bgr, f"person {score:.2f}", (x1, max(0,y1-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                best_score = max(best_score, score)
                person_found = True

        if person_found:
            ts = time.strftime("%Y%m%d_%H%M%S")
            out_path = IMAGES_DIR / f"remote_person_{ts}.jpg"
            # Save as RGB
            Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)).save(out_path, quality=JPEG_QUALITY)
            with open(CSV_PATH, "a", newline="") as f:
                csv.writer(f).writerow([ts, f"{best_score:.3f}", str(out_path)])
            print(f"[DETECT] {ts} score={best_score:.2f} -> {out_path}")

            now = time.time()
            if now - last_email_ts > EMAIL_COOLDOWN:
                try:
                    sent = send_email_with_image(
                        subject=f"Security alert: person {ts}",
                        body=f"Person detected at {ts}, best score {best_score:.2f}. See attachment.",
                        image_path=str(out_path)
                    )
                    if sent:
                        last_email_ts = now
                        print("[INFO] Email sent.")
                except Exception as e:
                    print("[WARN] Email failed:", e)

        if SHOW_WINDOW:
            cv2.imshow("Remote stream", bgr)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        if not person_found:
            time.sleep(0.03)

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    if SHOW_WINDOW:
        cv2.destroyAllWindows()
    print("[INFO] Stopped.")
