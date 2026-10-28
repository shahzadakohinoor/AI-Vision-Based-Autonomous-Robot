"""
Computer-side AI object detection for the ESP32-CAM rover.

Install:
  py -m pip install ultralytics opencv-python requests

Run:
  py ai_vision.py --esp32 http://192.168.1.50 --target bottle
  py ai_vision.py --esp32 http://192.168.1.50 --target bottle --autopilot

Without --autopilot, it only displays detections and never moves the rover.
With --autopilot, it sends slow direction commands for the selected detected class.
Use wheels-off-ground testing first. Ultrasonic obstacle avoidance in the ESP32 firmware
overrides forward movement when an obstacle is detected.
"""
import argparse
import time
import cv2
import requests
from ultralytics import YOLO

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp32", required=True, help="Base URL, e.g. http://192.168.1.50")
    ap.add_argument("--target", default="bottle", help="COCO class name to track, e.g. bottle, chair, person")
    ap.add_argument("--autopilot", action="store_true", help="Enable movement commands; otherwise detection only")
    ap.add_argument("--confidence", type=float, default=0.45)
    args = ap.parse_args()

    base = args.esp32.rstrip("/")
    stream_url = base.replace("http://", "http://").replace("https://", "http://") + ":81/stream"
    model = YOLO("yolov8n.pt")  # downloads pretrained weights on first run
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        raise SystemExit("Cannot open stream. Check the ESP32 IP, Wi-Fi, and http://IP:81/stream")

    last_cmd = ""
    last_send = 0.0

    def send(cmd):
        nonlocal last_cmd, last_send
        now = time.time()
        # Rate-limit HTTP requests and avoid sending duplicate commands continuously.
        if cmd != last_cmd or now - last_send > 0.35:
            try:
                requests.get(base + "/drive", params={"cmd": cmd}, timeout=0.5)
            except requests.RequestException:
                pass
            last_cmd, last_send = cmd, now

    print("AI vision running. Press q to quit.")
    print("Autopilot:", "ON" if args.autopilot else "OFF (detection only)")
    while True:
        ok, frame = cap.read()
        if not ok:
            if args.autopilot:
                send("S")
            time.sleep(0.05)
            continue

        results = model.predict(frame, conf=args.confidence, imgsz=320, verbose=False)
        annotated = results[0].plot()
        h, w = frame.shape[:2]
        chosen = []
        names = results[0].names

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label = names.get(cls_id, str(cls_id))
            if label.lower() == args.target.lower():
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx = (x1 + x2) // 2
                area_ratio = max(0, x2-x1) * max(0, y2-y1) / float(w*h)
                chosen.append((area_ratio, cx))

        if args.autopilot:
            if not chosen:
                send("S")
            else:
                # Follow the largest instance of the selected class.
                area_ratio, cx = max(chosen)
                offset = cx - w // 2
                if abs(offset) > int(w * 0.16):
                    send("L" if offset < 0 else "R")
                elif area_ratio < 0.18:
                    send("F")
                else:
                    send("S")  # target is close/large in frame; stop
        cv2.putText(annotated, f"Target: {args.target} | autopilot: {args.autopilot}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)
        cv2.imshow("ESP32-CAM AI Vision Rover", annotated)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    if args.autopilot:
        send("S")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
