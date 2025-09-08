#!/usr/bin/env python3
import cv2

def open_camera(dev="/dev/video0", width=1280, height=720, fps=30):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Не удалось открыть камеру: {dev}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS,         fps)

    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    open_camera("/dev/video0", 1280, 720, 30)
