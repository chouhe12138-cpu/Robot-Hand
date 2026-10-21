"""Shared resizable camera display with source-coordinate mouse selection."""
import logging
import re
import subprocess
import time

import cv2
import numpy as np


def create_camera_view(name, initial_width, initial_height, on_mouse):
    """Resize the displayed pixels with the viewport; retain source selection coordinates."""
    transform = None
    viewport = (initial_width, initial_height)
    last_size_check = 0.0
    size_warning_logged = False
    cv2.namedWindow(name, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO | cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow(name, initial_width, initial_height)

    def mouse(event, x, y, flags, param):
        if transform is None:
            return
        left, top, scaled_w, scaled_h, source_w, source_h = transform
        if event == cv2.EVENT_LBUTTONDOWN and not (
                left <= x < left + scaled_w and top <= y < top + scaled_h):
            return
        source_x = min(source_w - 1, max(0, int((x - left) * source_w / scaled_w)))
        source_y = min(source_h - 1, max(0, int((y - top) * source_h / scaled_h)))
        on_mouse(event, source_x, source_y, flags, param)

    cv2.setMouseCallback(name, mouse)

    def show(frame):
        nonlocal transform, viewport, last_size_check, size_warning_logged
        # This workstation's Qt backend reports the source image size from
        # getWindowImageRect, even when the actual window is much larger.
        # GUI_NORMAL has no toolbar, so the X11 client size is our canvas size.
        now = time.monotonic()
        if now - last_size_check >= 0.2:
            last_size_check = now
            try:
                info = subprocess.run(["xwininfo", "-name", name], capture_output=True,
                                      text=True, timeout=0.2, check=True).stdout
                width_match = re.search(r"^\s*Width:\s*(\d+)", info, re.MULTILINE)
                height_match = re.search(r"^\s*Height:\s*(\d+)", info, re.MULTILINE)
                if width_match and height_match:
                    size = int(width_match[1]), int(height_match[1])
                    if min(size) > 1:
                        viewport = size
            except (OSError, subprocess.SubprocessError) as error:
                if not size_warning_logged:
                    logging.warning("Cannot read camera window size; using last size: %s", error)
                    size_warning_logged = True
        width, height = viewport
        source_h, source_w = frame.shape[:2]
        scale = min(width / source_w, height / source_h)
        scaled_w, scaled_h = max(1, round(source_w * scale)), max(1, round(source_h * scale))
        left, top = (width - scaled_w) // 2, (height - scaled_h) // 2
        canvas = np.zeros((height, width, 3), dtype=frame.dtype)
        canvas[top:top + scaled_h, left:left + scaled_w] = cv2.resize(
            frame, (scaled_w, scaled_h), interpolation=cv2.INTER_LINEAR)
        transform = left, top, scaled_w, scaled_h, source_w, source_h
        cv2.imshow(name, canvas)

    return show


def draw_handover_hint(frame, hint, color, detail=""):
    """R2H confirmation text, shared by both handover directions."""
    cv2.putText(frame, hint, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
    if detail:
        cv2.putText(frame, detail[:85], (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1)


def draw_physical_hand(frame, landmarks, score, side, drawing, connections):
    """R2H skeleton, palm marker and physical-hand identity label."""
    drawing.draw_landmarks(frame, landmarks, connections)
    height, width = frame.shape[:2]
    palm = landmarks.landmark[9]
    cx, cy = int(palm.x * width), int(palm.y * height)
    cv2.circle(frame, (cx, cy), 10, (255, 0, 0), cv2.FILLED)
    cv2.putText(frame, f"Physical {side} hand {score:.2f}", (cx - 70, cy - 15),
                cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 0, 0), 2)
