"""Object detection — Listings 3.6 and 3.7.

Fragments from the thesis. Imports added; function bodies are verbatim.
"""
import cv2
import numpy as np


# Listing 3.6 — Background subtraction
def get_mask_from_frames(background, frame):
    diff = cv2.absdiff(background, frame)
    diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(diff_gray, 50, 255)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask, diff


# Listing 3.7 — Contour filtering + chromatic dominance colour labelling
def identify_cubes(mask, frame):
    results = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if area > 900 and 0.8 < w / h < 1.2:
            roi = frame[y:y + h, x:x + w]
            mean_color = cv2.mean(roi)[:3]  # BGR

            label = None
            if mean_color[0] > mean_color[1] + 10 and mean_color[0] > mean_color[2] + 10:
                label = "BLUE"
            elif mean_color[1] > mean_color[0] and mean_color[1] > mean_color[2]:
                label = "GREEN"
            elif mean_color[2] > mean_color[0] + 10 and mean_color[2] > mean_color[1] + 10:
                label = "RED"

            if label:
                results.append({'label': label, 'bbox': (x, y, w, h)})
    return results
