import cv2
import numpy as np

def get_line_markings(frame, **params):
    height, width = frame.shape[:2]

    cropped = frame[:, 160:width - 160]
    #cv2.imshow('Cropped', cropped)

    d_value = params.get("d_value")
    sigmaColor = params.get("sigmaColor")
    sigmaSpace = params.get("sigmaSpace")
    blur = cv2.bilateralFilter(cropped, d=d_value, sigmaColor=sigmaColor, sigmaSpace=sigmaSpace)
    #cv2.imshow('Blur', blur)

    # Kanten finden
    threshold_1 = params.get("threshold_1")
    threshold_2 = params.get("threshold_2")
    edges = cv2.Canny(blur, threshold_1, threshold_2)

    # Extra Linien
    h, w = cropped.shape[:2]
    cv2.line(edges, (0, h - 1), (w, h - 1), color=255, thickness=1)
    cv2.line(edges, (w - 150, h - 450), (w - 10, h - 450), color=255, thickness=1)
    cv2.line(edges, (w - 150, h - 200), (w - 10, h - 200), color=255, thickness=1)

    # --- Lücken schließen ---
    ksize = params.get("ksize")
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    #cv2.imshow("Edges (Closed)", closed)

    # --- Konturen finden ---
    contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # --- Maske vorbereiten ---
    filled_mask = np.zeros_like(cropped, dtype=np.uint8)

    # --- Alle Flächen füllen ---
    for cnt in contours:
        cv2.drawContours(filled_mask, [cnt], -1, 255, cv2.FILLED)

    return filled_mask

    # cv2.imshow("Gefüllte Flächen", filled_mask)
    # cv2.waitKey(1)

