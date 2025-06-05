import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class EdgeDetection():
    def __init__(self, frame):
        self.frame = frame
        self.h, self.w = frame.shape[:2]
        self.cropped = frame[:, 160:self.w - 160]
        self.closed = None
        self.bold_closed = None

    def get_closed_edges(self, **params):

        d_value = params.get("d_value")
        sigmaColor = params.get("sigmaColor")
        sigmaSpace = params.get("sigmaSpace")
        blur = cv2.bilateralFilter(self.cropped, d=d_value, sigmaColor=sigmaColor, sigmaSpace=sigmaSpace)
        #cv2.imshow('Blur', blur)

        # Kanten finden
        threshold_1 = params.get("threshold_1")
        threshold_2 = params.get("threshold_2")
        edges = cv2.Canny(blur, threshold_1, threshold_2)

        self.h, self.w = edges.shape[:2]

        # --- Horizontale Linien hinzufügen ---
        cv2.line(edges, (10, self.h - 475), (self.w - 10, self.h - 475), color=255, thickness=1)
        cv2.line(edges, (10, self.h - 375), (self.w - 10, self.h - 375), color=255, thickness=1)
        cv2.line(edges, (10, self.h - 250), (self.w - 10, self.h - 250), color=255, thickness=1)
        cv2.line(edges, (10, self.h - 125), (self.w - 10, self.h - 125), color=255, thickness=1)
        cv2.line(edges, (1, self.h - 1), (100, self.h - 1), color=255, thickness=1)
        cv2.line(edges, (self.w - 100, self.h - 1), (self.w - 1, self.h - 1), color=255, thickness=1)

        # --- Vertikale Linien hinzufügen ---
        # cv2.line(edges, (1, 1), (1, self.h), color=255, thickness=1)
        # cv2.line(edges, (self.w-1, 1), (self.w-1, self.h), color=255, thickness=1)

        cv2.imshow("Edges", edges)

        # --- Lücken schließen ---
        ksize = params.get("ksize")
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
        self.closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        iterations = params.get("iterations")
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.bold_closed = cv2.dilate(self.closed, dilate_kernel, iterations=iterations)
        cv2.imshow("Edges (Closed)", self.bold_closed)

        return self.bold_closed

    def get_filled_areas(self, **params):

        # --- Konturen finden ---
        contours, _ = cv2.findContours(self.closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # --- Maske vorbereiten ---
        filled_mask = np.zeros_like(self.cropped, dtype=np.uint8)

        # --- Alle Flächen füllen ---
        for cnt in contours:
            cv2.drawContours(filled_mask, [cnt], -1, 255, cv2.FILLED)

        # --- Füllung invertieren ---
        maske_invertiert = cv2.bitwise_not(self.bold_closed)
        masked = cv2.bitwise_and(filled_mask, filled_mask, mask=maske_invertiert)
        contours, _ = cv2.findContours(masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        filled_masked = np.zeros_like(self.cropped, dtype=np.uint8)

        # --- Alle Flächen füllen ---
        for cnt in contours:
            if cv2.contourArea(cnt) > params.get("min_contour_area") and cv2.contourArea(cnt) < params.get("max_contour_area"):  # kleinere Objekte ignorieren
                cv2.drawContours(filled_masked, [cnt], -1, 255, cv2.FILLED)

        cv2.imshow("Gefüllte Flächen", filled_masked)
        cv2.waitKey(1)

        return filled_masked
