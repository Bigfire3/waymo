# waymo/edge_detection.py
import cv2
import numpy as np
import traceback

# --- Hilfsfunktionen (unverändert) ---
def threshold(channel, thresh=(120, 255), thresh_type=cv2.THRESH_BINARY):
    if not isinstance(channel, np.ndarray) or channel.size == 0:
        print(f"WARNUNG: threshold() erhielt ungültigen Input.")
        empty_img = np.array([], dtype=np.uint8)
        if isinstance(channel, np.ndarray) and channel.ndim == 2:
              empty_img = np.zeros(channel.shape, dtype=np.uint8)
        return thresh[0], empty_img
    if channel.dtype != np.uint8:
        try:
            if channel.dtype in [np.float32, np.float64] and np.min(channel)>=0 and np.max(channel)<=1:
                 channel_uint8 = (channel * 255).astype(np.uint8)
            else: channel_uint8 = channel.astype(np.uint8)
        except (ValueError, TypeError):
            print(f"WARNUNG: Konnte Kanal nicht zu uint8 konvertieren in threshold(). dtype={channel.dtype}")
            return thresh[0], np.zeros_like(channel, dtype=np.uint8)
    else: channel_uint8 = channel
    if len(channel_uint8.shape) > 2:
         print(f"WARNUNG: threshold() erhielt mehrkanaliges Bild (shape={channel_uint8.shape}), nehme ersten Kanal.")
         channel_uint8 = channel_uint8[:,:,0] if channel_uint8.shape[2]>0 else np.array([], dtype=np.uint8)
         if channel_uint8.size == 0: return thresh[0], np.array([], dtype=np.uint8)
    try:
         ret_thresh, binary_output = cv2.threshold(channel_uint8, thresh[0], thresh[1], thresh_type)
         return ret_thresh, binary_output
    except cv2.error as e: print(f"ERROR in cv2.threshold: {e}"); return thresh[0], np.zeros_like(channel_uint8)

def blur_gaussian(channel, ksize=3):
    if channel is None or channel.size == 0: return None
    ksize_int = int(ksize);
    if ksize_int % 2 == 0: ksize_int += 1
    if ksize_int <= 0 : ksize_int = 1
    try: return cv2.GaussianBlur(channel, (ksize_int, ksize_int), 0)
    except cv2.error as e: print(f"ERROR in cv2.GaussianBlur: {e}"); return channel

def mag_thresh(image, sobel_kernel=3, thresh=(0, 255)):
    if image is None or image.size == 0: return None
    if len(image.shape) == 3: gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif len(image.shape) == 2: gray = image
    else: print("WARNUNG: Ungültige Bilddimension in mag_thresh"); return None
    ksize_int = int(sobel_kernel);
    if ksize_int % 2 == 0: ksize_int +=1
    if ksize_int <= 0: ksize_int = 1
    try:
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ksize_int)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=ksize_int)
        gradmag = np.sqrt(sobelx**2 + sobely**2)
        max_val = np.max(gradmag)
        if max_val > 0: scale_factor = max_val / 255.0; gradmag = (gradmag / scale_factor).astype(np.uint8)
        else: gradmag = gradmag.astype(np.uint8)
        _, binary_output = cv2.threshold(gradmag, thresh[0], thresh[1], cv2.THRESH_BINARY)
        return binary_output
    except cv2.error as e: print(f"ERROR in mag_thresh (cv2): {e}"); return np.zeros_like(gray)
    except Exception as e: print(f"ERROR in mag_thresh: {e}\n{traceback.format_exc()}"); return np.zeros_like(gray)

# --- EdgeDetection Klasse mit Erweiterungen ---
class EdgeDetection:
    def __init__(self):
        pass

    def detect_and_fill_lanes(self, image,
                               blur_ksize=5, canny_low_thresh=50, canny_high_thresh=150,
                               morph_op="NONE", morph_ksize=3, morph_iterations=1, # NEUE Parameter für Morphologie
                               min_contour_area=10): # NEUER Parameter für Konturfilter
        """
        Erkennt Kanten, filtert optional Rauschen/kleine Objekte und füllt relevante Konturen.
        """
        if image is None or image.size == 0:
            print("WARNUNG: detect_and_fill_lanes erhielt None oder leeres Bild.")
            return None

        # --- Parameter validieren ---
        blur_ksize = int(blur_ksize); morph_ksize = int(morph_ksize); morph_iterations = int(morph_iterations); min_contour_area = int(min_contour_area)
        if blur_ksize <= 0: blur_ksize = 1
        elif blur_ksize % 2 == 0: blur_ksize += 1
        if morph_ksize <= 0: morph_ksize = 1
        elif morph_ksize % 2 == 0: morph_ksize += 1
        if morph_iterations < 1 : morph_iterations = 1
        if min_contour_area < 0 : min_contour_area = 0
        morph_op = morph_op.upper() # Sicherstellen, dass Großbuchstaben für Vergleich

        # --- Vorverarbeitung (Graustufen, uint8) ---
        # (Code wie zuvor)
        if len(image.shape) == 3:
            if image.shape[2] == 3: gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            elif image.shape[2] == 1: gray_image = image[:,:,0]
            else: print(f"WARNUNG: Unerwartete Kanalanzahl: {image.shape[2]}"); return None
        elif len(image.shape) == 2: gray_image = image
        else: print(f"WARNUNG: Unerwartete Bild-Dim: {len(image.shape)}"); return None
        if gray_image.dtype != np.uint8:
            try:
                if np.max(gray_image) <= 1.0 and np.min(gray_image) >= 0.0: gray_image_uint8 = (gray_image * 255).astype(np.uint8)
                else: gray_image_uint8 = np.clip(gray_image, 0, 255).astype(np.uint8)
            except (ValueError, TypeError): print("WARNUNG: Konnte Graustufenbild nicht zu uint8 konvertieren."); return None
        else: gray_image_uint8 = gray_image

        try:
            # --- Kernverarbeitung ---
            blurred_image = cv2.GaussianBlur(gray_image_uint8, (blur_ksize, blur_ksize), 0)
            edges = cv2.Canny(blurred_image, canny_low_thresh, canny_high_thresh)

            # --- Morphologische Operation (NEU) ---
            processed_edges = edges # Standardmäßig das Canny-Ergebnis verwenden
            if morph_op != "NONE":
                 kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_ksize, morph_ksize))
                 if morph_op == "OPEN":
                     processed_edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel, iterations=morph_iterations)
                 elif morph_op == "CLOSE":
                     processed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=morph_iterations)
                 # Hier könnten noch andere Operationen wie ERODE, DILATE hinzugefügt werden
                 else:
                      print(f"WARNUNG: Unbekannte morphologische Operation '{morph_op}'. Ignoriere.")

            # --- Konturen finden (auf dem ggf. morphologisch bearbeiteten Bild) ---
            contours, _ = cv2.findContours(processed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # --- Konturfilterung (NEU) ---
            filtered_contours = []
            if contours:
                 if min_contour_area > 0:
                     for c in contours:
                         if cv2.contourArea(c) >= min_contour_area:
                             filtered_contours.append(c)
                 else: # Kein Filter, wenn min_contour_area <= 0
                      filtered_contours = contours

            # --- Gefülltes Bild erstellen (mit gefilterten Konturen) ---
            filled_lanes_image = np.zeros_like(edges) # Größe basiert auf Canny-Ergebnis
            if filtered_contours: # Nur wenn Konturen übrig sind
                cv2.drawContours(filled_lanes_image, filtered_contours, -1, (255), thickness=cv2.FILLED)

            return filled_lanes_image

        except cv2.error as e: print(f"ERROR during OpenCV in detect_and_fill_lanes: {e}"); return None
        except Exception as e: print(f"ERROR in detect_and_fill_lanes: {e}\n{traceback.format_exc()}"); return None