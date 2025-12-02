import cv2
import numpy as np

# Load grayscale map (0–255)
img = cv2.imread("maps/project_map.pgm", cv2.IMREAD_GRAYSCALE)
if img is None:
    raise RuntimeError("Could not open maps/project_map.pgm")

FREE_THRESHOLD = 254
OCC_THRESHOLD = 205

occ_grid = np.full(img.shape, -1, dtype=np.int8)

occ_grid[img >= FREE_THRESHOLD] = 0
occ_grid[img <= OCC_THRESHOLD] = 1
