import cv2
import numpy as np

from DLT import compute_homography
from ImageConversion import to_grayscale

# Parameters

num_runs = 1000
sigmas = np.arange(0,10.5,0.5)

# Experiment
img = to_grayscale(cv2.imread("../resources/Lenna.png"))
for sigma in sigmas:
    pass
