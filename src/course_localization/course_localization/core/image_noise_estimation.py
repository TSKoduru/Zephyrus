"""Robust image-only noise estimate for rejecting noise-supported texture.

This is an engineering confidence proxy, not calibrated Luxonis firmware
confidence. Fine scene texture can raise the estimate; clipping can lower it.
It does not use simulator noise settings, geometry, or pose.
"""

import cv2
import numpy as np


def image_noise_sigma(image):
    image = np.asarray(image)
    if image.ndim != 2 or min(image.shape) < 3 or not np.isfinite(image).all():
        raise ValueError("Expected a finite grayscale image at least 3x3")
    kernel = np.array([[1, -2, 1], [-2, 4, -2], [1, -2, 1]], np.float32)
    response = cv2.filter2D(image.astype(np.float32), -1, kernel)[1:-1, 1:-1]
    return float(np.median(abs(response)) / (6 * 0.6744897501960817))
