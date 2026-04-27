"""HSV-based color blob detection on BGR camera frames.

Ported from pincer's pincer/detect/color.py with light cleanup. Stays
framework-agnostic — no ROS imports — so it can be unit-tested directly
with cv2 + numpy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import cv2
import numpy as np


@dataclass
class Detection:
    """A single color blob detected in the image."""

    centroid: Tuple[int, int]   # (px, py) in image pixels
    area: float                 # contour area in pixels²
    bbox: Tuple[int, int, int, int]  # (x, y, w, h)


@dataclass
class ColorDetectorConfig:
    """Configuration for HSV color thresholding."""

    hsv_low: Tuple[int, int, int]
    hsv_high: Tuple[int, int, int]
    min_area: int = 100
    blur_ksize: int = 5


# A few common color presets in OpenCV HSV (H=0-179, S/V=0-255).
# Red wraps at H=0, so it has two ranges.
COLOR_RANGES: Dict[str, List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]] = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((35, 80, 50), (85, 255, 255))],
    "blue": [((100, 120, 50), (130, 255, 255))],
}


class ColorDetector:
    """HSV thresholding + morphology + contour finding."""

    def __init__(self, config: ColorDetectorConfig):
        self.config = config
        self._extra_ranges: List[Tuple[np.ndarray, np.ndarray]] = []

    @classmethod
    def from_color(cls, name: str, **kwargs) -> "ColorDetector":
        key = name.lower()
        if key not in COLOR_RANGES:
            raise ValueError(
                f"unknown color {name!r}; choose from: {sorted(COLOR_RANGES)}"
            )
        ranges = COLOR_RANGES[key]
        detector = cls(
            ColorDetectorConfig(hsv_low=ranges[0][0], hsv_high=ranges[0][1], **kwargs)
        )
        for low, high in ranges[1:]:
            detector._extra_ranges.append((np.array(low), np.array(high)))
        return detector

    def detect(self, bgr: np.ndarray) -> List[Detection]:
        """Detect color blobs in a BGR image. Returns largest-first."""
        ksize = self.config.blur_ksize
        blurred = cv2.GaussianBlur(bgr, (ksize, ksize), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, np.array(self.config.hsv_low), np.array(self.config.hsv_high))
        for low, high in self._extra_ranges:
            mask |= cv2.inRange(hsv, low, high)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections: List[Detection] = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.config.min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            moments = cv2.moments(cnt)
            if moments["m00"] == 0:
                continue
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            detections.append(Detection(centroid=(cx, cy), area=area, bbox=(x, y, w, h)))

        detections.sort(key=lambda d: d.area, reverse=True)
        return detections
