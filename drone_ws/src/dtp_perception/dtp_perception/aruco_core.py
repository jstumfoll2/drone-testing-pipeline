"""ROS-free ArUco detection and pose estimation (landing system V1).

Kept free of ROS imports so it can be unit-tested anywhere and reused by the
V2/V3 detectors for comparison. Works on both the legacy OpenCV ArUco API
(OpenCV <= 4.6, Ubuntu 24.04's system package) and the newer ``ArucoDetector``
API (OpenCV >= 4.7).

Conventions: poses are the marker's position and orientation in the camera
optical frame (x right, y down, z forward), in metres.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class MarkerObservation:
    """One detected marker and its estimated pose in the camera optical frame."""

    marker_id: int
    corners_px: np.ndarray  # (4, 2) image corners, ArUco order: TL, TR, BR, BL
    rvec: np.ndarray  # (3,) Rodrigues rotation, marker -> camera
    tvec: np.ndarray  # (3,) marker centre in camera frame, metres
    reprojection_error_px: float

    @property
    def distance_m(self) -> float:
        return float(np.linalg.norm(self.tvec))

    def quaternion_xyzw(self) -> np.ndarray:
        """Orientation of the marker in the camera frame as (x, y, z, w)."""
        rot, _ = cv2.Rodrigues(self.rvec.reshape(3, 1))
        return rotation_matrix_to_quaternion(rot)


def rotation_matrix_to_quaternion(rot: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit quaternion (x, y, z, w)."""
    m = np.asarray(rot, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    if q[3] < 0.0:  # keep a canonical hemisphere
        q = -q
    return q


def marker_object_points(marker_length_m: float) -> np.ndarray:
    """3D corners of a square marker centred at the origin, in ArUco corner order."""
    h = marker_length_m / 2.0
    return np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64
    )


def dictionary_from_name(name: str):
    """Look up a predefined ArUco dictionary by name, e.g. ``"DICT_4X4_50"``."""
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


class ArucoPoseEstimator:
    """Detect ArUco markers in an image and estimate each marker's pose."""

    def __init__(
        self,
        camera_matrix: Sequence[Sequence[float]],
        dist_coeffs: Sequence[float],
        marker_length_m: float,
        dictionary: str = "DICT_4X4_50",
        target_id: Optional[int] = None,
    ) -> None:
        if marker_length_m <= 0.0:
            raise ValueError("marker_length_m must be positive")
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        self.marker_length_m = float(marker_length_m)
        self.target_id = target_id
        self._object_points = marker_object_points(self.marker_length_m)
        self._dictionary = dictionary_from_name(dictionary)
        self._detect = self._make_detector(self._dictionary)

    @staticmethod
    def _make_detector(dictionary):
        if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            return detector.detectMarkers
        params = cv2.aruco.DetectorParameters_create()  # OpenCV <= 4.6
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)

    def update_camera(
        self, camera_matrix: Sequence[Sequence[float]], dist_coeffs: Sequence[float]
    ) -> None:
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)

    def detect(self, image: np.ndarray) -> List[MarkerObservation]:
        """Return pose estimates for every (or only the target) marker in ``image``."""
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detect(gray)
        if ids is None or len(ids) == 0:
            return []

        observations: List[MarkerObservation] = []
        for marker_corners, marker_id in zip(corners, ids.ravel()):
            if self.target_id is not None and int(marker_id) != self.target_id:
                continue
            img_pts = marker_corners.reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                self._object_points,
                img_pts,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            projected, _ = cv2.projectPoints(
                self._object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
            )
            err = float(np.mean(np.linalg.norm(projected.reshape(4, 2) - img_pts, axis=1)))
            observations.append(
                MarkerObservation(
                    marker_id=int(marker_id),
                    corners_px=img_pts,
                    rvec=rvec.reshape(3),
                    tvec=tvec.reshape(3),
                    reprojection_error_px=err,
                )
            )
        observations.sort(key=lambda o: o.distance_m)
        return observations


def camera_matrix_from_fov(width: int, height: int, horizontal_fov_rad: float) -> np.ndarray:
    """Pinhole intrinsics for an ideal simulated camera (square pixels, centred)."""
    fx = (width / 2.0) / np.tan(horizontal_fov_rad / 2.0)
    return np.array([[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]])
