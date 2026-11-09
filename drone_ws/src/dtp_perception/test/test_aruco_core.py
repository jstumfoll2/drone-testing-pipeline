"""Unit tests for the ROS-free ArUco pose estimator.

A marker is rendered synthetically at a known pose with a known camera, then
detected; the estimate must match the ground truth.
"""

import cv2
import numpy as np
import pytest

from dtp_perception.aruco_core import (
    ArucoPoseEstimator,
    camera_matrix_from_fov,
    dictionary_from_name,
    marker_object_points,
    rotation_matrix_to_quaternion,
)

# Matches PX4's Gazebo mono_cam: 1280x960, horizontal FOV 1.74 rad.
WIDTH, HEIGHT, HFOV = 1280, 960, 1.74
K = camera_matrix_from_fov(WIDTH, HEIGHT, HFOV)
DIST = np.zeros(5)
# PX4's aruco world: DICT_4X4_50 id 0, marker fills a 0.5 m square.
MARKER_LEN = 0.5


def render_marker(marker_id, rvec, tvec, marker_len=MARKER_LEN, dictionary="DICT_4X4_50"):
    """Render a marker into a white image at the given camera-frame pose."""
    side_px = 400
    border = side_px // 8
    d = dictionary_from_name(dictionary)
    if hasattr(cv2.aruco, "generateImageMarker"):
        tag = cv2.aruco.generateImageMarker(d, marker_id, side_px)
    else:
        tag = cv2.aruco.drawMarker(d, marker_id, side_px)
    # Quiet zone around the marker, like a printed tag.
    canvas = cv2.copyMakeBorder(tag, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    src = np.array(
        [[border, border], [border + side_px, border], [border + side_px, border + side_px],
         [border, border + side_px]], dtype=np.float32)
    dst, _ = cv2.projectPoints(marker_object_points(marker_len), rvec, tvec, K, DIST)
    homography = cv2.getPerspectiveTransform(src, dst.reshape(4, 2).astype(np.float32))
    out = cv2.warpPerspective(canvas, homography, (WIDTH, HEIGHT), borderValue=180,
                              flags=cv2.INTER_LINEAR)
    # The warp fills outside the canvas with grey; repaint the quiet zone white.
    quiet = cv2.warpPerspective(np.full_like(canvas, 255), homography, (WIDTH, HEIGHT),
                                borderValue=0)
    out = np.where((quiet > 0) & (out == 180), 255, out).astype(np.uint8)
    return out


def looking_down_rvec(tilt_rad=0.0):
    """Marker plane facing the camera (camera looking straight down at it), optional tilt."""
    rot = cv2.Rodrigues(np.array([np.pi + tilt_rad, 0.0, 0.0]))[0]
    return cv2.Rodrigues(rot)[0].reshape(3)


@pytest.mark.parametrize(
    "tvec",
    [
        np.array([0.0, 0.0, 3.0]),   # directly below, 3 m
        np.array([0.4, -0.3, 5.0]),  # offset, 5 m
        np.array([-0.8, 0.5, 8.0]),  # far and offset
    ],
)
def test_pose_matches_ground_truth(tvec):
    rvec = looking_down_rvec()
    img = render_marker(0, rvec, tvec)
    est = ArucoPoseEstimator(K, DIST, MARKER_LEN)
    obs = est.detect(img)
    assert len(obs) == 1
    o = obs[0]
    assert o.marker_id == 0
    # Position error under 2% of range.
    assert np.linalg.norm(o.tvec - tvec) < 0.02 * np.linalg.norm(tvec)
    assert o.reprojection_error_px < 1.0


def test_tilted_marker():
    tvec = np.array([0.2, 0.1, 4.0])
    rvec = looking_down_rvec(tilt_rad=np.deg2rad(20))
    obs = ArucoPoseEstimator(K, DIST, MARKER_LEN).detect(render_marker(0, rvec, tvec))
    assert len(obs) == 1
    assert np.linalg.norm(obs[0].tvec - tvec) < 0.08
    # Orientation within 3 degrees.
    r_est = cv2.Rodrigues(obs[0].rvec)[0]
    r_true = cv2.Rodrigues(rvec)[0]
    angle = np.degrees(np.arccos(np.clip((np.trace(r_est.T @ r_true) - 1) / 2, -1, 1)))
    assert angle < 3.0


def test_target_id_filter():
    img = render_marker(7, looking_down_rvec(), np.array([0.0, 0.0, 3.0]))
    assert ArucoPoseEstimator(K, DIST, MARKER_LEN, target_id=0).detect(img) == []
    obs = ArucoPoseEstimator(K, DIST, MARKER_LEN, target_id=7).detect(img)
    assert [o.marker_id for o in obs] == [7]


def test_no_marker_in_blank_image():
    blank = np.full((HEIGHT, WIDTH), 200, np.uint8)
    assert ArucoPoseEstimator(K, DIST, MARKER_LEN).detect(blank) == []


def test_marker_out_of_range_is_not_detected():
    # At 60 m the 0.5 m marker is ~8 px wide: too small to decode.
    img = render_marker(0, looking_down_rvec(), np.array([0.0, 0.0, 60.0]))
    assert ArucoPoseEstimator(K, DIST, MARKER_LEN).detect(img) == []


def test_quaternion_identity_and_unit_norm():
    np.testing.assert_allclose(rotation_matrix_to_quaternion(np.eye(3)), [0, 0, 0, 1], atol=1e-12)
    rot = cv2.Rodrigues(np.array([0.3, -1.2, 2.0]))[0]
    q = rotation_matrix_to_quaternion(rot)
    assert abs(np.linalg.norm(q) - 1.0) < 1e-12


def test_bad_marker_length():
    with pytest.raises(ValueError):
        ArucoPoseEstimator(K, DIST, 0.0)


def test_unknown_dictionary():
    with pytest.raises(ValueError):
        ArucoPoseEstimator(K, DIST, MARKER_LEN, dictionary="DICT_NOPE")


def test_camera_matrix_from_fov():
    k = camera_matrix_from_fov(WIDTH, HEIGHT, HFOV)
    assert k[0, 2] == WIDTH / 2 and k[1, 2] == HEIGHT / 2
    # Horizontal FOV recovered from fx.
    assert abs(2 * np.arctan(WIDTH / 2 / k[0, 0]) - HFOV) < 1e-9
