"""ROS 2 node: detect the landing marker and publish its pose (landing system V1).

Subscribes:
  image        (sensor_msgs/Image)       downward camera, remapped to /dtp/cam_down/image_raw
  camera_info  (sensor_msgs/CameraInfo)  intrinsics, remapped to /dtp/cam_down/camera_info
Publishes:
  marker_pose  (geometry_msgs/PoseStamped)  marker pose in the camera optical frame,
                                            remapped to /dtp/marker/pose
  debug_image  (sensor_msgs/Image)          optional annotated image

Successor of the paper's Appendix D ``marker_detector.py`` (ROS 1), ported to
ROS 2 and the current OpenCV API.
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from dtp_perception.aruco_core import ArucoPoseEstimator


class MarkerDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("marker_detector")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_length_m", 0.5)  # PX4 aruco world tag
        self.declare_parameter("target_id", -1)  # -1 = any marker
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("optical_frame_id", "")  # override header frame if set

        target_id = self.get_parameter("target_id").value
        self._estimator_args = dict(
            marker_length_m=float(self.get_parameter("marker_length_m").value),
            dictionary=str(self.get_parameter("dictionary").value),
            target_id=None if target_id < 0 else int(target_id),
        )
        self._estimator: ArucoPoseEstimator | None = None
        self._bridge = CvBridge()
        self._debug = bool(self.get_parameter("publish_debug_image").value)
        self._frame_override = str(self.get_parameter("optical_frame_id").value)

        self._pose_pub = self.create_publisher(
            PoseStamped,
            "marker_pose",
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=10),
        )
        self._debug_pub = self.create_publisher(Image, "debug_image", qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "camera_info", self._on_camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, "image", self._on_image, qos_profile_sensor_data)

        self._frames = 0
        self._detections = 0
        self._proc_time = 0.0
        self.create_timer(5.0, self._report)
        self.get_logger().info(f"marker_detector ready: {self._estimator_args}")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = np.array(msg.k, dtype=float).reshape(3, 3)
        d = np.array(msg.d, dtype=float) if len(msg.d) else np.zeros(5)
        if self._estimator is None:
            self._estimator = ArucoPoseEstimator(k, d, **self._estimator_args)
            self.get_logger().info(f"camera intrinsics received: fx={k[0, 0]:.1f}")
        else:
            self._estimator.update_camera(k, d)

    def _on_image(self, msg: Image) -> None:
        if self._estimator is None:
            self.get_logger().warn("waiting for camera_info", throttle_duration_sec=5.0)
            return
        t0 = time.perf_counter()
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        observations = self._estimator.detect(frame)
        self._proc_time += time.perf_counter() - t0
        self._frames += 1

        if observations:
            self._detections += 1
            obs = observations[0]  # closest marker
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp  # keep capture time for latency metrics
            pose.header.frame_id = self._frame_override or msg.header.frame_id
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
                float(v) for v in obs.tvec)
            qx, qy, qz, qw = obs.quaternion_xyzw()
            pose.pose.orientation.x, pose.pose.orientation.y = float(qx), float(qy)
            pose.pose.orientation.z, pose.pose.orientation.w = float(qz), float(qw)
            self._pose_pub.publish(pose)

        if self._debug and self._debug_pub.get_subscription_count() > 0:
            annotated = frame.copy()
            for obs in observations:
                cv2.polylines(annotated, [obs.corners_px.astype(np.int32)], True, (0, 255, 0), 2)
                cv2.drawFrameAxes(annotated, self._estimator.camera_matrix,
                                  self._estimator.dist_coeffs, obs.rvec, obs.tvec,
                                  self._estimator.marker_length_m / 2)
            out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header = msg.header
            self._debug_pub.publish(out)

    def _report(self) -> None:
        if self._frames == 0:
            return
        self.get_logger().info(
            f"frames={self._frames} detection_rate={self._detections / self._frames:.2f} "
            f"mean_proc_ms={1000 * self._proc_time / self._frames:.1f}")
        self._frames = self._detections = 0
        self._proc_time = 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
