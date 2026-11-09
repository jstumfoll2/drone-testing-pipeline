"""Gate 0 smoke test: take off in SITL and confirm the whole chain is alive.

Pass criteria (all within ``timeout_s``):
  1. PX4 status arrives over uXRCE-DDS and pre-flight checks pass.
  2. The vehicle arms and climbs (PX4 Takeoff mode, default takeoff altitude).
  3. Camera images arrive on /dtp/cam_down/image_raw.
  4. The marker detector publishes at least ``min_marker_poses`` poses on /dtp/marker/pose.

Exits 0 on pass, 1 on fail. Run against ``ros2 launch dtp_bringup sim.launch.py``.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from dtp_test.px4_topics import px4_qos, px4_topic


class SmokeFlight(Node):
    def __init__(self) -> None:
        super().__init__("smoke_flight")
        self.declare_parameter("timeout_s", 180.0)
        self.declare_parameter("min_marker_poses", 10)
        self.declare_parameter("min_altitude_m", 2.0)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.min_poses = int(self.get_parameter("min_marker_poses").value)
        self.min_alt = float(self.get_parameter("min_altitude_m").value)

        self.status: VehicleStatus | None = None
        self.altitude_m = 0.0
        self.images = 0
        self.marker_poses = 0
        self.takeoff_sent_at: float | None = None
        self.started = time.monotonic()

        qos = px4_qos()
        self.create_subscription(VehicleStatus, px4_topic("/fmu/out/vehicle_status", VehicleStatus),
                                 self._on_status, qos)
        self.create_subscription(VehicleLocalPosition,
                                 px4_topic("/fmu/out/vehicle_local_position", VehicleLocalPosition),
                                 self._on_position, qos)
        self.create_subscription(Image, "/dtp/cam_down/image_raw", self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/dtp/marker/pose", self._on_marker, 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, px4_topic("/fmu/in/vehicle_command", VehicleCommand), 10)
        self.create_timer(0.5, self._tick)
        self.result: bool | None = None

    # --- callbacks -------------------------------------------------------
    def _on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        if msg.z_valid:
            self.altitude_m = -float(msg.z)  # NED: z down

    def _on_image(self, _msg: Image) -> None:
        self.images += 1

    def _on_marker(self, _msg: PoseStamped) -> None:
        self.marker_poses += 1

    # --- logic -------------------------------------------------------------
    def _command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        for i in range(1, 8):
            setattr(msg, f"param{i}", float(params.get(f"param{i}", 0.0)))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started
        if elapsed > self.timeout_s:
            self._finish(False, "timeout")
            return
        s = self.status
        if s is None:
            self.get_logger().info("waiting for PX4 vehicle_status...", throttle_duration_sec=5.0)
            return
        armed = s.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if not armed:
            if not s.pre_flight_checks_pass:
                self.get_logger().info("waiting for pre-flight checks...", throttle_duration_sec=5.0)
                return
            # Takeoff mode first, then arm (repeat every 2 s until armed).
            if self.takeoff_sent_at is None or time.monotonic() - self.takeoff_sent_at > 2.0:
                self._command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                              param4=math.nan, param5=math.nan,
                              param6=math.nan, param7=math.nan)
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.takeoff_sent_at = time.monotonic()
                self.get_logger().info("sent takeoff + arm")
            return

        self.get_logger().info(
            f"alt={self.altitude_m:.1f} m images={self.images} marker_poses={self.marker_poses}",
            throttle_duration_sec=2.0)
        if self.altitude_m >= self.min_alt and self.images > 0 and self.marker_poses >= self.min_poses:
            self._finish(True, "PX4 link, takeoff, camera and marker detection all OK")

    def _finish(self, ok: bool, why: str) -> None:
        if self.result is not None:
            return
        self.result = ok
        summary = (f"{'PASS' if ok else 'FAIL'}: {why} | status={'yes' if self.status else 'no'} "
                   f"alt={self.altitude_m:.1f} images={self.images} marker_poses={self.marker_poses}")
        (self.get_logger().info if ok else self.get_logger().error)(summary)
        if ok:  # leave the vehicle in a safe state
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmokeFlight()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Give the land command a moment to go out.
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.result = False
    finally:
        ok = bool(node.result)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
