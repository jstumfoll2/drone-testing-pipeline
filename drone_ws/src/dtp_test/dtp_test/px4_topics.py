"""PX4 uXRCE-DDS topic naming and QoS helpers.

Since PX4 v1.16, versioned messages are published with a ``_v<N>`` suffix
(e.g. ``/fmu/out/vehicle_status_v1``), where N is the message's
``MESSAGE_VERSION`` constant. Version 0 messages have no suffix.
See PX4 ``src/modules/uxrce_dds_client/utilities.hpp`` (generate_topic_name).
"""

from __future__ import annotations


def px4_topic(base: str, msg_type=None, version: int | None = None) -> str:
    """Return the DDS topic PX4 uses for ``base`` given the message version.

    ``msg_type`` may be a px4_msgs class (its ``MESSAGE_VERSION`` is used), or
    pass ``version`` directly.
    """
    if version is None:
        version = int(getattr(msg_type, "MESSAGE_VERSION", 0)) if msg_type is not None else 0
    if version < 0:
        raise ValueError("message version must be >= 0")
    return f"{base}_v{version}" if version else base


def px4_qos():
    """QoS that matches PX4's publishers (best effort, transient local, depth 5)."""
    from rclpy.qos import (  # imported lazily so this module is testable without ROS
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )
