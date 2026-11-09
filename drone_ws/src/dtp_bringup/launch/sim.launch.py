"""Bring up the full simulation backend plus the autonomy stack.

Starts, in order:
  1. Micro XRCE-DDS Agent (UDP 8888), the PX4 <-> ROS 2 bridge
  2. PX4 SITL + Gazebo Harmonic (`make px4_sitl gz_<vehicle>_<world>`)
  3. ros_gz_bridge: downward camera, camera info, ground-truth poses, clock
  4. Autonomy nodes (marker detector for now)

Examples:
  ros2 launch dtp_bringup sim.launch.py
  ros2 launch dtp_bringup sim.launch.py scenario:=smoke_aruco headless:=true
  ros2 launch dtp_bringup sim.launch.py world:=windy vehicle:=x500_mono_cam_down

The topic contract (what the autonomy nodes see) is the same for every
backend; see docs/TOPICS.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _scenario_defaults(name: str):
    """Load vehicle/world/marker settings from a dtp_test scenario, if one is named."""
    if not name:
        return {}
    from dtp_test.scenario import load_scenario  # local import: dtp_test is optional at parse time

    path = os.path.join(get_package_share_directory("dtp_test"), "scenarios", f"{name}.yaml")
    s = load_scenario(path)
    return {
        "vehicle": s.vehicle,
        "world": s.world,
        "marker_length_m": s.marker.length_m,
        "marker_id": s.marker.marker_id,
        "dictionary": s.marker.dictionary,
    }


def _launch_setup(context, *args, **kwargs):
    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        "scenario", "vehicle", "world", "px4_dir", "headless", "start_agent", "start_px4",
        "use_sim_time", "camera_gz_topic", "agent_port", "require_gcs")}
    overrides = _scenario_defaults(cfg["scenario"])
    vehicle = overrides.get("vehicle", cfg["vehicle"])
    world = overrides.get("world", cfg["world"])
    model = f"{vehicle}_0"  # PX4 names the first spawned instance <vehicle>_0
    use_sim_time = cfg["use_sim_time"].lower() == "true"
    target = f"gz_{vehicle}" if world == "default" else f"gz_{vehicle}_{world}"

    actions = [LogInfo(msg=f"[dtp] PX4 target {target}, model {model}, world {world}")]

    if cfg["start_agent"].lower() == "true":
        actions.append(ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", cfg["agent_port"]],
            name="microxrce_agent", output="screen"))

    if cfg["start_px4"].lower() == "true":
        px4_dir = os.path.expanduser(cfg["px4_dir"])
        if not os.path.isdir(px4_dir):
            raise RuntimeError(f"PX4 not found at {px4_dir}; run scripts/setup_px4.sh or set px4_dir")
        env = {"PX4_GZ_WORLD": world}
        if cfg["require_gcs"].lower() != "true":
            # PX4 v1.17 refuses to arm without a ground station link while the
            # datalink-loss failsafe (NAV_DLL_ACT) is enabled. Automated sim
            # runs have no QGroundControl, so turn that failsafe off here only.
            env["PX4_PARAM_NAV_DLL_ACT"] = "0"
        if cfg["headless"].lower() == "true":
            env["HEADLESS"] = "1"
        actions.append(ExecuteProcess(
            cmd=["make", "px4_sitl", target], cwd=px4_dir, additional_env=env,
            name="px4_sitl", output="screen"))

    cam = cfg["camera_gz_topic"] or (
        f"/world/{world}/model/{model}/link/camera_link/sensor/imager")  # mono_cam's sensor name
    bridge_args = [
        f"{cam}/image@sensor_msgs/msg/Image[gz.msgs.Image",
        f"{cam}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        f"/world/{world}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    ]
    actions.append(Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge",
        arguments=bridge_args, output="screen",
        remappings=[
            (f"{cam}/image", "/dtp/cam_down/image_raw"),
            (f"{cam}/camera_info", "/dtp/cam_down/camera_info"),
            (f"/world/{world}/pose/info", "/dtp/ground_truth/tf"),
        ],
        parameters=[{"use_sim_time": use_sim_time}]))

    detector_params = os.path.join(
        get_package_share_directory("dtp_perception"), "config", "marker_detector.yaml")
    marker_overrides = {}
    if "marker_length_m" in overrides:
        marker_overrides = {
            "marker_length_m": float(overrides["marker_length_m"]),
            "target_id": int(overrides["marker_id"]),
            "dictionary": overrides["dictionary"],
        }
    actions.append(Node(
        package="dtp_perception", executable="marker_detector", name="marker_detector",
        output="screen",
        # PX4's Tools/setup/ubuntu.sh pip-installs NumPy 2 into ~/.local, which
        # breaks Ubuntu's python3-opencv (built against NumPy 1). Keep the
        # user site out of our Python nodes so they use the system packages.
        additional_env={"PYTHONNOUSERSITE": "1"},
        parameters=[detector_params, marker_overrides, {"use_sim_time": use_sim_time}],
        remappings=[
            ("image", "/dtp/cam_down/image_raw"),
            ("camera_info", "/dtp/cam_down/camera_info"),
            ("marker_pose", "/dtp/marker/pose"),
            ("debug_image", "/dtp/marker/debug_image"),
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="",
                              description="dtp_test scenario name; overrides vehicle/world/marker"),
        DeclareLaunchArgument("vehicle", default_value="x500_mono_cam_down"),
        DeclareLaunchArgument("world", default_value="aruco"),
        DeclareLaunchArgument("px4_dir", default_value=os.environ.get("PX4_DIR", "~/PX4-Autopilot")),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("start_agent", default_value="true"),
        DeclareLaunchArgument("start_px4", default_value="true",
                              description="false to attach to an already running PX4/Gazebo"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("agent_port", default_value="8888"),
        DeclareLaunchArgument("require_gcs", default_value="false",
                              description="true keeps PX4's GCS-link arming check (use with QGroundControl)"),
        DeclareLaunchArgument("camera_gz_topic", default_value="",
                              description="override the Gazebo camera topic prefix (see `gz topic -l`)"),
        OpaqueFunction(function=_launch_setup),
    ])
