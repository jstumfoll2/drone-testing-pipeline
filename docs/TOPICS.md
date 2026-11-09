# Topic contract

The autonomy nodes only ever see the topics below. Each backend (`sim`, `hil`,
`real`) is responsible for producing them, so the same nodes and the same test
scenarios run unchanged from simulation to hardware.

## Sensors (produced by the backend)

| Topic | Type | QoS | Sim source |
|---|---|---|---|
| `/dtp/cam_down/image_raw` | `sensor_msgs/Image` | sensor data | Gazebo camera via `ros_gz_bridge` |
| `/dtp/cam_down/camera_info` | `sensor_msgs/CameraInfo` | sensor data | Gazebo camera via `ros_gz_bridge` |
| `/clock` | `rosgraph_msgs/Clock` | default | Gazebo (sim only; `use_sim_time:=true`) |

## Perception (produced by the autonomy stack)

| Topic | Type | Notes |
|---|---|---|
| `/dtp/marker/pose` | `geometry_msgs/PoseStamped` | Marker pose in the camera optical frame, stamped with the image capture time |
| `/dtp/marker/debug_image` | `sensor_msgs/Image` | Only when `publish_debug_image:=true` |

## Flight controller (PX4 via Micro XRCE-DDS)

PX4 publishes under `/fmu/out/*` and listens on `/fmu/in/*` with
best effort, transient local, depth 5 QoS (`dtp_test.px4_topics.px4_qos()`).

Since PX4 v1.16, messages with `MESSAGE_VERSION > 0` get a `_v<N>` suffix on
their topic name. Use `dtp_test.px4_topics.px4_topic(base, MsgType)` instead
of hard-coding names. Examples for PX4 v1.17:

| Topic | Type |
|---|---|
| `/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` |
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` |

## Evaluation only (never used by autonomy nodes)

| Topic | Type | Notes |
|---|---|---|
| `/dtp/ground_truth/tf` | `tf2_msgs/TFMessage` | All model poses from Gazebo; the scorer uses this in sim. On hardware it comes from mocap or RTK. |
