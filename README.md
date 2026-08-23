# Sensor fusion

This standalone ROS 2 Humble module provides an EKF for differential-drive
odometry. It fuses commanded velocity, IMU yaw rate, and encoder velocity and
publishes odometry plus the optional `odom` to `base_link` transform.

The runtime image uses Eclipse Cyclone DDS and starts the node with the packaged
configuration file by default:

```bash
ros2 run ekf ekf_node \
  --ros-args \
  --params-file /opt/robot_module/share/ekf/config/ekf.yaml
```

## Build and run

Build the image directly from this module directory:

```bash
docker build -t robot-modules/sensor-fusion:humble .
```

Run it on the host network so DDS discovery works with the other robot modules:

```bash
docker run --rm --network host \
  -e ROS_DOMAIN_ID=0 \
  robot-modules/sensor-fusion:humble
```

The example Compose file provides the same defaults:

```bash
docker compose up --build
```

To use a robot-specific configuration, keep it in the containing robot project,
mount it read-only, and override the image command from that project's script:

```bash
docker run --rm --network host \
  -e ROS_DOMAIN_ID=0 \
  -v /absolute/path/to/robot-ekf.yaml:/config/ekf.yaml:ro \
  robot-modules/sensor-fusion:humble \
  ros2 run ekf ekf_node --ros-args --params-file /config/ekf.yaml
```

## Configuration

The packaged defaults are in `src/ekf/config/ekf.yaml`. They preserve the
original filter tuning and interfaces while also exposing topics, frame IDs,
prediction frequency, TF publication, state/process covariance diagonals, and
measurement covariance.

Parameters are loaded and validated at startup. Restart the container after a
configuration change.

### Encoder input modes

`encoder_input_type: "twist"` is the default. The node consumes a
`geometry_msgs/msg/TwistStamped` from `encoder_twist_topic`; its `linear.x` and
`angular.z` fields are used directly as the differential-drive velocity
measurement.

Set `encoder_input_type: "joint_states"` to consume wheel angular velocities
from `sensor_msgs/msg/JointState`. In this mode, configure `left_wheel_joint`
and `right_wheel_joint`. The node finds the joints by name, reads their velocity
values in rad/s, and computes:

```text
left_linear  = radius * left_angular
right_linear = radius * right_angular
linear       = (right_linear + left_linear) / 2
angular      = (right_linear - left_linear) / wheel_base
```

Use the wheel velocity multipliers to correct the sign when the two joint axes
have opposite conventions. The node refuses to start in `joint_states` mode if
the two wheel joint names are missing or identical.
