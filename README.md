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

Prediction intervals are integrated in steps no larger than the smaller of
`nominal_dt` and `max_dt`. The velocity dynamics use the exact exponential
discretization of the configured first-order model, so their response does not
depend on how sensor callbacks divide the same time interval.

Measurements older than the current filter time by no more than
`out_of_sequence_tolerance` are accepted as near-simultaneous updates. This
absorbs small scheduling and transport reorderings without propagating the
filter backwards; measurements outside the tolerance are still rejected.

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

Clock selection: set `use_sim_time` in this module’s ROS parameter YAML.
For standalone use, `false` uses the system clock and `true` subscribes to `/clock`.
When launched by Open-PiRobot_simulation, `config/simulation.yaml` →
`runtime.use_sim_time` overrides module YAML values; `null` preserves them.

The `src/sensor_fusion_bringup` ROS 2 package installs `sensor_fusion.launch.py`:

```bash
ros2 launch sensor_fusion_bringup sensor_fusion.launch.py
ros2 launch sensor_fusion_bringup sensor_fusion.launch.py use_sim_time:=true
```

`config_file` defaults to the installed `config/ekf.yaml`. An explicit
`use_sim_time:=true|false` overrides that YAML; the default `config` preserves it.
The simulation workspace calls this module's `sensor_fusion.sh` script to start its
own container. `PIROBOT_EKF_CONFIG` can select a temporary ROS parameter file for
the container; standalone runs use `config/ekf.yaml`.
