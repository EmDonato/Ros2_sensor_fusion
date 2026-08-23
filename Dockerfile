ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base-jammy AS builder

ARG ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake libeigen3-dev python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY src/ src/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM ros:${ROS_DISTRO}-ros-base-jammy
ARG ROS_DISTRO=humble
ARG USERNAME=ekf
ARG UID=1000
ARG GID=1000


RUN groupadd -g ${GID} ${USERNAME} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME}

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=0 \
    MODULE_SETUP=/opt/robot_module/setup.bash
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /ws/install /opt/robot_module
COPY docker-entrypoint.sh /ros_module_entrypoint.sh
USER ${USERNAME} 
ENTRYPOINT ["/ros_module_entrypoint.sh"]
CMD ["ros2", "run", "ekf", "ekf_node", "--ros-args", "--params-file", "/opt/robot_module/share/ekf/config/ekf.yaml"]
