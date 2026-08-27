ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base-jammy

ARG ROS_DISTRO=humble
ARG USERNAME=ekf
ARG UID=1000
ARG GID=1000

SHELL ["/bin/bash", "-c"]

RUN groupadd -g ${GID} ${USERNAME} && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME}

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libeigen3-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY src/ src/
COPY config/ config/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker-entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh && \
    chown -R ${USERNAME}:${USERNAME} /ws

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=0

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc
    
USER ${USERNAME}

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "run", "ekf", "ekf_node", \
     "--ros-args", "--params-file", "/ws/config/ekf.yaml"]