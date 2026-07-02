FROM docker.io/osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    WORKSPACE_DIR=/opt/benchmark_ws \
    QT_X11_NO_MITSHM=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-yaml \
    python3-vcstool \
    xauth \
    x11-apps \
    mesa-utils \
    libgl1-mesa-dri \
    libglu1-mesa \
    libxkbcommon0 \
    libwayland-client0 \
    wayland-protocols \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true
RUN rosdep update --rosdistro ${ROS_DISTRO}

WORKDIR ${WORKSPACE_DIR}

# Stable, rarely-changed inputs first to maximise cache reuse.

# Pre-populate the Gazebo Fuel model cache so Gazebo doesn't download on every launch.
# GZ_FUEL_CACHE_PATH is set here and in the entrypoint so the path is consistent.
ENV GZ_FUEL_CACHE_PATH=/root/.gz/fuel
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse && \
    gz fuel download -u https://fuel.gazebosim.org/1.0/MovAi/models/pallet_box_mobile && \
    gz fuel download -u https://fuel.gazebosim.org/1.0/MovAi/models/shelf && \
    gz fuel download -u https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big && \
    gz fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey%20Barrier' && \
    gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair && \
    gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/foldable_chair && \
    gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table && \
    gz fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue%20Randy%20Sitting' && \
    gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/MaleVisitorOnPhone && \
    gz fuel download -u 'https://fuel.gazebosim.org/1.0/plateau/models/Casual%20female'"

COPY config config
COPY docker docker
COPY simulation simulation
COPY rviz rviz

# Third-party exploration packages — effectively never change.
COPY exploration_packages/m-explore-ros2 exploration_packages/m-explore-ros2
COPY exploration_packages/nav2_wavefront_frontier_exploration exploration_packages/nav2_wavefront_frontier_exploration

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep install --from-paths exploration_packages simulation rviz --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*"

# Build stable third-party packages first — these layers are almost never invalidated.
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --packages-select \
        explore_lite_msgs explore \
        nav2_wavefront_frontier_exploration"

# Simulation and benchmark rviz package — change occasionally.
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WORKSPACE_DIR}/install/setup.bash && \
    colcon build --symlink-install --packages-select \
        bme_ros2_navigation bme_ros2_navigation_py \
        rviz_autonomous_exploration_benchmark"
    
# ── frontier_exploration_ros2 (frequently changed) ────────────────────────────
COPY exploration_packages/frontier_exploration_ros2 exploration_packages/frontier_exploration_ros2

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep install --from-paths exploration_packages/frontier_exploration_ros2 --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*"

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WORKSPACE_DIR}/install/setup.bash && \
    colcon build --symlink-install --packages-select frontier_exploration_ros2"

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WORKSPACE_DIR}/install/setup.bash && \
    colcon build --symlink-install --packages-select frontier_exploration_ros2_rviz"

# ── vxch (frequently changed) ─────────────────────────────────────────────────
COPY exploration_packages/vxch exploration_packages/vxch

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep install --from-paths exploration_packages/vxch --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*"

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WORKSPACE_DIR}/install/setup.bash && \
    colcon build --symlink-install --packages-select voxelcodec_msgs"

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${WORKSPACE_DIR}/install/setup.bash && \
    colcon build --symlink-install --packages-select voxelcodec_ros"

# Scripts and experiment config go last — changes here don't bust the build cache.
COPY launch.sh launch.sh
COPY launch_real_hardware.sh launch_real_hardware.sh
COPY docker.sh docker.sh
COPY spawn_presets.yaml spawn_presets.yaml
COPY experiment.conf experiment.conf

RUN chmod +x ${WORKSPACE_DIR}/launch.sh ${WORKSPACE_DIR}/docker/entrypoint.sh

ENTRYPOINT ["/opt/benchmark_ws/docker/entrypoint.sh"]
CMD ["./launch.sh"]
