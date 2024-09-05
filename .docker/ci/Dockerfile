ARG ROS_DISTRO="hunble"
FROM ghcr.io/ssl-roots/consai_ros2:${ROS_DISTRO}-light

ENV ROS_OVERLAY /root/ros2_ws
WORKDIR $ROS_OVERLAY

# Copy source files
COPY . src/consai_ros2

# パッケージをビルド
RUN cd $ROS_OVERLAY && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install
