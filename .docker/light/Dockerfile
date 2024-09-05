ARG ROS_DISTRO="humble"
FROM ros:$ROS_DISTRO-ros-base

ENV ROS_OVERLAY /root/ros2_ws
WORKDIR $ROS_OVERLAY/src

# Copy source files
COPY . consai_ros2

# CON-SAIパッケージをクローンして
# 依存関係をインストール
RUN git clone -b main https://github.com/SSL-Roots/consai_frootspi_msgs.git && \
    git clone -b main https://github.com/SSL-Roots/frootspi_msgs.git && \
    apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# パッケージをビルド
RUN cd $ROS_OVERLAY && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# スクリプトを書き換え
WORKDIR $ROS_OVERLAY
RUN sed --in-place --expression \
    '$i if [ -e $ROS_OVERLAY/install/setup.bash ]; then\n\tsource "$ROS_OVERLAY/install/setup.bash" \nfi' \
    /ros_entrypoint.sh
