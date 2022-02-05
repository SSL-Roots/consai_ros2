ARG ROS_DISTRO="foxy"
FROM osrf/ros:$ROS_DISTRO-desktop

ENV ROS_OVERLAY /root/ros2_ws
WORKDIR $ROS_OVERLAY

# CON-SAIパッケージをクローンして
# 依存関係をインストール
ARG CONSAI_BRANCH="main"
ARG FROOTSPI_MSGS_BRANCH="main"
RUN mkdir -p $ROS_OVERLAY/src && \
    cd $ROS_OVERLAY/src && \
    git clone -b $CONSAI_BRANCH https://github.com/SSL-Roots/consai_ros2 && \
    git clone -b $FROOTSPI_MSGS_BRANCH https://github.com/SSL-Roots/consai_frootspi_msgs.git && \
    apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# パッケージをビルド
RUN cd $ROS_OVERLAY && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# スクリプトを書き換え
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
