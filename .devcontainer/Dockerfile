ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Install apt packages
RUN sudo apt update && sudo apt install -y \
    curl \
    # grSim dependencies
    git build-essential cmake pkg-config qtbase5-dev \
    libqt5opengl5-dev libgl1-mesa-dev libglu1-mesa-dev \
    libprotobuf-dev protobuf-compiler libode-dev libboost-dev \
    && sudo apt clean \
    && sudo rm -rf /var/lib/apt/lists/*

# Install ssl-game-controller
RUN cd /home/${USERNAME} \
    && curl -L -o ssl-game-controller https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.8.0/ssl-game-controller_v3.8.0_linux_amd64 \
    && chmod +x ssl-game-controller

# Install grSim
RUN cd /home/${USERNAME}/ && \
    git clone https://github.com/RoboCup-SSL/grSim.git \
    && cd grSim && mkdir build && cd build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. \
    && make && sudo make install

# Create ROS2 Workspace
RUN mkdir -p /home/${USERNAME}/ros2_ws/src

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> ~/.bashrc

# Copy source files
COPY . /home/${USERNAME}/ros2_ws/src/consai_ros2

# Install CONSAI dependent packages
RUN cd /home/${USERNAME}/ros2_ws/src \
    && git clone https://github.com/SSL-Roots/consai_frootspi_msgs.git \
    && git clone https://github.com/SSL-Roots/frootspi_msgs.git \
    && sudo apt update \
    && rosdep update  \
    && rosdep install -r -y -i --from-paths .

# Build the workspace
RUN cd /home/${USERNAME}/ros2_ws \
    && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install