# syntax=docker/dockerfile:1.4
FROM osrf/ros:humble-desktop-full-jammy AS base

# Build arguments for configuration
ARG WORKSPACE_PATH=/workspace/humble_harmonic
ARG GZ_ROS2_CONTROL_WS=/opt/gz_ros2_control_ws
ARG INSTALL_DEV_TOOLS=false
ARG INSTALL_CLAUDE=false
ARG INSTALL_TELEOP=true
ARG INSTALL_TEST_TOOLS=false
ARG ENABLE_GPU_SUPPORT=true
ARG ROS_DOMAIN_ID=0
ARG ROS_LOCALHOST_ONLY=0

# Labels for metadata
LABEL maintainer="danish.tapia@combatrobotic.in"
LABEL description="ROS 2 Humble + Gazebo Harmonic + gz_ros2_control"
LABEL version="1.1"

# [1/11] Core System Dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    lsb-release \
    gnupg \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# [2/11] GPU Support Libraries (Optional)
RUN --mount=type=cache,target=/var/cache/apt \
    if [ "$ENABLE_GPU_SUPPORT" = "true" ]; then \
    apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# [3/11] Development Tools (Optional)
RUN --mount=type=cache,target=/var/cache/apt \
    if [ "$INSTALL_DEV_TOOLS" = "true" ]; then \
    apt-get update && apt-get install -y \
    vim \
    nano \
    gdb \
    valgrind \
    htop \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# [4/11] Gazebo Harmonic Repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# [5/11] Gazebo Harmonic Installation
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# [6/11] ROS-Gazebo Integration
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    apt-get remove -y ros-humble-ros-gz* || true && \
    apt-get install -y ros-humble-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# [7/11] ROS 2 Control Stack
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-realtime-tools \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# [8/11] Teleop Tools (Optional)
RUN --mount=type=cache,target=/var/cache/apt \
    if [ "$INSTALL_TELEOP" = "true" ]; then \
    apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# [9/11] Test Tools (Optional)
RUN --mount=type=cache,target=/var/cache/apt \
    if [ "$INSTALL_TEST_TOOLS" = "true" ]; then \
    apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher-gui \
    && pip3 install pytest pytest-cov && \
    rm -rf /var/lib/apt/lists/*; \
    fi

# [10/11] gz_ros2_control from Source
RUN mkdir -p ${GZ_ROS2_CONTROL_WS}/src && \
    cd ${GZ_ROS2_CONTROL_WS}/src && \
    git clone https://github.com/ros-controls/gz_ros2_control.git -b humble

RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    libgz-sim8-dev \
    libgz-plugin2-dev \
    libgz-transport13-dev \
    libgz-msgs10-dev \
    libgz-math7-dev \
    libgz-cmake3-dev \
    libgz-common5-dev \
    libgz-tools2-dev \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "cd ${GZ_ROS2_CONTROL_WS} && \
    source /opt/ros/humble/setup.bash && \
    export GZ_VERSION=harmonic && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# [11/11] Claude AI Assistant (Optional)
RUN --mount=type=cache,target=/var/cache/apt \
    if [ "$INSTALL_CLAUDE" = "true" ]; then \
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g @anthropic-ai/claude-code && \
    rm -rf /var/lib/apt/lists/*; \
    fi

# Workspace Setup
RUN mkdir -p ${WORKSPACE_PATH}

# Environment Configuration
RUN echo "# ROS 2 Humble" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Gazebo Harmonic" >> ~/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# gz_ros2_control" >> ~/.bashrc && \
    echo "source ${GZ_ROS2_CONTROL_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Workspace paths" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=${GZ_ROS2_CONTROL_WS}/install/share:${WORKSPACE_PATH}/install/share:${WORKSPACE_PATH}/src:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Source local workspace if it exists" >> ~/.bashrc && \
    echo "if [ -f ${WORKSPACE_PATH}/install/setup.bash ]; then" >> ~/.bashrc && \
    echo "    source ${WORKSPACE_PATH}/install/setup.bash" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# ROS configuration" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> ~/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Display configuration" >> ~/.bashrc && \
    echo "export DISPLAY=\${DISPLAY:-:0}" >> ~/.bashrc && \
    echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc && \
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Build configuration (for reference)" >> ~/.bashrc && \
    echo "export ENABLE_GPU_SUPPORT=${ENABLE_GPU_SUPPORT}" >> ~/.bashrc && \
    echo "export INSTALL_DEV_TOOLS=${INSTALL_DEV_TOOLS}" >> ~/.bashrc && \
    echo "export INSTALL_CLAUDE=${INSTALL_CLAUDE}" >> ~/.bashrc && \
    echo "export INSTALL_TELEOP=${INSTALL_TELEOP}" >> ~/.bashrc && \
    echo "export INSTALL_TEST_TOOLS=${INSTALL_TEST_TOOLS}" >> ~/.bashrc

WORKDIR ${WORKSPACE_PATH}

SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]