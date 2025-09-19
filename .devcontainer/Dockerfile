# syntax=docker/dockerfile:1.4
ARG BASE_IMAGE=ros-humble-gz-base:latest
FROM ${BASE_IMAGE}

ARG WORKSPACE_PATH=/workspace/humble_harmonic
ARG GZ_ROS2_CONTROL_WS=/opt/gz_ros2_control_ws
ARG INSTALL_DEV_TOOLS=false
ARG INSTALL_CLAUDE=false
ARG INSTALL_TELEOP=true
ARG INSTALL_TEST_TOOLS=false
ARG ENABLE_GPU_SUPPORT=true
ARG ROS_DOMAIN_ID=0
ARG ROS_LOCALHOST_ONLY=0

# Additional development dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-pip \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# GPU Support Libraries (Optional)
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

# Development Tools (Optional)
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

# Additional ROS packages for development
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    && rm -rf /var/lib/apt/lists/*

# Teleop Tools (Optional)
RUN --mount=type=cache,target=/var/cache/apt \
    if [ "$INSTALL_TELEOP" = "true" ]; then \
    apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# Test Tools (Optional)
RUN --mount=type=cache,target=/var/cache/apt \
    if [ "$INSTALL_TEST_TOOLS" = "true" ]; then \
    apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher-gui \
    && pip3 install pytest pytest-cov && \
    rm -rf /var/lib/apt/lists/*; \
    fi

# Claude AI Assistant (Optional)
RUN --mount=type=cache,target=/var/cache/apt \
    if [ "$INSTALL_CLAUDE" = "true" ]; then \
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g @anthropic-ai/claude-code && \
    rm -rf /var/lib/apt/lists/*; \
    fi

# Workspace Setup
RUN mkdir -p ${WORKSPACE_PATH}

# Environment Configuration (extends base image environment)
RUN echo "" >> ~/.bashrc && \
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