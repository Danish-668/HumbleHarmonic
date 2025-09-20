# syntax=docker/dockerfile:1.4
ARG BASE_IMAGE=ros-humble-gz-base:latest
FROM ${BASE_IMAGE}

ARG WORKSPACE_PATH=/workspace/humble_harmonic
ARG GZ_ROS2_CONTROL_WS=/opt/gz_ros2_control_ws
ARG INSTALL_CLAUDE=false
ARG ROS_DOMAIN_ID=0
ARG ROS_LOCALHOST_ONLY=0

# Copy packages list
COPY packages.txt /tmp/packages.txt

# Install all packages from packages.txt
# Simply comment/uncomment lines in packages.txt to control what gets installed
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive grep -v '^#' /tmp/packages.txt | grep -v '^$' | \
    xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/packages.txt

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
    echo "export INSTALL_CLAUDE=${INSTALL_CLAUDE}" >> ~/.bashrc

WORKDIR ${WORKSPACE_PATH}

SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]