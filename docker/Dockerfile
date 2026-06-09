# ROS distribution to use
ARG ROS_DISTRO=jazzy

#######################################
# Base Image — ROS + Nav2 sim deps      #
#######################################
FROM ros:${ROS_DISTRO} AS base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install basic apt packages (prefer apt over pip for Python deps in ROS containers)
RUN apt-get update && apt-get install -y --no-install-recommends \
 curl git libcanberra-gtk-module libcanberra-gtk3-module fuse3 libfuse2 libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine \
 python3-matplotlib python3-transforms3d

# Match host 'render' group GID for GPU access via Docker (avoids "cannot find name for group ID 110")
RUN groupadd -g 110 render 2>/dev/null || true

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# OR-Tools prebuilt: amd64 ubuntu-24.04; arm64 AlmaLinux aarch64 (Apple Silicon Docker).
ARG TARGETARCH
ARG ORTOOLS_VERSION=9.12.4544
COPY docker/install-ortools.sh /tmp/install-ortools.sh
RUN chmod +x /tmp/install-ortools.sh \
 && ORTOOLS_VERSION=${ORTOOLS_VERSION} TARGETARCH=${TARGETARCH} /tmp/install-ortools.sh \
 && rm /tmp/install-ortools.sh
ENV CMAKE_PREFIX_PATH="/opt/ortools:${CMAKE_PREFIX_PATH}"
ENV LD_LIBRARY_PATH="/opt/ortools/lib:${LD_LIBRARY_PATH}"

# Create Colcon workspace with external dependencies
RUN mkdir -p /turtlebot_ws/src
WORKDIR /turtlebot_ws/src
COPY dependencies.repos .
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /turtlebot_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install
ENV TURTLEBOT_MODEL=3

# Optional Groot2 BT editor (x86_64 AppImage only).
WORKDIR /root/
COPY docker/install-groot2.sh /tmp/install-groot2.sh
RUN chmod +x /tmp/install-groot2.sh \
 && TARGETARCH=${TARGETARCH} /tmp/install-groot2.sh /root/Groot2.AppImage \
 && rm /tmp/install-groot2.sh

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
WORKDIR /turtlebot_ws
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

##########################################
# Overlay Image — GPR coverage workspace   #
##########################################
FROM base AS overlay

# Create an overlay Colcon workspace with GPR packages baked in for CI/images.
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./gpr_common/ ./src/gpr_common/
COPY ./gpr_msgs/ ./src/gpr_msgs/
COPY ./gpr_metrics/ ./src/gpr_metrics/
COPY ./gpr_perception/ ./src/gpr_perception/
COPY ./gpr_planning/ ./src/gpr_planning/
COPY ./gpr_control/ ./src/gpr_control/
COPY ./gpr_mission/ ./src/gpr_mission/
COPY ./gpr_bringup/ ./src/gpr_bringup/
COPY docker/verify-ortools-build.sh /tmp/verify-ortools-build.sh
RUN chmod +x /tmp/verify-ortools-build.sh \
 && source /turtlebot_ws/install/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install \
 && /tmp/verify-ortools-build.sh \
 && rm /tmp/verify-ortools-build.sh

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#####################
# Development Image #
#####################
FROM overlay AS dev

# Dev container arguments
ARG USERNAME=devuser
ARG UID=1000
ARG GID=${UID}
ARG TARGETARCH

# Docs toolchain: mmdc + pandoc/xelatex + md-to-pdf. Browser install is arch-specific.
RUN apt-get update && apt-get install -y --no-install-recommends \
 gdb gdbserver nano \
 nodejs npm \
 pandoc \
 lmodern \
 texlive-xetex \
 texlive-latex-recommended \
 texlive-latex-extra \
 texlive-fonts-recommended \
 && npm install -g @mermaid-js/mermaid-cli \
 && rm -rf /var/lib/apt/lists/*
COPY docker/install-chromium.sh /tmp/install-chromium.sh
RUN chmod +x /tmp/install-chromium.sh \
 && TARGETARCH=${TARGETARCH} /tmp/install-chromium.sh \
 && rm /tmp/install-chromium.sh

# In Ubuntu 24.04, chromium-browser apt is a snap stub on the host OS; install-chromium.sh
# uses Google Chrome on amd64 and apt Chromium on arm64 inside the container.

# In Ubuntu 24.04, there is already a user named "ubuntu" with UID 1000.
# Delete this in the (common) event that the user on the host also has this UID.
RUN touch /var/mail/ubuntu \
    && chown ubuntu /var/mail/ubuntu \
    && userdel -r ubuntu

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /overlay_ws/

# Move Groot2 when present (amd64 images only).
RUN groupadd fuse \
 && usermod -aG fuse,render ${USERNAME}
RUN if [[ -f /root/Groot2.AppImage ]]; then \
      mv /root/Groot2.AppImage /home/${USERNAME}/ \
      && chown ${UID}:${GID} /home/${USERNAME}/Groot2.AppImage; \
    fi

# Set up the entrypoint, including it in the .bashrc for interactive shells
USER ${USERNAME}
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc
