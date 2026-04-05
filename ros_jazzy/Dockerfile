FROM osrf/ros:jazzy-desktop

# Arguments for non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# Install essential tools, ROS-Gazebo, and your specific hardware/SLAM packages
RUN apt-get update && apt-get install -y \
    sudo curl wget gnupg2 lsb-release \
    x11-apps x11-xserver-utils xauth \
    software-properties-common \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-dynamixel-sdk \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-turtlebot3-teleop \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
 && rm -rf /var/lib/apt/lists/*

# Create or reuse non-root user and grant passwordless sudo
RUN set -eux; \
 if getent group "${USER_GID}" >/dev/null; then \
   existing_group="$(getent group "${USER_GID}" | cut -d: -f1)"; \
   if [ "${existing_group}" != "${USERNAME}" ]; then \
     groupmod -n "${USERNAME}" "${existing_group}"; \
   fi; \
 else \
   groupadd -g "${USER_GID}" "${USERNAME}"; \
 fi; \
 if getent passwd "${USER_UID}" >/dev/null; then \
   existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
   if [ "${existing_user}" != "${USERNAME}" ]; then \
     usermod -l "${USERNAME}" -g "${USER_GID}" -d "/home/${USERNAME}" -m "${existing_user}"; \
   else \
     usermod -g "${USER_GID}" -d "/home/${USERNAME}" -m "${USERNAME}"; \
   fi; \
 else \
   useradd -m -u "${USER_UID}" -g "${USER_GID}" "${USERNAME}"; \
 fi; \
 usermod -aG sudo "${USERNAME}"; \
 echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" >/etc/sudoers.d/99-${USERNAME}-nopasswd; \
 chmod 0440 /etc/sudoers.d/99-${USERNAME}-nopasswd

# Ensure a valid XDG runtime directory for Qt-based apps
RUN mkdir -p /run/user/${USER_UID} && \
    chown ${USERNAME}:${USERNAME} /run/user/${USER_UID} && \
    chmod 700 /run/user/${USER_UID}
ENV XDG_RUNTIME_DIR=/run/user/${USER_UID}

# Switch to non-root user and set up environment
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Automatically source ROS setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc