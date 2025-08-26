ARG ARCH
ARG ROS_IMAGE

FROM ${ROS_IMAGE:-osrf/ros:humble-desktop-full-jammy}

ARG INSTALL_VNC=${INSTALL_VNC:-false}
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_INDEX_URL=https://pypi.org/simple

# Installing programs
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN apt-get clean \
    && apt-get update \
    && apt-get install -y \
    apt-utils \
    nano \
    vim \
    python3-pip \
    joystick \
    jstest-gtk \
    evtest \
    ros-humble-rmw-cyclonedds-cpp 
ADD requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && passwd -d $USERNAME \
    && mkdir -p /home/$USERNAME/.config \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo for $USERNAME
RUN \
  echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Add User to dialout group to provide access to serial communication
RUN usermod -aG dialout ${USERNAME}

# Setup working directory
WORKDIR /home/$USERNAME/

# Setup VNC
# ARG INSTALL_VNC
ADD install_vnc.sh /install_vnc.sh
ADD ./fluxbox/styles /home/${USERNAME}/.fluxbox/styles
RUN echo "INSTALL_VNC is $INSTALL_VNC"
RUN if [ "$INSTALL_VNC" = "true" ]; then \
      bash /install_vnc.sh; \
    fi

# Add the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
# inputrc is optional: for command line history search
ADD entrypoint.sh /entrypoint.sh
ADD bashrc /tmp/temp_bashrc.txt
RUN cat /tmp/temp_bashrc.txt >>  /home/${USERNAME}/.bashrc
# Architecture-specific override
RUN if [ $(uname -m) = "aarch64" ]; then \
      echo "source /opt/ros/humble/install/setup.bash" >> /home/ros/.bashrc; \
    else \
      echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc; \
    fi
ADD inputrc /home/${USERNAME}/.inputrc
RUN chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.inputrc 

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]