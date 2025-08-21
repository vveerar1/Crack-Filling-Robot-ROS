ARG ARCH
ARG ROS_IMAGE

FROM ${ROS_IMAGE:-osrf/ros:humble-desktop-full-jammy}

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
    evtest \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --index-url https://pypi.org/simple -r /tmp/requirements.txt

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

WORKDIR /home/$USERNAME/

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Add User to dialout group to provide access to serial communication
RUN usermod -aG dialout ${USERNAME}

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
# inputrc is optional: for command line history search
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /tmp/temp_bashrc.txt
RUN cat /tmp/temp_bashrc.txt >>  /home/${USERNAME}/.bashrc
# Architecture-specific override
RUN if [ $(uname -m) = "aarch64" ]; then \
      echo "source /opt/ros/humble/install/setup.bash" >> /home/ros/.bashrc; \
    else \
      echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc; \
    fi
COPY inputrc /home/${USERNAME}/.inputrc 

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]