FROM osrf/ros:foxy-desktop

# Dev tools
# Try to keep ROS-related dependencies out of here and inside package.xml to be installed with rosdep instead
RUN apt-get -y update && apt-get install -y \
    x11-apps \
    python3-pip \
    build-essential \
    libcanberra-gtk3-module \
    vim \
    nano \
    git \
    wget \
    tmux \
    sudo \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

# Create a non-root user with sudo privileges
ARG USERNAME=guest
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Prepare to build the project
ENV HOMEDIR=/home/${USERNAME}
ENV COLCON_WS=${HOMEDIR}/colcon_ws
ENV SOURCE_DIR=${COLCON_WS}/src/image_path_planning
WORKDIR ${COLCON_WS}


# Copy package xml files to decide dependencies
COPY image_planner/package.xml ${SOURCE_DIR}/image_planner/package.xml
COPY waypoint_generator/package.xml ${SOURCE_DIR}/waypoint_generator/package.xml
COPY controller/package.xml ${SOURCE_DIR}/controller/package.xml

# # Project-specific dependency install
RUN apt-get update && rosdep update && DEBIAN_FRONTED=noninteractive rosdep install --from-paths src --ignore-src -r -y \
    # && apt-get install -y python3-pcl \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
# Build project
COPY . ${SOURCE_DIR}
# RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd /${COLCON_WS}; colcon build --symlink-install'


# Change owner of the files to non-root user
RUN chown -R ${USERNAME} /home/${USERNAME}
# Add source commands to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${USERNAME}/.bashrc
# Change prompt to show we are in a docker container
RUN echo "export PS1='\[\e]0;\u@docker: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@docker\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /home/${USERNAME}/.bashrc
# Add workspace sourcing alias (means you can type 'src' to source the workspace)
RUN echo "alias src='source ~/mcav_ws/install/setup.bash'" >> /home/${USERNAME}/.bashrc
# Add tmux configuration file
COPY docker/.tmux.conf /home/${USERNAME}/.tmux.conf

WORKDIR ${SOURCE_DIR}
CMD /bin/bash
USER $USERNAME