# author: georgiosmatzarapis

FROM osrf/ros:noetic-desktop-full

ARG USER=rosuser
ARG DEBIAN_FRONTEND=noninteractive
ARG GITHUB_TOKEN

RUN useradd -m -s /bin/bash ${USER} && \
    mkdir -p /home/${USER}/gem_ws/src && \
    chown -R ${USER}:${USER} /home/${USER}

RUN apt-get update && apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    ros-noetic-actionlib-tools \
    ros-noetic-ackermann-msgs \
    python3-pip \
    git \
    terminator && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git /home/${USER}/gem_ws/src/POLARIS_GEM_e2 && \
    git clone https://github.com/georgiosmatzarapis/POLARIS_GEM_e2_MANAGER.git /home/${USER}/gem_ws/src/POLARIS_GEM_e2_MANAGER

# Set-up PurePursuit controller
RUN mv /home/${USER}/gem_ws/src/POLARIS_GEM_e2_MANAGER/path_tracking_controllers/pure_pursuit/pure_pursuit_sim.py \
       /home/${USER}/gem_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/scripts/ && \
    mv /home/${USER}/gem_ws/src/POLARIS_GEM_e2_MANAGER/path_tracking_controllers/pure_pursuit/CMakeLists.txt \
       /home/${USER}/gem_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/ && \
    mv /home/${USER}/gem_ws/src/POLARIS_GEM_e2_MANAGER/path_tracking_controllers/pure_pursuit/package.xml \
       /home/${USER}/gem_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_pure_pursuit_sim/

# Set-up Stanley controller
RUN mv /home/${USER}/gem_ws/src/POLARIS_GEM_e2_MANAGER/path_tracking_controllers/stanley/stanley_sim.py \
       /home/${USER}/gem_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_stanley_sim/scripts/ && \
    mv /home/${USER}/gem_ws/src/POLARIS_GEM_e2_MANAGER/path_tracking_controllers/stanley/CMakeLists.txt \
       /home/${USER}/gem_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_stanley_sim/ && \
    mv /home/${USER}/gem_ws/src/POLARIS_GEM_e2_MANAGER/path_tracking_controllers/stanley/package.xml \
       /home/${USER}/gem_ws/src/POLARIS_GEM_e2/polaris_gem_drivers_sim/gem_stanley_sim/

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /home/${USER}/gem_ws && catkin_make"

RUN echo "source /home/${USER}/gem_ws/devel/setup.bash" >> /home/${USER}/.bashrc

USER ${USER}
WORKDIR /home/${USER}/gem_ws/
