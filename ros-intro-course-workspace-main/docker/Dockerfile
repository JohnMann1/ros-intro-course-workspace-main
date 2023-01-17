# Start with ros noetic image
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# Install some useful stuff
RUN apt-get update && apt-get install -y \
    locales \
    lsb-release \
    cmake \
    g++ \
    git \
    gnupg gnupg1 gnupg2 \
    libcanberra-gtk* \
    python3-catkin-tools \
    python3-pip \
    python3-tk \
    wget \
    vim \
    emacs \
    nano \
    dos2unix \
    python3.8 \ 
    python-is-python3 \ 
    evince

# Install ros packages for TurtleBot3
RUN apt-get update && apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
  ros-noetic-turtle-tf2 ros-noetic-tf2-tools ros-noetic-tf

RUN apt install -y ros-noetic-dynamixel-sdk
RUN apt install -y ros-noetic-turtlebot3-msgs
RUN apt install -y ros-noetic-turtlebot3

RUN mkdir -p ~/catkin_ws/src \
    && source /opt/ros/noetic/setup.bash \
    && cd ~/catkin_ws/src \
    && catkin_init_workspace \
    && cd ~/catkin_ws \
    && source /opt/ros/noetic/setup.bash \
    && catkin_make

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
RUN source ~/.bashrc

RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
ENV NO_AT_BRIDGE 1

LABEL source=ros-course

VOLUME /root/catkin_ws/src

WORKDIR /root/catkin_ws
COPY ./entrypoint.sh /
RUN dos2unix /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
