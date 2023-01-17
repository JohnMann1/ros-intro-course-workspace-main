#!/bin/bash

docker container prune --force --filter "label=source=ros-course"

chmod +x ../docker/entrypoint.sh

docker image inspect ros-course >/dev/null 2>&1

if [ $? != 0 ]
then
    echo Docker image ros-course not found. Will now build it...
    docker build --no-cache -t ros-course ../docker/
else
    echo Existing Docker image found!
fi

xhost +local:root

docker run -it \
    --net=host \
    --env "DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${PWD}/../workspace":"/root/catkin_ws/src/":rw \
    --name=ros-course \
    --device=/dev/dri:/dev/dri \
    ros-course \
    bash

xhost -local:root