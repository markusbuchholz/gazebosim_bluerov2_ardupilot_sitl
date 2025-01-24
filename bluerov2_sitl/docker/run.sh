#!/usr/bin/env bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

local_gz_ws="/home/markus/underwater/gazebosim_bluerov2_ardupilot_sitl/gz_ws"
local_SITL_Models="/home/markus/underwater/gazebosim_bluerov2_ardupilot_sitl/SITL_Models"
#-v "$local_gz_ws:/home/blueboat_sitl/gz_ws" \
#-v "$local_SITL_Models:/home/blueboat_sitl/SITL_Models" \
# Running the Docker container with new volume mounts
docker run -it \
    --rm \
    --name bluerov2_sitl \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "$local_gz_ws:/home/bluerov2_sitl/gz_ws" \
    -v "$local_SITL_Models:/home/bluerov2_sitl/SITL_Models" \
    --device /dev/dri/card1:/dev/dri/card1 \
    --device /dev/dri/card1:/dev/dri/card2 \
    --privileged \
    --security-opt seccomp=unconfined \
    --network host \
    --gpus all \
    bluerov2_sitl:latest





