#!/bin/sh

xhost +local:
docker run -it \
    --rm \
    --privileged \
    --network host \
    --name moveit_docker \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix:ro \
    -v /etc/localtime:/etc/localtime:ro \
    -v /etc/timezone:/etc/timezone:ro \
    -v ~/.bash_aliases:/root/.bash_aliases \
    -v $(pwd):/root/mounted_dir \
    move_it_panda:latest
