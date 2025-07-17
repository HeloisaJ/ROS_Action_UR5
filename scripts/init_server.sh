#! usr/bin/bash

#-----------------------------------------------------
# Script generated based on ros2-oranor repository
#-----------------------------------------------------
docker run --rm -it --security-opt seccomp=unconfined \
    --shm-size=512m \
    --privileged \
    --net ursim_net \
    --ip 192.168.56.100 \
    --name ros2_sv \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=42 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    ros2_sv:latest
