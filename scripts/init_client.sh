#! usr/bin/bash

#-----------------------------------------------------
# Script generated based on ros2-oranor repository
#-----------------------------------------------------
docker run --rm -it --security-opt seccomp=unconfined \
    --shm-size=512m \
    --net ursim_net \
    --ip 192.168.56.90 \
    --name ros2_cl \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=42 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    ros2_cl:latest
