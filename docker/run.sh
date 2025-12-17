#!/bin/bash
NAME=simonemiche/manip_approach
TAG=grounding_dino_1.0

sudo xhost +
sudo docker run \
     --network=host --privileged \
     -it \
     --rm \
     --gpus all \
     -e DISPLAY=unix${DISPLAY} \
     -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
     --device /dev/dri/card0:/dev/dri/card0 \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v /home/smicheletti-iit.local/rosbags_vlm:/home/user1/rosbags_vlm \
     ${NAME}:${TAG} bash
