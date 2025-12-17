#!/bin/bash
DOCKER_NAME=simonemiche/manip_approach
TAG=grounding_dino_1.0
cd $PWD
docker build . --build-arg "GIT_USERNAME=$1" --build-arg "GIT_USER_EMAIL=$2" --build-arg "GIT_TOKEN=$3" -t ${DOCKER_NAME}:${TAG} -f Dockerfile
