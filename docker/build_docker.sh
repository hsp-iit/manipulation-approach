#!/bin/bash         
cd $PWD
docker build . --build-arg "GIT_USERNAME=$1" --build-arg "GIT_USER_EMAIL=$2" --build-arg "GIT_TOKEN=$3" -t simonemiche/manip_approach:grounding_dino_1.0 -f Dockerfile
