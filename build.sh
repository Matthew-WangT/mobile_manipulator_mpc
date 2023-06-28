#!/bin/bash
docker build -f Dockerfile -t mm_mpc:0.1.0 .
# 
# xhost +
# docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix mm_mpc:0.1.0
