#!/bin/bash
set -ex
docker run -v $PWD/shared:/shared -e DISPLAY=":0" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"  -p 19997:19997 -it coppeliasim-ubuntu18 "$@"
