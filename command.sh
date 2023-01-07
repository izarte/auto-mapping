#/bin/bash

apt update
rosdep install -q -y -r --from-paths src --ignore-src
