#!/usr/bin/env bash

set -e

if [ $# -eq 0 ]; then
    echo "Please set ROS_DISTRO to the argument."
    echo "e.g. ./build.sh humble"
fi
ROS_DISTRO=$1

OPTION=""
if [ $# -ge 2 ]; then
    OPTION=$2
fi

# .dockerディレクトリへ移動する
cd $(dirname $0)/../../

# Dockerfileのビルド
docker build $OPTION -t consai:$ROS_DISTRO-ci -f .docker/ci/Dockerfile . \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
