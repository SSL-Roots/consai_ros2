#!/usr/bin/env bash

set -e

OPTION=""
if [ $# -eq 1 ]; then
    OPTION=$1
fi

# .dockerディレクトリへ移動する
cd $(dirname $0)/

# Dockerfileのビルド
docker build $OPTION -t consai:latest .