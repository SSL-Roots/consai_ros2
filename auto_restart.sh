#!/bin/bash

# ログにTracebackが出たら関連プロセスを一通りkillして
# 一定時間後に再起動するためのスクリプト
# 実行方法:
#   ./auto_restart.sh

LAUNCH_PGID=""

# kill対象プロセス名の一覧
pkill_targets=(
  "consai_visualizer"
  "component_conta"
  "parameter_publisher"
  "consai_referee_parser"
  "controller.launch.py"
)

cleanup() {
  echo "Interrupted! Cleaning up..."

  if [ -n "$LAUNCH_PGID" ]; then
    echo "Killing process group $LAUNCH_PGID"
    kill -- -"$LAUNCH_PGID" 2>/dev/null
  fi

  echo "Killing residual processes..."
  for target in "${pkill_targets[@]}"; do
    pkill -9 -f "$target"
  done

  exit 0
}

trap cleanup SIGINT

if [ -z ${CONSAI_COLOR} ]; then
    YELLOW="false"
elif [ ${CONSAI_COLOR} = "yellow" ]; then
    YELLOW="true"
elif [ ${CONSAI_COLOR} = "blue" ]; then
    YELLOW="false"
fi

if [ -z ${CONSAI_SIDE} ]; then
    INVERT="false"
elif [ ${CONSAI_SIDE} = "right" ]; then
    INVERT="true"
elif [ ${CONSAI_SIDE} = "left" ]; then
    INVERT="false"
fi

while true; do
  echo "Starting CONSAI ROS2..."

  # launch起動＋リアルタイム出力
  setsid bash -c "ros2 launch consai_game start.launch yellow:=${YELLOW} invert:=${INVERT} 2>&1 | tee ros2_output.log" &
  LAUNCH_PID=$!
  echo "ROS2 leader PID: $LAUNCH_PID"

  LAUNCH_PGID=$(ps -o pgid= "$LAUNCH_PID" | tr -d ' ')
  echo "Captured PGID: $LAUNCH_PGID"

  tail -F ros2_output.log | while read line; do
    echo "$line" | grep -q "Traceback"
    if [ $? -eq 0 ]; then
      echo "Detected Traceback, terminating all child processes..."

      kill -- -"$LAUNCH_PGID"

      echo "Killing residual processes..."
      for target in "${pkill_targets[@]}"; do
        pkill -9 -f "$target"
      done

      break
    fi
  done

  echo "Restarting in 5 seconds..."
  sleep 5
done
