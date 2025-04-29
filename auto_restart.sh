#!/bin/bash

LAUNCH_PGID=""

cleanup() {
  echo "Interrupted! Cleaning up..."

  if [ -n "$LAUNCH_PGID" ]; then
    echo "Killing process group $LAUNCH_PGID"
    kill -- -"$LAUNCH_PGID" 2>/dev/null
  fi

  # 念のため補足的にプロセス名でもkill
  # これやらないとvisualizerとcomponent_contaが残る
  pkill -9 -f "consai_visualizer"
  pkill -9 -f "component_conta"
  pkill -9 -f "parameter_publisher"
  pkill -9 -f "consai_referee_parser"
  pkill -9 -f "controller.launch.py"

  exit 0
}

trap cleanup SIGINT

while true; do
  echo "Starting CONSAI ROS2..."

  # launch起動＋リアルタイム出力
  setsid bash -c 'ros2 launch consai_game start.launch 2>&1 | tee ros2_output.log' &
  LAUNCH_PID=$!
  echo "ROS2 leader PID: $LAUNCH_PID"

  LAUNCH_PGID=$(ps -o pgid= "$LAUNCH_PID" | tr -d ' ')
  echo "Captured PGID: $LAUNCH_PGID"

  # Traceback検知のために並列 tail を使う
  tail -F ros2_output.log | while read line; do
    echo "$line" | grep -q "Traceback"
    if [ $? -eq 0 ]; then
      echo "Detected Traceback, terminating all child processes..."

      kill -- -"$LAUNCH_PGID"

      # 念のため補足的にプロセス名でもkill
      # これやらないとvisualizerとcomponent_contaが残る
      pkill -9 -f "consai_visualizer"
      pkill -9 -f "component_conta"
      pkill -9 -f "parameter_publisher"
      pkill -9 -f "consai_referee_parser"
      pkill -9 -f "controller.launch.py"

      break
    fi
  done

  echo "Restarting in 5 seconds..."
  sleep 5
done
