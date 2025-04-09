#!/bin/bash
set -e

# ROS 2 環境が読み込まれていることが前提！
FILES=$(git diff --cached --name-only --diff-filter=ACM | grep -E '\.(cpp|hpp|c|h)$' || true)

if [ -z "$FILES" ]; then
  echo "[ament_cpplint] No C/C++ files to check."
  exit 0
fi

echo "[ament_cpplint] Running on files:"
echo "$FILES"

ament_cpplint $FILES

