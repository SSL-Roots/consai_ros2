#!/bin/bash

# --- 使い方 ---
# ./record_rosbag.sh /path/to/save/rosbag

# --- 保存先ディレクトリ（引数） ---
if [ -z "$1" ]; then
  echo "Usage: $0 <output_directory>"
  exit 1
fi

OUTPUT_BASE="$1"
TIMESTAMP=$(date +%Y%m%d)
BAG_DIR="${OUTPUT_BASE}/${TIMESTAMP}"

mkdir -p "$BAG_DIR"
cd $BAG_DIR
echo "[INFO] Saving rosbag to: $BAG_DIR"

# --- 除外したいトピックリスト ---
EXCLUDE_TOPICS=(
  "/tf"
  "/tf_static"
  "/diagnostics"
)

# --- 除外オプションを構築 ---
EXCLUDE_ARGS=()
for topic in "${EXCLUDE_TOPICS[@]}"; do
  EXCLUDE_ARGS+=("--exclude" "$topic")
done

# --- 実行コマンド表示（確認用） ---
echo "[INFO] Executing: ros2 bag record -a --output  ${EXCLUDE_ARGS[*]}"

# --- 実行 ---
ros2 bag record -a "${EXCLUDE_ARGS[@]}"
