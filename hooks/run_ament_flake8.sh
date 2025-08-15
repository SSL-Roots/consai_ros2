#!/bin/bash
set -e

# プロジェクトルートへ移動（どこから実行されても .cfg が見えるように）
cd "$(git rev-parse --show-toplevel)"

# 現在の設定を表示
echo "Current configuration:"
cat setup.cfg

# ステージ済みの Python ファイルのみ取得
FILES=$(git diff --cached --name-only --diff-filter=ACM | grep '\.py$' || true)

if [ -z "$FILES" ]; then
  echo "[ament_flake8] No Python files to check."
  exit 0
fi

echo "[ament_flake8] Running on files:"
echo "$FILES"

ament_flake8 --config setup.cfg $FILES
