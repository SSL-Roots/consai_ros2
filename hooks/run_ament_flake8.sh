#!/bin/bash
set -e

# プロジェクトルートへ移動（どこから実行されても .cfg が見えるように）
cd "$(git rev-parse --show-toplevel)"

# ステージ済みの Python ファイルのみ取得
FILES=$(git diff --cached --name-only --diff-filter=ACM | grep '\.py$' || true)

if [ -z "$FILES" ]; then
  echo "[ament_flake8] No Python files to check."
  exit 0
fi

echo "[ament_flake8] Running on files:"
echo "$FILES"

# 設定ファイル (setup.cfg or tox.ini) を自動検出し、flake8 を実行
ament_flake8 --config setup.cfg $FILES
