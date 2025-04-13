#!/bin/bash
set -e

# プロジェクトルートへ移動（設定ファイルが見えるように）
cd "$(git rev-parse --show-toplevel)"

# ステージ済みの Python ファイルのみ取得
FILES=$(git diff --cached --name-only --diff-filter=ACM | grep '\.py$' || true)

if [ -z "$FILES" ]; then
  echo "[ament_pep257] No Python files to check."
  exit 0
fi

echo "[ament_pep257] Running on files:"
echo "$FILES"

# 設定ファイル（setup.cfg など）を使って ament_pep257 を実行
ament_pep257 $FILES
