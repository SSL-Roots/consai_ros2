#!/bin/bash
set -e

# プロジェクトルートへ移動
cd "$(git rev-parse --show-toplevel)"

FILES=$(git diff --cached --name-only --diff-filter=ACM | grep '\.py$' || true)

if [ -z "$FILES" ]; then
  echo "[ament_black] No Python files to format."
  exit 0
fi

echo "[ament_black] Formatting files:"
echo "$FILES"

ament_black --reformat --config pyproject.toml $FILES