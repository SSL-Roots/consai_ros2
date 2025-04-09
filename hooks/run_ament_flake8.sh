#!/bin/bash
set -e

# Pythonファイルを渡されたものだけに絞る
FILES=$(git diff --cached --name-only --diff-filter=ACM | grep '\.py$' || true)
[ -z "$FILES" ] && exit 0

ament_flake8 $FILES

