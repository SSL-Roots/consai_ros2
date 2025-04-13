#!/bin/bash
set -e

FILES=$(git diff --cached --name-only --diff-filter=ACM | grep '\.py$' || true)
[ -z "$FILES" ] && exit 0

ament_pep257 $FILES

