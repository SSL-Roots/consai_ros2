#!/bin/bash
set -e

# Git のインデックスに登録された C/C++ ファイルを抽出する
FILES=$(git diff --cached --name-only --diff-filter=ACM | grep -E '\.(cpp|hpp|c|h)$' || true)

if [ -z "$FILES" ]; then
  echo "[ament_uncrustify] チェック対象の C/C++ ファイルがありません。"
  exit 0
fi

echo "[ament_uncrustify] 以下のファイルをチェックします:"
echo "$FILES"

# ファイルごとに ament_uncrustify のチェックを実行
for file in $FILES; do
    echo "[ament_uncrustify] チェック中: $file"
    # --check オプションでフォーマット違反を検出
    # ament_uncrustify --check "$file"
    ament_uncrustify "$file"
done
