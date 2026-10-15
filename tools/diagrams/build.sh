#!/usr/bin/env bash
# README 그림(docs/diagrams/*.png)을 이 폴더의 Graphviz .dot 에서 다시 만든다.
# 필요: graphviz(dot), 한글 글꼴 Noto Sans CJK KR.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$here/../../docs/diagrams"
mkdir -p "$out"
for f in "$here"/*.dot; do
  dot -Tpng "$f" -o "$out/$(basename "${f%.dot}").png"
  echo "ok $(basename "$f")"
done
