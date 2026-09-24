#!/usr/bin/env bash
# 겹1 검증 한 방 실행 — Gazebo 없이 돈다 (macOS/Ubuntu 둘 다).
#
#   ./gz_aero/verify.sh              두 소스 다
#   ./gz_aero/verify.sh sized        하나만
#
# 하는 일: CSV 표 생성 -> 기준값/스윕 생성 -> C++ 테스트 빌드 -> 대조
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

SOURCES=("${@:-sized placeholder}")
read -r -a SOURCES <<< "${SOURCES[*]}"

BUILD_DIR="${TMPDIR:-/tmp}/fast_drone_gz_aero"
mkdir -p "$BUILD_DIR"
BIN="$BUILD_DIR/test_aero_core"

echo "▶ C++ 테스트 빌드"
c++ -std=c++17 -O2 -Wall -Wextra \
    -I gz_aero/include gz_aero/test/test_aero_core.cc -o "$BIN"

FAILED=0
for src in "${SOURCES[@]}"; do
  echo
  echo "════════════════════════ $src ════════════════════════"
  PYTHONPATH="$REPO" python3 gz_aero/tools/gen_aero_csv.py  --source "$src"
  echo
  PYTHONPATH="$REPO" python3 gz_aero/tools/gen_reference.py --source "$src" | tail -3
  echo
  "$BIN" "gz_aero/data/aero_${src}.csv" \
         "gz_aero/test/aero_reference_${src}.csv" \
         "gz_aero/test/aero_interp_${src}.csv" || FAILED=1
done

echo
if [ "$FAILED" -eq 0 ]; then echo "✅ 전부 통과"; else echo "❌ 실패 있음"; fi
exit "$FAILED"
