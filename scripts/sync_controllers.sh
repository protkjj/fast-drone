#!/usr/bin/env bash
# ============================================================
# 루트 제어기 소스 → ROS2 패키지 controllers/ 동기화
# ============================================================
# 루트(controller.py 등)는 표준 시뮬용이라 절대 import 사용:
#     from dynamics import ...
# ROS2 패키지의 서브패키지에선 상대 import 필요:
#     from .dynamics import ...
# → symlink 불가. 이 스크립트가 복사 + import 자동 변환.
#
# 사용: 루트 제어기 파일 수정 후 실행
#     bash scripts/sync_controllers.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
DST="$ROOT/ros2_ws/src/fast_drone_ctrl/fast_drone_ctrl/controllers"

# offboard_node.py가 필요로 하는 모듈들
# (Hybrid 실기 도입 시 hybrid_comparison 추가)
MODULES="controller vehicle_params dynamics trim nmpc fallback_controller"

mkdir -p "$DST"
touch "$DST/__init__.py"

echo "동기화: $ROOT → controllers/"
for m in $MODULES; do
    src="$ROOT/$m.py"
    if [ ! -f "$src" ]; then
        echo "  ⚠️  $m.py 루트에 없음 — 건너뜀"
        continue
    fi
    cp "$src" "$DST/$m.py"
    # 로컬 모듈 절대 import → 상대 import (들여쓰기 보존)
    sed -i -E \
      "s/^(\s*)from (${MODULES// /|}) import/\1from .\2 import/" \
      "$DST/$m.py"
    echo "  ✓ $m.py"
done

echo "완료. 다음: 빌드 워크스페이스로 복사 후 colcon build"
echo "  (예: cp -r $ROOT/ros2_ws/src/fast_drone_ctrl ~/px4_drone/ros2_ws/src/ && \\"
echo "        cd ~/px4_drone/ros2_ws && colcon build --packages-select fast_drone_ctrl)"
