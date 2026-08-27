#!/usr/bin/env python3
"""겹2(프레임 검증)용 Gazebo 월드 생성기 — DESIGN.md 4장.

    python3 gz_aero/tools/gen_gz_test_world.py
    gz sim -s -r --iterations 2000 gz_aero/test/frame_check.world
    python3 gz_aero/tools/check_gz_frames.py

무엇을 검증하나
---------------
겹1 은 "수식을 옳게 옮겼나" 를 gz 없이 확인했다. 남은 건 **좌표 변환**이다:
    월드(ENU) -> 링크(FLU) -> 동체(FRD) -> [코어] -> 링크 -> 월드
부호가 하나 뒤집혀도 시뮬은 멀쩡히 돌기 때문에, 눈으로 봐서는 절대 못 잡는다.

설계
----
* **중력 0, 로터 없음.** 공력만 작용하게 해서 원인을 하나로 좁힌다.
* 플러그인의 `test_velocity_world` 훅으로 **월드 속도를 주입**한다.
  동체 좌표로 주입하면 정작 검증하려던 변환을 건너뛴다.
* 기체를 고정하지 **않는다.** 공력 모멘트로 자유롭게 텀블링하게 두면
  한 번 돌릴 때마다 수백 개의 서로 다른 자세에서 변환이 검증된다.
  대조 스크립트가 **로그에 찍힌 자세**를 쓰므로 기체가 어디로 가든 상관없다.
* 프로브를 여러 개 둔다. 축 하나가 뒤바뀐 오류는 특정 자세에서만 드러나므로
  단순한 자세 하나로는 통과해 버린다.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
for _p in (_REPO, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aero_sources import get_source        # noqa: E402

# (이름, pose "x y z roll pitch yaw", 주입 월드속도, 주입 각속도, 바람)
PROBES = [
    # 0. 가장 단순 — 동쪽으로 순항. 여기서 틀리면 기본 변환부터 틀린 것이다
    ("p0_east",     (0, 0, 20,  0.0,  0.0,  0.0),   (60, 0, 0),  (0, 0, 0),   (0, 0, 0)),
    # 1. 요 90도 — 링크 x 가 월드 북쪽. y/x 뒤바뀜이 여기서 드러난다
    ("p1_yaw90",    (0, 6, 20,  0.0,  0.0,  math.pi/2), (60, 0, 0), (0, 0, 0), (0, 0, 0)),
    # 2. 롤 90도 + 하강 — z/y 뒤바뀜 검출
    ("p2_roll90",   (0, 12, 20, math.pi/2, 0.0, 0.0),  (0, 0, -30), (0, 0, 2), (0, 0, 0)),
    # 3. 완전 비대칭 — 자세 3축, 속도 3축, 각속도 3축이 전부 다르다.
    #    부호 사고 검출기. 대칭 조건만 시험하면 그냥 통과한다
    ("p3_asym",     (0, 18, 20, 0.3, -0.7, 1.1),      (40, 15, -8), (0.5, -1.2, 0.8), (0, 0, 0)),
    # 4. 바람 경로 — 기체는 정지, 순수 측풍. v_air = -wind 가 맞는지 본다
    ("p4_wind",     (0, 24, 20, 0.0,  0.0,  0.0),     (0, 0, 0),   (0, 0, 0),   (0, 25, 0)),
    # 5. 고속 고받음각 — 크로스플로가 지배하는 영역
    ("p5_highalpha",(0, 30, 20, 0.0, -0.6,  0.0),     (83.3, 0, 0), (0, 0, 0),  (0, 0, 0)),
]

WORLD_TEMPLATE = """<?xml version="1.0" ?>
<!-- 자동 생성: gz_aero/tools/gen_gz_test_world.py — 직접 고치지 마세요 -->
<sdf version="1.9">
  <world name="aero_frame_check">
    <!-- 중력 0: 공력만 남긴다. 중력이 있으면 자세가 빨리 무너져 관측 구간이 짧다 -->
    <gravity>0 0 0</gravity>
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
{models}
  </world>
</sdf>
"""

MODEL_TEMPLATE = """
    <model name="{name}">
      <pose>{pose}</pose>
      <link name="body">
        <!-- 링크 원점 = 무게중심. x_cp 가 CG 기준이므로 이렇게 두는 게 맞다.
             관성은 팀 사이징 확정 설계점 값 (장축 = 링크 x = J_xx = 롤) -->
        <inertial>
          <pose>0 0 0 0 0 0</pose>
          <mass>{mass:.9f}</mass>
          <inertia>
            <ixx>{ixx:.10g}</ixx><iyy>{iyy:.10g}</iyy><izz>{izz:.10g}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <visual name="hull">
          <pose>0 0 0 0 {half_pi} 0</pose>
          <geometry><cylinder><radius>{radius:.6g}</radius>
            <length>{length:.6g}</length></cylinder></geometry>
        </visual>
      </link>
      <plugin filename="FastDroneAero" name="fast_drone::AeroPlugin">
        <link_name>body</link_name>
        <csv_file>{csv}</csv_file>
        <nose>1 0 0</nose>
        <right>0 -1 0</right>
        <wind>{wind}</wind>
        <test_velocity_world>{vel}</test_velocity_world>
        <test_omega_world>{omega}</test_omega_world>
        <debug_csv>{log}</debug_csv>
        <debug_csv_every>{every}</debug_csv_every>
      </plugin>
    </model>
"""


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["placeholder", "sized"], default="sized")
    ap.add_argument("--log-dir", default="/tmp/fast_drone_aero",
                    help="플러그인이 디버그 CSV 를 쓸 곳")
    ap.add_argument("--every", type=int, default=5, help="몇 스텝마다 기록할지")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    src = get_source(args.source)
    m = src.meta
    out = args.out or os.path.join(_REPO, "gz_aero", "test", "frame_check.world")
    # 상대경로로 적는다. 플러그인이 SDF 파일 기준으로 풀기 때문에, 이러면
    # 월드 파일이 기계에 종속되지 않아 그대로 커밋할 수 있다.
    csv = os.path.relpath(
        os.path.join(_REPO, "gz_aero", "data", f"aero_{args.source}.csv"),
        os.path.dirname(out))

    mass = m.get("MTOW", 1.660778087)
    ixx = m.get("J_xx", 0.00604071)
    iyy = m.get("J_yy", 0.04045326)
    izz = m.get("J_zz", 0.04045326)
    length = m.get("l_body", 0.72)

    models = []
    for name, pose, vel, omega, wind in PROBES:
        models.append(MODEL_TEMPLATE.format(
            name=name,
            pose=" ".join(f"{x:.9g}" for x in pose),
            mass=mass, ixx=ixx, iyy=iyy, izz=izz,
            radius=src.d_ref / 2.0, length=length, half_pi=math.pi / 2,
            csv=csv,
            wind=" ".join(f"{x:.9g}" for x in wind),
            vel=" ".join(f"{x:.9g}" for x in vel),
            omega=" ".join(f"{x:.9g}" for x in omega),
            log=os.path.join(args.log_dir, f"{name}.csv"),
            every=args.every))

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(WORLD_TEMPLATE.format(models="".join(models)))

    print(f"{out}")
    print(f"  소스     {args.source}  (표 {csv})")
    print(f"  프로브   {len(PROBES)}개: {', '.join(p[0] for p in PROBES)}")
    print(f"  로그     {args.log_dir}/<프로브>.csv  (매 {args.every} 스텝)")
    print()
    print("Ubuntu 에서:")
    print(f"  mkdir -p {args.log_dir}")
    print(f"  export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/gz_aero/build:$GZ_SIM_SYSTEM_PLUGIN_PATH")
    print(f"  gz sim -s -r --iterations 2000 {out}")
    print(f"  python3 gz_aero/tools/check_gz_frames.py --source {args.source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
