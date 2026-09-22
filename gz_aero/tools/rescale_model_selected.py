#!/usr/bin/env python3
"""fast_missile_base 를 선정안(selected-6931, 1.7117 kg) 스케일로 맞춘다 (되돌릴 수 있게).

    python3 gz_aero/tools/rescale_model_selected.py --model <경로>
    python3 gz_aero/tools/rescale_model_selected.py --model <경로> --restore

전제: `fix_model_rocket.py`가 이미 돈 상태(로터가 yz평면·조인트축 기수방향인
"로켓형" 레이아웃, 동체 visual이 fuselage/nose/nose_tip/tail 4개 원기둥/구
프리미티브)라고 가정한다. 그 스크립트는 관성·질량·공력 플러그인은 일부러
안 건드렸다 — 그 시점엔 `control/vehicle_params.py`의 8kg 플레이스홀더
그대로였다. 이 스크립트가 그 나머지(질량·관성·로터 암 반경·동체 형상)를
research/profiles/selected.json(선정안 CSV 사이징, 1.7117 kg)로 바꾼다.

바꾸는 것:
  1. base_link의 <inertial> 질량·관성을 selected.json 값으로.
  2. 로터 4개의 위치를 새 암 반경(radial_arm_m/√2)으로 재조정. 배치 패턴
     (yz평면, 대각 교대 회전방향)은 fix_model_rocket.py가 이미 맞혀놨으므로
     안 건드리고 반경만 스케일한다.
  3. 동체 visual(원기둥/구 4개)을 gz_aero/meshes/body.stl(단일 메시,
     CG 기준·+x 기수 방향으로 이미 맞춰져 있음, pose 항등)로 교체.
  4. base_link에 motor_mount.stl 4개(정적 모터 마운트) visual 추가 — 로터
     허브 위치에, base_link 자체가 모델 프레임이라 pose 회전 없이 그대로.
  5. 각 rotor_N 링크에 motor_bell.stl(회전하는 모터벨) visual 추가 — 링크
     자체가 fix_model_rocket.py에 의해 Y축 -90도 회전돼 있으므로, 메시가
     (동체좌표계 기준으로 만들어져 있어) 보이는 방향이 안 틀어지게 visual
     pose에 보정 회전 Y축 +90도를 넣는다.

바꾸지 않는 것: 충돌체(collision) 지오메트리는 그대로 둔다 — 현재 실제
SDF의 충돌체 구조를 이 저장소에서 확인할 수 없어서, 잘못 추측해 고치는
것보다는 미기재로 남기는 쪽을 택했다. 프로펠러 블레이드(1345_prop_*.stl)도
이 CAD에 안 들어있어 기존 것을 그대로 쓴다 — 안 건드림. 공력 플러그인
블록도 안 건드림.

⚠ 검증 못한 부분: 4번·5번의 pose 회전 부호는 macOS에서 Gazebo를 못 띄워서
직접 확인 못 했다 — 좌표계 변환은 research/stl.js의 parse()와 같은 공식
(CAD mm, 기수 X=0, +X 뒤 -> 동체 FRD: [cgFromNose-x, y, -z] m)을 손으로
맞춘 것이다. gz sim으로 띄워서 모터 마운트/벨이 뒤집히거나 90도 돌아가
보이면, MOTOR_VISUAL_ROT의 부호를 반대로(-1.5707963) 바꿔서 다시 돌릴 것.
"""
import argparse
import math
import pathlib
import re
import shutil
import sys

# research/profiles/selected.json 값 그대로 (2026-09-22 gz_aero/meshes export 때와 동일).
MASS_KG = 1.7117140755237128
IXX, IYY, IZZ = 0.007651084881527969, 0.0349665345797696, 0.0349665345797696
CG_FROM_NOSE_M = 0.4282511278030152
ARM_HALF_M = 0.11938790893553698          # radial_arm_m/sqrt(2)
ROTOR_HUB_X = CG_FROM_NOSE_M - 0.611       # gz_aero/meshes/README.md와 동일 공식/값
PI2 = "1.5707963"

NAMES = ("fuselage", "nose", "nose_tip", "tail")
# fix_model_rocket.py의 ROTOR_YZ/ROTOR_DIR과 같은 순서(사분면 대각 교대) -
# 배치 패턴은 그대로, 반경만 새 값으로.
ROTOR_YZ = [(ARM_HALF_M, ARM_HALF_M), (-ARM_HALF_M, ARM_HALF_M),
            (-ARM_HALF_M, -ARM_HALF_M), (ARM_HALF_M, -ARM_HALF_M)]


def alloc_rank(yz, kQ_over_kT=5e-6 / 6e-5):
    """fix_model_rocket.py와 같은 특이성 확인 - 반경만 바뀌므로 통과해야 정상이다."""
    dirs = [1, -1, 1, -1]
    A = [[1.0] * 4,
         [dirs[i] * kQ_over_kT for i in range(4)],
         [yz[i][1] for i in range(4)],
         [-yz[i][0] for i in range(4)]]
    M = [row[:] for row in A]
    det = 1.0
    for c in range(4):
        piv = max(range(c, 4), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-12:
            return 0.0
        if piv != c:
            M[c], M[piv] = M[piv], M[c]
            det = -det
        det *= M[c][c]
        for r in range(c + 1, 4):
            f = M[r][c] / M[c][c]
            for k in range(c, 4):
                M[r][k] -= f * M[c][k]
    return det


def block(name):
    return re.compile(r'[ \t]*<visual name="' + re.escape(name) + r'">'
                      r'[\s\S]*?</visual>')


MESH_BODY_VISUAL = """      <visual name="body_cad">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh><uri>model://fast_missile_base/meshes/body.stl</uri></mesh>
        </geometry>
        <material>
          <ambient>0.03 0.03 0.05 1</ambient>
          <diffuse>0.07 0.08 0.11 1</diffuse>
          <specular>0.35 0.35 0.4 1</specular>
        </material>
      </visual>"""


def mount_visual(index, py, pz):
    return (f'      <visual name="motor_mount_{index}">\n'
            f'        <pose>{ROTOR_HUB_X:.8f} {py:.8f} {pz:.8f} 0 0 0</pose>\n'
            f'        <geometry>\n'
            f'          <mesh><uri>model://fast_missile_base/meshes/motor_mount.stl</uri></mesh>\n'
            f'        </geometry>\n'
            f'      </visual>')


def bell_visual_block(index):
    # 링크 자체가 Y축 -90도 회전(fix_model_rocket.py) - 메시가 동체좌표계
    # 기준으로 만들어져 있으므로 보정으로 Y축 +90도를 준다.
    return (f'      <visual name="motor_bell_{index}">\n'
            f'        <pose>0 0 0 0 {PI2} 0</pose>\n'
            f'        <geometry>\n'
            f'          <mesh><uri>model://fast_missile_base/meshes/motor_bell.stl</uri></mesh>\n'
            f'        </geometry>\n'
            f'      </visual>\n')


def replace_inertial(text):
    """base_link의 <inertial> 블록 안 mass/ixx/iyy/izz만 값 교체."""
    m = re.search(r'<link name="base_link">[\s\S]*?<inertial>([\s\S]*?)</inertial>', text)
    if not m:
        return text, False
    inertial = m.group(1)

    def sub_tag(src, tag, value):
        pat = re.compile(r'(<' + tag + r'>)[^<]*(</' + tag + r'>)')
        if not pat.search(src):
            return src, False
        return pat.sub(lambda mm: mm.group(1) + repr(value) + mm.group(2), src, count=1), True

    new_inertial, ok_mass = sub_tag(inertial, "mass", MASS_KG)
    changed = ok_mass
    for tag, value in (("ixx", IXX), ("iyy", IYY), ("izz", IZZ)):
        new_inertial, ok = sub_tag(new_inertial, tag, value)
        changed = changed and ok
    if not changed:
        return text, False
    start, end = m.span(1)
    return text[:start] + new_inertial + text[end:], True


def replace_rotor_positions(text):
    n = 0
    for i, (py, pz) in enumerate(ROTOR_YZ):
        pat = re.compile(r'(<link name="rotor_' + str(i) + r'">[\s\S]*?<pose>)'
                         r'[^<]*(</pose>)')
        text, k = pat.subn(
            lambda m: m.group(1) + f"{ROTOR_HUB_X:.8f} {py:.8f} {pz:.8f} 0 -{PI2} 0" + m.group(2),
            text, count=1)
        n += k
    return text, n


def insert_mount_visuals(text):
    """base_link의 </link> 바로 앞에 motor_mount 4개 visual을 끼워 넣는다."""
    inserts = "\n".join(mount_visual(i, py, pz) for i, (py, pz) in enumerate(ROTOR_YZ))
    pat = re.compile(r'(<link name="base_link">[\s\S]*?)(\n[ \t]*</link>)')
    new_text, n = pat.subn(lambda m: m.group(1) + "\n" + inserts + m.group(2), text, count=1)
    return new_text, n


def insert_bell_visuals(text):
    n = 0
    for i in range(4):
        pat = re.compile(r'(<link name="rotor_' + str(i) + r'">[\s\S]*?)(\n[ \t]*</link>)')
        text, k = pat.subn(lambda m: m.group(1) + "\n" + bell_visual_block(i) + m.group(2),
                           text, count=1)
        n += k
    return text, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="fast_missile_base/model.sdf")
    ap.add_argument("--restore", action="store_true", help=".bak 에서 되돌린다")
    a = ap.parse_args()

    path = pathlib.Path(a.model)
    if not path.is_file():
        print(f"model.sdf 가 없습니다: {path}", file=sys.stderr)
        return 1
    bak = path.with_suffix(path.suffix + ".selected.bak")

    if a.restore:
        if not bak.exists():
            print(f"백업이 없습니다: {bak}", file=sys.stderr)
            return 1
        shutil.copy2(bak, path)
        print(f"되돌림  {path}  <- {bak}")
        return 0

    text = path.read_text(encoding="utf-8")
    missing = [n for n in NAMES if not block(n).search(text)]
    if missing:
        print(f"못 찾은 동체 visual: {', '.join(missing)}", file=sys.stderr)
        print("  fix_model_rocket.py가 먼저 돌아야 합니다(로켓형 레이아웃 전제).",
              file=sys.stderr)
        return 1
    if "base_link" not in text or "rotor_0" not in text:
        print("base_link 또는 rotor_0 링크를 못 찾았습니다. 다른 모델입니다.",
              file=sys.stderr)
        return 1

    det = alloc_rank(ROTOR_YZ)
    if abs(det) < 1e-12:
        print(f"할당행렬이 특이합니다 (det={det:.3e}). 아무것도 쓰지 않았습니다.",
              file=sys.stderr)
        return 1

    if not bak.exists():
        shutil.copy2(path, bak)
        print(f"원본 백업  {bak}")

    for n in NAMES:
        text = block(n).sub("", text, count=1)
    # 첫 번째 동체 visual 자리(fuselage가 있던 자리는 이미 지워졌으므로) 대신
    # base_link 닫는 태그 앞에 메시 visual을 새로 끼워 넣는다.
    pat = re.compile(r'(<link name="base_link">[\s\S]*?)(\n[ \t]*</link>)')
    text, n_body = pat.subn(lambda m: m.group(1) + "\n" + MESH_BODY_VISUAL + m.group(2),
                            text, count=1)
    if n_body != 1:
        print("동체 메시 visual을 못 넣었습니다. 되돌리세요(--restore).", file=sys.stderr)
        return 1

    text, n_inertial = replace_inertial(text)
    text, n_rotor = replace_rotor_positions(text)
    text, n_mount = insert_mount_visuals(text)
    text, n_bell = insert_bell_visuals(text)

    path.write_text(text, encoding="utf-8")

    print(f"수정  {path}")
    print(f"  질량·관성: {'OK' if n_inertial else '실패 - base_link inertial 구조 확인 필요'}"
          f" (mass={MASS_KG:.4f} kg, Ixx={IXX:.6f} Iyy=Izz={IYY:.6f})")
    print(f"  로터 위치: {n_rotor}/4 곳 (반경 {ARM_HALF_M*math.sqrt(2):.5f} m, "
          f"허브 x={ROTOR_HUB_X:.5f} m from CG)")
    print(f"  동체 메시: body.stl 삽입됨")
    print(f"  모터 마운트(정적): {n_mount}/1회 삽입(4개 visual 포함)")
    print(f"  모터 벨(회전): {n_bell}/4 곳")
    print(f"  할당행렬 det={det:.4f} (특이 아님)")
    print("\n  ⚠ meshes/*.stl을 model://fast_missile_base/meshes/ 로 먼저 복사해야")
    print("    gz sim이 참조를 찾습니다:")
    print("    cp gz_aero/meshes/{body,motor_mount,motor_bell}.stl "
          "<PX4 트리>/Tools/simulation/gz/models/fast_missile_base/meshes/")
    print("\n  ⚠ 프로펠러 블레이드(1345_prop_*.stl)는 이 CAD에 없어 기존 것을 그대로 씀 -")
    print("    회전 반경·위치가 새 로터 허브와 시각적으로 안 맞을 수 있음, 확인 필요")
    print("\n  확인: gz sim으로 띄워서 (1) 모터 마운트/벨이 똑바로 보이는지")
    print("        (뒤집혀 보이면 bell_visual_block의 +1.5707963를 -1.5707963로)")
    print("        (2) 무게중심이 동체 중앙 부근에 있는지 (3) 총 4개 로터가")
    print("        기수축 둘레 대칭으로 보이는지")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
