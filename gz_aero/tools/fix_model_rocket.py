#!/usr/bin/env python3
"""fast_missile_base 를 로켓형 배치로 바꾼다 (되돌릴 수 있게).

    python3 gz_aero/tools/fix_model_rocket.py --model <경로>
    python3 gz_aero/tools/fix_model_rocket.py --model <경로> --restore

무엇을 바꾸나:

  기체가 로켓형으로 확정됐다 — 로터가 **동체축에 수직인 평면**에 놓이고 추력이
  기수 방향이다. 기수축을 따라 내려다보면 동체 단면이 원으로 보이고 그 둘레에
  로터 넷이 있다. control/vehicle_params.py 의 rocket_params 와 같은 배치다.

  그런데 model.sdf 는 아직 옛 배치다:
    · 로터가 xy 평면 (±0.174, ±0.174, 0.06), 조인트 축이 z
    · 동체 visual 은 z 로 세워 그려져 있는데 관성은 장축 x 를 말한다

  관성이 옳다. Iyy = Izz = 0.70 은 **x 축 둘레 4겹 대칭**을 요구하는데, 로터가
  xy 평면에 다 있으면 Izz 에만 기여해 그 조건이 깨진다. 즉 관성값이 처음부터
  로켓 배치를 가리키고 있었고, 어긋나 있던 것은 로터 기하다.

바꾸는 것:
  1. 로터 링크를 yz 평면으로 옮기고 (0.02, ±0.177, ±0.177) 각 링크를 y 축
     -90 도 돌린다. 링크를 돌리면 프로펠러 메시·충돌체가 통째로 따라오고,
     링크 지역 z 가 모델 +x 가 되어 조인트 축이 자연히 기수 방향이 된다.
  2. 조인트 축을 expressed_in="__model__" 로 (1,0,0) 을 명시한다. SDF 버전마다
     축 기준 프레임 해석이 달라 헷갈릴 자리라 못박는다.
  3. 동체 visual 을 장축 x 로 눕히고 지름을 공력표의 d_ref 0.15 m 에 맞춘다.

옛 설명 (참고, DESIGN.md 9 장):
  model.sdf 는 동체 원통을 **Z 축(세로)** 으로 그린다. 그런데 같은 파일의
  관성과 로터 조인트 축은 **기수가 X(가로)** 라고 말한다.

    관성  Ixx 0.02  Iyy=Izz 0.70   -> 장축 X (세장체는 장축 관성이 제일 작다)
    로터  <xyz>0 0 1</xyz>          -> 추력이 Z, 기수와 수직
    주석  "장축 = X(비행방향)"       -> X
    visual 원통을 Z 로 그림          -> Z   ← 이것만 어긋난다

  그래서 화면으로 보면 어뢰가 위를 보면서 옆으로 나는 것처럼 보인다. 물리는
  멀쩡한데 그림이 90도 틀려서, 보는 사람이 기체 개념을 잘못 읽게 된다.
  실제로 그 혼동이 생겼다.

같이 바로잡는 것:
  · 지름 0.17 m (반지름 0.085) -> **0.15 m**. 공력표의 d_ref 가 0.15 이고
    S_ref = pi*0.15^2/4 이다. 그림이 표와 다른 기체를 보여주고 있었다.
  · 길이. vehicle_params 의 body_length 는 1.0 m 인데 그림은 0.45 m 였다.
  · 무게중심. <inertial><pose> 가 없어 무게중심이 링크 원점인데, 그림은 동체가
    통째로 원점 위에 얹혀 있었다. 원점을 걸치도록 옮긴다.

관성·질량·공력 플러그인 블록은 건드리지 않는다.
"""
import argparse
import pathlib
import re
import shutil
import sys

PI2 = "1.5707963"
NAMES = ("fuselage", "nose", "nose_tip", "tail")
ARM = 0.25 / (2 ** 0.5)          # vehicle_params 의 arm/sqrt(2) = 0.17678
ROTOR_X = 0.02                   # 로터 평면의 기수축 위치 (무게중심 근처)
# 로터 넷을 기수축(x) 둘레 90 도 간격으로. rocket_params 와 같은 순서·회전방향.
ROTOR_YZ = [(ARM, ARM), (-ARM, ARM), (-ARM, -ARM), (ARM, -ARM)]

# ★ 회전방향도 같이 바꿔야 한다. 안 그러면 할당행렬이 특이해진다.
#
#   이 스크립트가 로터 위치를 **원을 도는 순서**(1->2->3->4 사분면)로 새로
#   배정하는데, x500 의 turningDirection 은 **대각쌍이 (0,1)** 인 순서를
#   전제한 값이다. 위치만 바꾸고 방향을 두면 로켓 배치에서
#
#     Mx 행 = 0.4714 x My 행     (정확히 상수배)
#
#   가 되어 rank 3 이 된다 — 롤 명령과 피치 명령이 같은 모터 조합을 요구하니
#   어느 쪽도 원하는 대로 안 나온다. Ixx 가 Iyy 의 1/35 라 롤이 제일 민감한
#   축인데 그 축의 권한이 통째로 사라진다. 실측: 특이값 [2, 0.391, 0.354, 4.7e-17].
#   대각 교대로 바꾸면 rank 4, cond 12.0 이 된다.
ROTOR_DIR = [1, -1, 1, -1]       # control/vehicle_params.py 의 rotor_directions
# 규약: vehicle_params.py 주석이 d=+1 을 CW 로 적고 있어 그대로 따른다.
#   ⚠ 이 **전역 부호**는 아직 확정이 아니다. dynamics.py 는 반토크를
#     `M += d_i * Q_i` 로 쓰는데, 그러면 로터는 -d_i 로 도는 것이라 위 주석과
#     반대로 읽힌다. 패턴(대각 교대)은 부호와 무관하게 확실하고, 전역 부호가
#     뒤집혀 있으면 **롤만** 반대로 돈다 — SITL 에서 한 번 재면 끝난다.
#     방법: 정지비행에서 롤 명령을 조금 주고 기체가 명령과 같은 쪽으로 도는지 본다.
#     반대면 아래 ROTOR_DIR 의 부호를 전부 뒤집는다 (패턴은 그대로).
DIR_WORD = {1: "cw", -1: "ccw"}


def alloc_rank(yz, dirs, kQ_over_kT=5e-6 / 6e-5):
    """로켓 배치 할당행렬 [T, Mx, My, Mz] 의 rank. 특이하면 쓰지 않는다."""
    A = [[1.0] * 4,
         [dirs[i] * kQ_over_kT for i in range(4)],
         [yz[i][1] for i in range(4)],
         [-yz[i][0] for i in range(4)]]
    # 가우스 소거로 행렬식. 4x4 라 이걸로 충분하고 의존성이 안 는다.
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

# body_length 1.0, body_diameter 0.15 (control/vehicle_params.py) 기준.
# 원통 0.62 + 노즈 0.19 + 테일 0.19 = 1.0, 원점을 걸치게 배치.
NEW = {
    "fuselage": f"""      <visual name="fuselage">
        <pose>0 0 0 0 {PI2} 0</pose>
        <geometry>
          <cylinder>
            <radius>0.075</radius>
            <length>0.62</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.03 0.03 0.05 1</ambient>
          <diffuse>0.07 0.08 0.11 1</diffuse>
          <specular>0.35 0.35 0.4 1</specular>
        </material>
      </visual>""",
    "nose": f"""      <visual name="nose">
        <pose>0.405 0 0 0 {PI2} 0</pose>
        <geometry>
          <cylinder>
            <radius>0.075</radius>
            <length>0.19</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.03 0.03 0.05 1</ambient>
          <diffuse>0.07 0.08 0.11 1</diffuse>
          <specular>0.35 0.35 0.4 1</specular>
        </material>
      </visual>""",
    "nose_tip": """      <visual name="nose_tip">
        <pose>0.5 0 0 0 0 0</pose>
        <geometry>
          <sphere><radius>0.062</radius></sphere>
        </geometry>
        <material>
          <ambient>0.55 0.06 0.06 1</ambient>
          <diffuse>0.85 0.10 0.08 1</diffuse>
          <specular>0.4 0.2 0.2 1</specular>
        </material>
      </visual>""",
    "tail": f"""      <visual name="tail">
        <pose>-0.395 0 0 0 {PI2} 0</pose>
        <geometry>
          <cylinder><radius>0.062</radius><length>0.17</length></cylinder>
        </geometry>
        <material>
          <ambient>0.10 0.10 0.10 1</ambient>
          <diffuse>0.16 0.16 0.16 1</diffuse>
        </material>
      </visual>""",
}


def block(name):
    return re.compile(r'[ \t]*<visual name="' + re.escape(name) + r'">'
                      r'[\s\S]*?</visual>')


def move_rotors(text):
    """로터 링크를 yz 평면으로 옮기고 조인트 축을 기수 방향으로 못박는다."""
    n = 0
    for i, (py, pz) in enumerate(ROTOR_YZ):
        # <link name="rotor_i"> ... <pose>..</pose>
        pat = re.compile(r'(<link name="rotor_' + str(i) + r'">[\s\S]*?<pose>)'
                         r'[^<]*(</pose>)')
        text, k = pat.subn(
            lambda m: m.group(1)
            + f"{ROTOR_X} {py:.6f} {pz:.6f} 0 -1.5707963 0"
            + m.group(2), text, count=1)
        n += k
        # 조인트 축을 모델 프레임 기준으로 못박는다
        jpat = re.compile(r'(<joint name="rotor_' + str(i)
                          + r'_joint"[\s\S]*?<axis>\s*)<xyz[^>]*>[^<]*</xyz>')
        text, k2 = jpat.subn(
            lambda m: m.group(1) + '<xyz expressed_in="__model__">1 0 0</xyz>',
            text, count=1)
        n += k2
    return text, n


def set_spin(text):
    """로터마다 turningDirection 을 ROTOR_DIR 에 맞춘다.

    플러그인 블록을 joint 이름으로 찾는다. <motorNumber> 순서를 믿지 않는
    이유는, 이 스크립트가 바꾸는 것이 링크 위치이고 그 위치와 짝지어야 하는
    것이 같은 링크를 도는 조인트이기 때문이다.
    """
    n, changed = 0, []
    for i, d in enumerate(ROTOR_DIR):
        want = DIR_WORD[d]
        pat = re.compile(
            r'(<plugin\b(?:(?!</plugin>)[\s\S])*?rotor_' + str(i)
            + r'_joint(?:(?!</plugin>)[\s\S])*?<turningDirection>)'
            r'([^<]*)(</turningDirection>)')
        m = pat.search(text)
        if not m:
            continue
        was = m.group(2).strip()
        if was != want:
            changed.append(f"rotor_{i}: {was} -> {want}")
        text = pat.sub(lambda mm: mm.group(1) + want + mm.group(3), text, count=1)
        n += 1
    return text, n, changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="fast_missile_base/model.sdf")
    ap.add_argument("--restore", action="store_true",
                    help=".bak 에서 되돌린다 (플러그인 블록까지 함께 사라진다)")
    a = ap.parse_args()

    path = pathlib.Path(a.model)
    if not path.is_file():
        print(f"model.sdf 가 없습니다: {path}", file=sys.stderr)
        return 1
    bak = path.with_suffix(path.suffix + ".bak")

    if a.restore:
        if not bak.exists():
            print(f"백업이 없습니다: {bak}", file=sys.stderr)
            return 1
        shutil.copy2(bak, path)
        print(f"되돌림  {path}  <- {bak}")
        print("  ⚠ 백업 시점으로 돌아가므로 공력 플러그인 블록도 사라집니다.")
        print("     필요하면 attach_to_model.py 를 다시 돌리세요.")
        return 0

    text = path.read_text(encoding="utf-8")
    missing = [n for n in NAMES if not block(n).search(text)]
    if missing:
        print(f"못 찾은 visual: {', '.join(missing)}", file=sys.stderr)
        print("  이미 고쳤거나 다른 모델입니다. 아무것도 바꾸지 않았습니다.",
              file=sys.stderr)
        return 1

    already = 'expressed_in="__model__"' in text
    if not bak.exists():
        shutil.copy2(path, bak)
        print(f"원본 백업  {bak}")
    for n in NAMES:
        text = block(n).sub(lambda _m, k=n: NEW[k], text, count=1)
    text, nrot = move_rotors(text)
    if nrot != 8:
        print(f"로터 링크/조인트 8 곳 중 {nrot} 곳만 바꿨습니다. 확인이 필요합니다.",
              file=sys.stderr)
    text, nspin, spin_changed = set_spin(text)
    if nspin != 4:
        print(f"turningDirection 4 곳 중 {nspin} 곳만 찾았습니다. 확인이 필요합니다.",
              file=sys.stderr)

    # ★ 쓰기 전에 할당행렬이 특이하지 않은지 본다. 특이한 SDF 는 시뮬이
    #   멀쩡히 돌면서 결과만 틀리는 종류라, 조용히 내보내면 안 된다.
    det = alloc_rank(ROTOR_YZ, ROTOR_DIR)
    if abs(det) < 1e-12:
        print(f"할당행렬이 특이합니다 (det={det:.3e}). 아무것도 쓰지 않았습니다.",
              file=sys.stderr)
        print("  ROTOR_YZ 와 ROTOR_DIR 이 대각 교대인지 확인하세요.", file=sys.stderr)
        return 1
    path.write_text(text, encoding="utf-8")

    print(f"{'이미 고쳐져 있던 것을 덮어씀' if already else '수정'}  {path}")
    print("  동체 visual: 장축 X, 지름 0.15 m, 길이 1.0 m, 무게중심을 걸치게.")
    print(f"  로터 4 개: 기수축 둘레 반지름 {ARM:.4f} m, yz 평면, 링크를 y -90 도 회전.")
    print("  조인트 축: expressed_in=\"__model__\" 로 (1,0,0) 명시.")
    print(f"  회전방향: {', '.join(DIR_WORD[d] for d in ROTOR_DIR)}"
          f"  (할당행렬 det={det:.4f}, 특이 아님)")
    if spin_changed:
        for c in spin_changed:
            print(f"    바꿈  {c}")
        print("    ↑ 이게 없으면 Mx 행이 My 행의 상수배가 되어 rank 3 입니다"
              " — 롤 권한이 통째로 사라집니다.")
    else:
        print("    (이미 맞아 있었습니다)")
    print("  관성·질량·공력 플러그인은 건드리지 않았습니다.")
    print("\n  확인:  gz sim 으로 띄우면 기수가 **위를 보고 서 있어야** 합니다.")
    print("         기수축을 따라 내려다보면 동체 단면이 원으로 보이고")
    print("         그 둘레에 로터 넷이 놓여 있어야 합니다.")
    print("\n  ⚠ 회전방향의 **전역 부호**는 아직 확정이 아닙니다.")
    print("     패턴(대각 교대)은 확실하지만, 전부 뒤집혀 있으면 롤만 반대로 돕니다.")
    print("     정지비행에서 롤을 조금 주고 명령과 같은 쪽으로 도는지 한 번만 보세요.")
    print("     반대면 이 파일의 ROTOR_DIR 부호를 전부 뒤집고 다시 돌리면 됩니다.")
    print("\n  ⚠ 공력 플러그인을 다시 붙여야 할 수 있습니다:")
    print("     python3 gz_aero/tools/attach_to_model.py --model <경로> --csv <표>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
