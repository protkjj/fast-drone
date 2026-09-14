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
    path.write_text(text, encoding="utf-8")

    print(f"{'이미 고쳐져 있던 것을 덮어씀' if already else '수정'}  {path}")
    print("  동체 visual: 장축 X, 지름 0.15 m, 길이 1.0 m, 무게중심을 걸치게.")
    print(f"  로터 4 개: 기수축 둘레 반지름 {ARM:.4f} m, yz 평면, 링크를 y -90 도 회전.")
    print("  조인트 축: expressed_in=\"__model__\" 로 (1,0,0) 명시.")
    print("  관성·질량·공력 플러그인은 건드리지 않았습니다.")
    print("\n  확인:  gz sim 으로 띄우면 기수가 **위를 보고 서 있어야** 합니다.")
    print("         기수축을 따라 내려다보면 동체 단면이 원으로 보이고")
    print("         그 둘레에 로터 넷이 놓여 있어야 합니다.")
    print("\n  ⚠ 공력 플러그인을 다시 붙여야 할 수 있습니다:")
    print("     python3 gz_aero/tools/attach_to_model.py --model <경로> --csv <표>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
