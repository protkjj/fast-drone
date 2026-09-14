#!/usr/bin/env python3
"""fast_missile_base 의 동체 visual 을 물리와 맞춘다 (되돌릴 수 있게).

    python3 gz_aero/tools/fix_model_visual.py --model <경로>
    python3 gz_aero/tools/fix_model_visual.py --model <경로> --restore

무엇이 틀렸나 (DESIGN.md 9 장):
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

물리는 **하나도 건드리지 않는다.** 관성·로터·조인트·플러그인 블록은 그대로다.
"""
import argparse
import pathlib
import re
import shutil
import sys

PI2 = "1.5707963"
NAMES = ("fuselage", "nose", "nose_tip", "tail")

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

    already = '<pose>0 0 0 0 1.5707963 0</pose>' in text
    if not bak.exists():
        shutil.copy2(path, bak)
        print(f"원본 백업  {bak}")
    for n in NAMES:
        text = block(n).sub(lambda _m, k=n: NEW[k], text, count=1)
    path.write_text(text, encoding="utf-8")

    print(f"{'이미 고쳐져 있던 것을 덮어씀' if already else '수정'}  {path}")
    print("  동체 visual 을 장축 X 로 눕히고 지름을 0.15 m, 길이를 1.0 m 로 맞췄습니다.")
    print("  무게중심(링크 원점)을 걸치도록 배치했습니다.")
    print("  물리(관성·로터·조인트·플러그인)는 건드리지 않았습니다.")
    print("\n  확인:  gz sim 으로 띄우면 어뢰가 **가로로 누워** 보여야 합니다.")
    print("         그게 관성과 로터 축이 말하는 실제 기체입니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
