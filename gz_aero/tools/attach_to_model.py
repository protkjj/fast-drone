#!/usr/bin/env python3
"""PX4 기체 model.sdf 에 공력 플러그인 블록을 넣는다 (되돌릴 수 있게).

왜 스크립트인가:
  model.sdf 는 우리 저장소 밖(PX4 트리)에 있다. 손으로 고치면 PX4 를 다시 받을
  때마다 사라지고, 무엇을 바꿨는지 기록도 안 남는다. 이 스크립트는
    · 처음 한 번 .bak 을 만들고
    · 이미 있는 우리 블록을 지운 뒤 다시 넣으므로 몇 번 돌려도 결과가 같고
    · --remove 로 원상복구된다.

사용:
    python3 gz_aero/tools/attach_to_model.py --model <경로> --csv <경로>
    python3 gz_aero/tools/attach_to_model.py --model <경로> --remove
"""
import argparse
import pathlib
import re
import shutil
import sys

PLUGIN_NAME = "fast_drone::AeroPlugin"

# 이미 박혀 있는 우리 블록을 찾는 정규식. 앞의 들여쓰기·주석까지 같이 먹어서
# 여러 번 돌려도 빈 줄이 쌓이지 않게 한다.
EXISTING = re.compile(
    r"\n[ \t]*<!-- fast_drone 공력 -->"
    r"|\n[ \t]*<plugin\b[^>]*name=['\"]" + re.escape(PLUGIN_NAME) + r"['\"][\s\S]*?</plugin>"
)


def block(csv: str, link: str, nose: str, right: str, wind: str, debug: str) -> str:
    lines = [
        "    <!-- fast_drone 공력 -->",
        f'    <plugin filename="FastDroneAero" name="{PLUGIN_NAME}">',
        f"      <link_name>{link}</link_name>",
        f"      <csv_file>{csv}</csv_file>",
        f"      <nose>{nose}</nose>",
        f"      <right>{right}</right>",
        f"      <wind>{wind}</wind>",
    ]
    if debug:
        lines.append(f"      <debug_csv>{debug}</debug_csv>")
        lines.append("      <debug_csv_every>10</debug_csv_every>")
    lines.append("    </plugin>")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="대상 model.sdf")
    ap.add_argument("--csv", help="공력표 CSV (절대경로 권장)")
    ap.add_argument("--link", default="base_link")
    ap.add_argument("--nose", default="1 0 0", help="링크좌표에서 기수 방향")
    ap.add_argument("--right", default="0 -1 0", help="링크좌표에서 오른날개 방향")
    ap.add_argument("--wind", default="0 0 0", help="월드 ENU 바람 [m/s]")
    ap.add_argument("--debug-csv", default="", help="비우면 로그를 안 남긴다")
    ap.add_argument("--remove", action="store_true", help="블록만 빼고 원상복구")
    args = ap.parse_args()

    path = pathlib.Path(args.model)
    if not path.is_file():
        print(f"model.sdf 가 없습니다: {path}", file=sys.stderr)
        return 1

    text = path.read_text(encoding="utf-8")
    had = bool(EXISTING.search(text))
    text = EXISTING.sub("", text)

    # 삽입·제거가 </model> 앞 공백을 **똑같이** 다시 만든다. 이걸 양쪽에서 공유해야
    # 여러 번 돌려도 결과가 같고(멱등), --remove 가 원본과 바이트 단위로 일치한다.
    # 자기검사에서 이 정규화를 빼먹어 둘 다 깨졌었다.
    idx = text.rstrip().rfind("</model>")
    if idx < 0:
        print("</model> 을 못 찾았습니다", file=sys.stderr)
        return 1
    head, tail = text[:idx].rstrip(), text[idx:].lstrip()

    if args.remove:
        text = head + "\n  " + tail
    else:
        if not args.csv:
            print("--csv 가 필요합니다", file=sys.stderr)
            return 1
        csv = pathlib.Path(args.csv).resolve()
        if not csv.is_file():
            print(f"CSV 가 없습니다: {csv}", file=sys.stderr)
            return 1
        if f'name="{args.link}"' not in text:
            print(f"링크 '{args.link}' 를 model.sdf 에서 못 찾았습니다", file=sys.stderr)
            return 1
        text = (head + "\n"
                + block(str(csv), args.link, args.nose, args.right,
                        args.wind, args.debug_csv)
                + "\n  " + tail)

    bak = path.with_suffix(path.suffix + ".bak")
    if not bak.exists():
        shutil.copy2(path, bak)
        print(f"원본 백업  {bak}")
    path.write_text(text, encoding="utf-8")

    if args.remove:
        print(f"블록 제거  {path}" if had else f"제거할 블록이 없었습니다  {path}")
    else:
        print(f"{'교체' if had else '삽입'}  {path}")
        print(f"  링크  {args.link}")
        print(f"  표    {pathlib.Path(args.csv).resolve()}")
        print(f"  기수  {args.nose}   오른쪽  {args.right}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
