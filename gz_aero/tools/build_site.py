#!/usr/bin/env python3
"""GitHub Pages 로 올릴 site/ 를 조립한다.

    python3 gz_aero/tools/build_site.py

Artifact 는 Claude 가 호스팅해서 GitHub Actions 로 못 올린다. 대신 두 페이지가
자립형 HTML 이라 Pages 에 그대로 올라간다. 저장소 도메인이라 링크가 안 바뀌고
푸시할 때마다 갱신된다.

발행용(.artifact.html)은 감싸는 쪽이 doctype·charset 을 붙여 주는 판이라
Pages 에는 로컬용(.html, charset 포함)을 올린다.
"""
import pathlib
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
PAGES = [
    ("flight_sim.html", "sim.html", "비행 시뮬레이터",
     "브라우저가 6자유도를 직접 풉니다. 공력 계수를 바꿔가며 날려 속도 한계를 찾습니다."),
    ("aero_dashboard.html", "aero.html", "공력 계수",
     "표의 특성 곡선과 실기체 비행 결과. 계수를 만져보고 검증 상태를 확인합니다."),
]

# 페이지에서 목록으로 돌아가는 길. Artifact 판에는 목록이 없으므로 **사이트를
# 조립할 때만** 넣는다. 두 페이지의 머리글 구조가 달라서, 있으면 머리글 안에
# 끼우고 없으면 화면 구석에 띄운다.
NAV = """
<style>
#site-home{display:inline-flex;align-items:center;gap:6px;text-decoration:none;
 padding:5px 11px;border:1px solid #2C3A48;border-radius:999px;background:#18212B;
 color:#AEBAC6;font:500 12px/1 "IBM Plex Sans",Arial,sans-serif;white-space:nowrap}
#site-home:hover{border-color:#F2AA4C;color:#EEF2F6}
#site-home:focus-visible{outline:2px solid #F2AA4C;outline-offset:2px}
#site-home.float{position:fixed;left:14px;bottom:14px;z-index:9999}
</style>
<script>
(function(){
  var a = document.createElement("a");
  a.id = "site-home"; a.href = "./"; a.textContent = "\u2190 목록";
  a.setAttribute("aria-label", "페이지 목록으로");
  var h = document.querySelector("header");
  if (h) { h.insertBefore(a, h.firstChild); a.style.marginRight = "14px"; }
  else { a.className = "float"; document.body.appendChild(a); }
})();
</script>
"""

INDEX = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>fast_drone · 공력</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600&family=IBM+Plex+Sans+Condensed:wght@700&family=IBM+Plex+Sans:wght@400;500&display=swap">
<style>
:root{--ground:#101820;--panel:#18212B;--panel-2:#202B37;--rule:#2C3A48;
 --ink:#EEF2F6;--ink-2:#AEBAC6;--ink-3:#7C8A98;--accent:#F2AA4C;color-scheme:dark}
*{box-sizing:border-box}
body{margin:0;background:var(--ground);color:var(--ink);
 font:400 15px/1.6 "IBM Plex Sans",Arial,sans-serif;-webkit-font-smoothing:antialiased}
.wrap{max-width:760px;margin:0 auto;padding:72px 24px 80px}
.eyebrow{font:400 11px/1 "IBM Plex Mono",monospace;letter-spacing:.14em;
 text-transform:uppercase;color:var(--ink-3)}
h1{font:700 clamp(30px,5vw,44px)/1.1 "IBM Plex Sans Condensed",sans-serif;
 margin:10px 0 14px;letter-spacing:-.015em}
p.lead{color:var(--ink-2);margin:0 0 40px;max-width:58ch}
.cards{display:grid;gap:14px}
a.card{display:block;text-decoration:none;color:inherit;background:var(--panel);
 border:1px solid var(--rule);border-radius:12px;padding:20px 22px;
 transition:border-color .15s,transform .15s}
a.card:hover{border-color:var(--accent);transform:translateY(-2px)}
a.card:focus-visible{outline:2px solid var(--accent);outline-offset:3px}
a.card h2{margin:0;font:700 20px/1.2 "IBM Plex Sans Condensed",sans-serif}
a.card p{margin:7px 0 0;color:var(--ink-3);font-size:14px}
a.card .go{margin-top:12px;display:inline-block;font:600 12px/1 "IBM Plex Mono",monospace;
 letter-spacing:.09em;text-transform:uppercase;color:var(--accent)}
footer{margin-top:46px;padding-top:18px;border-top:1px solid var(--rule);
 color:var(--ink-3);font-size:12.5px}
</style></head><body>
<div class="wrap">
<div class="eyebrow">fast_drone · gz_aero</div>
<h1>축대칭 동체 공력</h1>
<p class="lead">고속 ISR 미사일형 동체의 공력 모델과 비행 결과입니다.
설치 없이 브라우저에서 열립니다.</p>
<div class="cards">%%CARDS%%</div>
<footer>페이지는 저장소의 <code>results/</code> 를 그대로 올린 것입니다.
<code>gz_aero/tools/</code> 의 생성기를 다시 돌려 갱신합니다.</footer>
</div></body></html>
"""


def main():
    site = ROOT / "site"
    if site.exists():
        shutil.rmtree(site)
    site.mkdir()
    cards, missing = [], []
    for src, dst, title, desc in PAGES:
        f = ROOT / "results" / src
        if not f.is_file():
            missing.append(src)
            continue
        # 되돌아가기 링크를 끼워 넣는다. </body> 바로 앞이면 본문이 다 만들어진
        # 뒤라 머리글을 찾을 수 있다.
        html = f.read_text(encoding="utf-8")
        if "</body>" in html:
            html = html.replace("</body>", NAV + "\n</body>", 1)
        else:
            html += NAV
        (site / dst).write_text(html, encoding="utf-8")
        cards.append(f'<a class="card" href="{dst}"><h2>{title}</h2>'
                     f'<p>{desc}</p><span class="go">열기 →</span></a>')
    if not cards:
        print("올릴 페이지가 없습니다. 먼저 생성기를 돌리세요.", file=sys.stderr)
        return 1
    (site / "index.html").write_text(INDEX.replace("%%CARDS%%", "\n".join(cards)),
                                     encoding="utf-8")
    # Jekyll 이 밑줄로 시작하는 경로를 건너뛰지 않게 한다
    (site / ".nojekyll").write_text("", encoding="utf-8")
    print(f"site/ 조립 완료 — {len(cards)} 쪽")
    for src, dst, title, _ in PAGES:
        if (site / dst).exists():
            print(f"  {dst:18s} {title}  ({(site/dst).stat().st_size/1024:.0f} KB)")
    if missing:
        print(f"  빠짐: {', '.join(missing)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
