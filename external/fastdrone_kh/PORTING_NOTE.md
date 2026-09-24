# 이 사본에 대한 메모 (control 브랜치 쪽에서 작성)

**출처**: https://github.com/KYUHYUNKANG03/fastdrone_kh, 브랜치 `repro/kyuhyun`,
커밋 `34981c5`("Publish frozen pack_forward v2 aircraft, integration guide, and
preliminary results"). 2026-09-25 클론.

## 이 사본에서 바꾼 것 — 딱 하나, 기계적 변경만

원본 저장소의 패키지 이름이 `control`이라 우리 프로젝트의 `control/`(물리·
제어기 핵심 모듈)과 이름이 겹친다. 두 코드베이스를 같은 프로세스에서 동시에
쓰려면(재현·비교 시험) 서로 다른 이름이 필요하다.

**한 일**: 패키지 디렉터리를 `control/` → `kh_control/`로 옮기고, 그 안팎의
import 문에서 `from control.X import` / `import control.X` 패턴만 정규식으로
`kh_control`로 바꿨다. 로직·수치·주석·문서는 전혀 건드리지 않았다(재구현
아님 — kj 지시: "팀원의 플랜트를 다시 구현하지 말고 그대로 불러 쓰는 어댑터로
감싼다").

## 동등성 검증 — 원본이 이미 만들어 둔 도구로 확인

원본 저장소의 `README.md`가 명시한 기준 해시·트림표, `AGENTS.md`가 요구하는
검증 명령을 이름 변경 **전후로 각각** 돌려 비교했다.

| 확인 | 이름변경 전 | 이름변경 후 |
|---|---|---|
| `python -m control(.\|_control).validate_model --authority`의 파라미터 SHA256 | `13a1dcc6332987eba93ea4653f4d6087741d1ac085701982ed94c22e6658c807` | **동일** |
| 0~85 m/s 트림표(피치각·최대rpm·여유·잔차) | README 표와 일치 | **동일**(자릿수까지) |
| `pytest tests -q` | 32 passed | **32 passed**(동일) |

세 가지가 이름 변경 전후로 완전히 같다 — 이 사본은 원본과 수치적으로 동등하다.

## 이 디렉터리를 쓰는 법

```python
import sys
sys.path.insert(0, 'external/fastdrone_kh')   # kh_control 패키지를 찾도록
from kh_control.baseline_v2 import baseline_params
from kh_control.dynamics import AxialDronePlant
from kh_control.trim import find_trim
```

우리 쪽 `control.*`와 이름이 겹치지 않으므로 같은 스크립트에서 우리 V13과
이 저장소의 분리형을 동시에 불러 비교할 수 있다.

## 하지 않은 것

- 물리·제어기 로직 수정 없음.
- 파라미터·프로펠러 곡선·공력 계수 수정 없음.
- `results/`(원본의 61개 시험 결과) 내용 수정 없음 — 그대로 보존.
- 이 저장소를 우리 team 원격 저장소(protkjj/fast-drone)에 push 하지 않았다
  (원본 AGENTS.md: "No push to the upstream team repository").
