# e2 맥 전 과정 점검 — 분산 실행기(`control/main_distributed.py`) (2026-09-28 밤)

**목적**: 조각 실행 → 중단 → 이어 돌리기 → 합치기 → 교차 확인을 실제 시행으로 한 번 돌려 본다.
**성능 결과로 쓰지 않는다**(맥 결과는 본 실험과 섞지 않는다 — kj 규칙).

- spec: `spec_maccheck.json` — `configs/main_experiment.json`에서 기준을 `arena.json`, 제어기를 GSLQR·CPID(retune_v3 기록)로 바꾼 점검 전용 사본.
- 조각: `shards.json` — `make-shards --n 2 --only-batches gust reference`. 조각 0에 16시행, 조각 1에 30시행(예상 시간 기준 LPT 배분).
- 실행 폴더(`runs/`, `runs_b/`, `cross_rerun/`)는 궤적(.npz)과 컴퓨터 이름이 들어 있어 커밋하지 않는다.

## 결과

| 단계 | 한 일 | 결과 |
|---|---|---|
| 조각 실행 + 중단 | 조각 0을 돌리다 완료 파일 3개가 생긴 순간 `kill -9` | 끊긴 네 번째 시행은 결과 파일이 없다(쓰기 전에 끊김). 임시 파일도 남지 않았다 |
| 이어 돌리기 | 같은 명령 다시 실행 | 끝난 3개 건너뜀, 끊긴 시행(V_L 수직 돌풍 +5 / CPID)부터 13개 실행 → 16/16 |
| 합치기(기본) | `merge` (Windows 요구) | **거부**: 46건 모두 `platform Darwin != required Windows` — 의도대로 |
| 합치기(첫 시도, Darwin 허용) | 조각 0 앞부분이 NaN 수정 커밋 **전** 작업 트리(`b556441`, dirty)에서 돌았다 | **거부**: `rows come from different code states` — 검사가 실제 섞임을 잡았다 |
| 조각 0 재실행 | 깨끗한 `bccfb96`에서 새 폴더(`runs_b`)로, 중단·이어 돌리기 다시 | 3개 후 kill → 13개 재개, 16/16 |
| 합치기(Darwin 허용, 맥 점검 전용) | `runs_b` + `runs/shard1` | **MERGED** 46/46(돌린 32, 설계 영역 제외 14), 코드 상태 1개(`bccfb96`) |
| 교차 확인 선택 | `cross-select` 5%, 시드 20260928 | 모집단 32 중 2시행: `gust/main_gust_vertical_+5_V_H/GSLQR`, `reference/main_ref_accel_VH_rho1/GSLQR` |
| 교차 확인(기본) | 같은 맥에서 재실행 → `cross-compare` | **FAIL**: 같은 컴퓨터 — 의도대로(본 실험은 다른 Windows 컴퓨터 필수) |
| 교차 확인(`--allow-same-machine`, 맥 점검 전용) | 같은 재실행 결과 | **PASS**, 두 시행 모두 비트 동일 |

## 드러난 점
- 끊긴 시행의 임시 파일 정리 경로(`*.tmp` 삭제)는 이번 실제 중단에서는 쓰이지 않았다(kill이 쓰기 전에 걸림). 그 경로는 단위 시험(`control/test_main_distributed.py`)이 확인한다.
- 조각 실행 중에 코드를 커밋하면 한 조각 안에서 코드 상태가 갈린다 — 합치기가 거부한다. **본 실험은 태그로 받은 깨끗한 트리에서만 돌린다**(B절).
