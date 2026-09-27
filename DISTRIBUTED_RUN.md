# 분산 실행 — 학교 컴퓨터에서 절차 (kj 작업지시서 5단계)

이 문서 하나만 위에서 아래로 따라 하면 된다. 각 절은 **한 명령씩** 순서대로다. 스레드 환경변수는
셸을 export한 뒤에만 유지되므로, 새 터미널을 열면 그 export부터 다시 한다.

**두 갈래**: **A. 튜닝 분산**(지금 필요 — F13·M17이 맥에서 너무 오래 걸려서)과
**B. 본 실험 분산**(4단계 `configs/main_experiment.json`이 나온 뒤, 지금은 준비 중)이다. A부터 본다.

---

## 0. 공통 준비

### 0.1 읽기 전용 clone

개인 계정 자격정보를 학교 컴퓨터에 남기지 않는다. 짧게 쓰고 버릴 자격정보(읽기 전용 개인 액세스
토큰 등)를 GitHub에서 새로 만들어 아래처럼 URL에 한 번만 쓴다(셸 기록에 남지 않게 `HISTIGNORE`나
`set +o history` 뒤에 실행하거나, 다음 방법을 권한다: 명령을 실행한 뒤 `history -d`로 그 줄만 지우기).

```bash
git clone --branch protkjj/arena-completion --single-branch https://<READ_ONLY_TOKEN>@github.com/protkjj/fast-drone.git fast-drone-shard
cd fast-drone-shard
git config credential.helper ""
```

일이 끝나면(A.5 또는 B.6) 그 토큰을 GitHub에서 즉시 폐기한다. `git config --list`로 자격정보가
설정 파일에 평문으로 남지 않았는지 확인한다(`credential.helper ""`가 이미 막는다).

### 0.2 파이썬 환경

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-lock.txt
```

`requirements-lock.txt`는 기준 컴퓨터(맥, Darwin arm64)의 정확한 버전이다. 학교 컴퓨터가
Linux/x86_64면 똑같은 빌드가 없을 수 있다 — 그래도 이 명령으로 최대한 가깝게 맞추고, 다음 단계
(`setup_env.py`)가 나머지를 WARN으로 알려준다. 못 맞는 것은 실행을 막지 않는다. 최종 판정은
A.4/B.5의 교차 확인(허용오차 비교)이 한다.

### 0.3 스레드 환경변수 (이 터미널 세션 내내 유지)

```bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 MKL_NUM_THREADS=1
```

### 0.4 환경 점검 — 여기서 FAIL이 나면 다음 단계로 가지 않는다

```bash
python3 scripts/setup_env.py --commit <kj가 알려준 커밋 sha>
```

PASS가 아니면 멈추고 표를 그대로 kj에게 보낸다. 흔한 원인: 0.3을 건너뜀(스레드 FAIL),
clone이 다른 브랜치/커밋(git revision FAIL), pip install이 일부만 됨(패키지 WARN 다수).

---

## A. 튜닝 분산 (지금 — F13·M17을 학교 컴퓨터로)

경기장 튜닝(`control/arena_tune.py`)은 제어기별로 완전히 독립이다 — **제어기 1개 = 컴퓨터 1대,
끝까지**(kj 규칙). 그래서 "나누기"는 어느 컴퓨터가 어느 제어기를 맡는지 정하는 것뿐이고, "합치기"는
결과 파일 두 개(`<제어기>.record.json`, `<제어기>.jsonl`)를 한 폴더에 모으는 것뿐이다 — 시나리오·
행 단위로 쪼개거나 다시 섞을 필요가 없다(제어기마다 파일이 따로라서 충돌이 안 난다).

### A.1 시작(학교 컴퓨터, 0단계를 마친 뒤)

```bash
python3 scripts/setup_env.py --commit <sha> --config configs/arena.json \
  --expect-config-sha256 <kj가 알려준 해시> --expect-controller-model-sha256 <kj가 알려준 해시>
```

PASS를 확인한 뒤:

```bash
mkdir -p results/arena/tuning/retune_v2
caffeinate -dims python3 -m control.arena_tune --controllers M17 --budget 120 \
  --run-dir results/arena/tuning/retune_v2 > retune_v2_M17.log 2>&1
```

(`caffeinate`는 macOS 전용이다. Ubuntu 학교 컴퓨터면 그냥 앞의 `caffeinate -dims` 없이 실행하고,
대신 화면이 꺼지지 않게 `systemd-inhibit --what=idle python3 -m control.arena_tune …`을 쓰거나
로그인 세션을 유지한다.) F13도 다른 컴퓨터에서 같은 방식으로:

```bash
python3 -m control.arena_tune --controllers F13 --budget 120 --run-dir results/arena/tuning/retune_v2
```

**중단·재개**: 같은 명령을 다시 실행하면 `<제어기>.jsonl`에서 끝난 평가를 재생하고 이어 간다
(재개 가드가 설정 해시·시나리오 id를 확인한다 — 다르면 멈춘다).

### A.2 진행 확인

```bash
tail -1 retune_v2_M17.log
```

### A.3 결과 회수 (학교 컴퓨터 → kj에게, 또는 공유 폴더로)

두 파일만 있으면 된다:

```bash
scp results/arena/tuning/retune_v2/M17.record.json results/arena/tuning/retune_v2/M17.jsonl \
  <kj 컴퓨터 또는 공유 폴더>
```

### A.4 합치기 + I-4 검사 (맥에서, 5종이 모두 모인 뒤)

각 제어기의 두 파일을 `results/arena/tuning/retune_v2/`(맥의 같은 폴더)에 넣기만 하면 된다 —
파일명이 제어기별로 다르므로 충돌하지 않는다. 그다음 기존 도구로 검사한다:

```bash
python3 -m control.arena_tune --summarize --run-dir results/arena/tuning/retune_v2
```

`i4_violations`가 빈 목록이어야 한다(예산·시나리오·탐색 방식이 5종 모두 같은지). 다르면 위반
목록을 그대로 kj에게 보낸다.

### A.5 마무리

학교 컴퓨터의 clone을 지우고(`rm -rf fast-drone-shard`), 0.1에서 만든 읽기 전용 토큰을 GitHub에서
폐기한다.

---

### A.6 맥에서 미리 시험한 결과(2026-09-27)

절 A 전체(clone→venv→pip install→환경변수→`setup_env.py`→작은 튜닝(예산 3)→결과 회수→
`--summarize`)를 별도 git worktree(가상의 "학교 컴퓨터")로 한 번 돌렸다. `pip install -r
requirements-lock.txt`가 `python==3.13.7` 줄 때문에 실패하는 버그를 여기서 찾아 고쳤다(락 파일에서
그 줄을 빼고 `setup_env.py`가 따로 확인하게 바꿈, 커밋 `54ebfe0`). 고친 뒤에는 전 과정이 통과했고,
사전값 목적함수(0.7847247648921318)가 원래 맥 실행과 정확히 같았다(같은 코드·설정·게인이면 기대되는 값).

## B. 본 실험 분산 (준비 중 — 4단계 `configs/main_experiment.json` 확정 후 채운다)

`scripts/make_shards.py`(사례 단위로 나누기, 한 사례의 전 제어기는 같은 조각) →
`scripts/run_shard.py`(조각 하나 실행, 중간저장·재개, 컴퓨터 지문 기록) →
`scripts/merge_results.py`(누락·중복·해시 불일치 검사) →
`scripts/cross_check.py`(무작위 5% 사례를 다른 컴퓨터에서 재실행, rtol 1e-3 + 판정 일치 비교).

이 절은 4단계가 끝나면 A절과 같은 형식(clone → 환경 점검 → 조각 실행 → 회수 → 합치기 → 교차 확인
→ 마무리)으로 채운다.
