# 분산 실행 — 학교 컴퓨터에서 절차 (kj 작업지시서 5단계)

> **이 문서는 GitHub의 `protkjj/arena-completion` 브랜치 최신본을 보고 따라 한다.** 태그 `tune-final-6`으로 받은
> 폴더 안의 DISTRIBUTED_RUN.md는 태그를 단 뒤 고친 이 절차가 아니라 옛 Linux용 절차다. 코드는 태그 것을 쓰고,
> 절차는 이 최신본을 쓴다.

이 문서 하나만 위에서 아래로 따라 하면 된다. 각 절은 **한 명령씩** 순서대로다. 스레드 환경변수는
셸을 export한 뒤에만 유지되므로, 새 터미널을 열면 그 export부터 다시 한다.

**두 갈래**: **A. 튜닝 분산**(지금 필요 — F13·M17, 학교 컴퓨터는 **Windows**)과
**B. 본 실험 분산**(튜닝 확정·`configs/main_experiment.json` 가드 통과 뒤, 태그 `<MAIN_TAG>`)이다.
**A절은 Windows(PowerShell) 기준으로 준비부터 마무리까지 자체 완결이다 — 0절(Linux/맥용)은 건너뛴다.**

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

## A. 튜닝 분산 — F13·M17(·GSLQR)을 학교 컴퓨터(Windows)에서, 태그 `tune-final-6`, 설정 `configs/arena_v2.json`

> **2026-09-28 밤 변경(보고서 23절)**: 제어기 모델에 트림 밖 피치·요 모멘트 보정이 들어갔다(`configs/arena_v2.json`).
> 보정을 쓰는 M17·F13·GSLQR은 **다시 튜닝**한다. `tune-final-3`(옛 모델)으로 돌던 F13·M17 튜닝은 중단했고 결과는 쓰지 않는다
> (중단 전 진행 횟수만 보고서에 기록). V13·CPID는 옛 기록을 **승계**한다(`configs/tuning_carryover.json`, 비트 동일 증명).
> **튜닝 명령에는 반드시 `--config configs/arena_v2.json`을 준다.** 빠뜨리면 옛 모델로 돈다.
> **`tune-final-4`는 쓰지 않는다**(보고서 23.7절): 보정항이 호버에서 미분 NaN을 내 GSLQR 설계·NLP 기울기가 깨졌다.
> 고친 코드가 `tune-final-5`다. 결과 폴더도 새로 `results/arena/tuning/tune5`를 쓴다(`tune4`는 무효 결과).
> **`tune-final-6`(최신)**: `tune-final-5`와 실행 코드가 같다. 바뀐 것은 A.3 점검(`scripts/setup_env.py` — 제어기 모델을
> sha 정확 일치 대신 적합 계수 rtol 1e-8로 비교), 시험, 비교용 기준 파일(`configs/controller_model_reference.json`),
> 승계 증명 도구뿐이다. 튜닝 결과에 영향이 없으므로 **결과 폴더는 `tune5` 그대로** 쓰고, `tune-final-5`로 이미 시작한
> 튜닝은 그대로 둔다(보고서 23.8절).

> **재시작·로그오프 뒤 이어 돌리기 (학교 정책으로 컴퓨터가 꺼졌을 때)**
> PowerShell을 새로 열고 아래 세 줄을 실행한 다음 **A.6의 시작 명령을 그대로 다시 실행**한다.
> 기록된 평가는 재생하고 이어 간다. 같은 컴퓨터에서만 한다.
> ```powershell
> cd $HOME\fast-drone-shard
> Set-ExecutionPolicy -Scope Process Bypass; .\.venv\Scripts\Activate.ps1
> $env:OMP_NUM_THREADS="1"; $env:OPENBLAS_NUM_THREADS="1"; $env:VECLIB_MAXIMUM_THREADS="1"; $env:MKL_NUM_THREADS="1"; $env:PYTHONUTF8="1"
> ```
>
> **진행 확인**(언제든)
> ```powershell
> Select-String '"spent"|"status"' results\arena\tuning\tune5\M17.record.json; (Get-Item results\arena\tuning\tune5\M17.jsonl).LastWriteTime
> ```
> `spent`는 끝난 평가 수다. 마지막 기록 시각이 평가 1회 시간(M17은 수십 분)보다 훨씬 오래전이면
> 멈춘 것이다. 이 경우 A.7로 프로세스가 살아 있는지 보고, 없으면 위 절차로 재개한다.

경기장 튜닝(`control/arena_tune.py`)은 제어기마다 완전히 독립이다. "나누기"는 어느 컴퓨터가 어느
제어기를 맡을지 정하는 것뿐이다. "합치기"는 결과 파일 두 개(`<제어기>.record.json`, `<제어기>.jsonl`)를
한 폴더에 모으는 것뿐이다.

**규칙(kj)**
- **제어기 1개 = 컴퓨터 1대, 끝까지.** 도중에 다른 컴퓨터로 옮기지 않는다. 재개는 같은 컴퓨터에서만 한다.
- **F13과 M17은 같은 컴퓨터에서 돌리지 않는다.**
- 결과 폴더 이름은 모든 컴퓨터에서 `results/arena/tuning/tune5`로 쓴다. 합칠 때 복사만 하면 된다.

**제어기별 튜닝 태그** (보고서 19.1절)
| 제어기 | 태그 | 설정 | 튜닝 플랫폼 | 컴퓨터 |
|---|---|---|---|---|
| V13·CPID | `tune-final` (`8911ee1`) — **승계** | `arena.json` 기록을 `arena_v2`에서 비트 동일로 증명 | macOS (Darwin arm64) | kj 맥(retune_v3) |
| F13·M17 | `tune-final-6` (실행 코드 = `tune-final-5`) | `arena_v2` | Windows | 학교 |
| GSLQR | `tune-final-5` | `arena_v2` | macOS (Darwin arm64) | kj 맥(tune5) |

V13·CPID 승계: `arena.json`으로 튜닝한 retune_v3 기록의 평가 0과 최종 최선 평가를 `arena_v2`에서 다시 계산해
전 필드가 비트 동일할 때만 승계 표에 오른다(CPID 완료, V13은 retune_v3 종료 뒤 — 보고서 23.5절). `arena.json`
기본 경로가 모멘트 보정 뒤에도 비트 동일한 것도 재현 도구로 확인했다(23.6절).

기준값(tune-final-5·6, `configs/arena_v2.json` — 튜닝 기록의 config_sha256이 이 값이어야 한다):
- config_sha256 = `9729aec6ff8b54499c96d38c850fedf305970caad5bb2256e948dfa667ea32ca` (`configs/arena_v2.json`, 정규화 해시)
- controller_model_sha256 = `2582197db2546d329b62bee631beef5f89459c2c4b11a2131ab024240177956d`
  (명목 제어기 모델 — 모멘트 보정은 이 sha에 안 들어가고 config_sha256이 모멘트 모델 파일 sha까지 묶는다)

**미리 알아 둘 것**
- 아래 명령은 전부 **PowerShell**에서 한 줄씩 실행한다(cmd 아님).
- Windows 절차는 **맥에서 만들었고 Windows에서는 아직 한 번도 돌려 보지 않았다.** 처음 하는 컴퓨터에서는 A.4(재현)와 A.5(고아 점검)가 이 절차 자체의 시험이기도 하다. 어느 단계든 오류가 나면 멈추고 화면을 그대로 kj에게 보낸다.
- 필요한 프로그램: Git for Windows, Python 3.13(python.org 설치본). `py -3.13 --version`이 3.13.x를 보여야 한다.

### A.1 받기
PowerShell은 입력한 명령을 파일에 저장한다. 토큰이 디스크에 남지 않게 **먼저 기록 저장을 끈다**(이 창에서만).
```powershell
Set-PSReadLineOption -HistorySaveStyle SaveNothing
```
```powershell
cd $HOME
```
```powershell
git clone --branch tune-final-6 --depth 1 https://<READ_ONLY_TOKEN>@github.com/protkjj/fast-drone.git fast-drone-shard
```
```powershell
cd fast-drone-shard
```
```powershell
git config credential.helper ""
```
```powershell
git describe --tags --exact-match
```
마지막 명령은 `tune-final-6`을 출력해야 한다.

### A.2 파이썬 환경
```powershell
py -3.13 -m venv .venv
```
```powershell
Set-ExecutionPolicy -Scope Process Bypass
```
```powershell
.\.venv\Scripts\Activate.ps1
```
```powershell
python -m pip install -r requirements-lock.txt
```
- `Set-ExecutionPolicy -Scope Process`는 이 창에서만 스크립트 실행을 허용한다. 관리자 권한은 필요 없다.
- 환경변수는 이 창에서만 유지된다. 새 창을 열면 맨 위 '재시작 뒤' 상자의 세 줄부터 다시 한다.
- `PYTHONUTF8=1`은 출력과 파일의 한글 인코딩을 UTF-8로 맞춘다. 계산에는 영향이 없다.
```powershell
$env:OMP_NUM_THREADS="1"; $env:OPENBLAS_NUM_THREADS="1"; $env:VECLIB_MAXIMUM_THREADS="1"; $env:MKL_NUM_THREADS="1"; $env:PYTHONUTF8="1"
```

### A.3 설정·모델 점검
```powershell
python scripts\setup_env.py --commit $(git rev-parse HEAD) --config configs/arena_v2.json --expect-config-sha256 9729aec6ff8b54499c96d38c850fedf305970caad5bb2256e948dfa667ea32ca --expect-controller-model-sha256 2582197db2546d329b62bee631beef5f89459c2c4b11a2131ab024240177956d 2>&1 | Tee-Object -FilePath setup_env_A3.txt
```
- 종료코드(`$LASTEXITCODE`)가 0이 아니면 멈추고 표를 kj에게 보낸다.
- 패키지 버전 WARN은 괜찮다. 기준이 맥이라 Windows에서는 같은 빌드가 없을 수 있고, 최종 판정은 A.4가 한다.
- 스레드 FAIL은 A.2의 환경변수 줄을 빠뜨린 것이다. git FAIL은 받은 코드가 수정됐거나 다른 커밋이라는 뜻이다.
- 제어기 모델은 **계수**로 판정한다(`model C_Na` … `model x_cp_poly` 8줄, 맥 기준 rtol 1e-8). 이 컴퓨터에서 다시 적합한
  계수라 맥과 비트가 같지 않을 수 있다(리눅스 실측 sha 10b40c8c…). 그래서 `controller_model sha256` 줄은 **INFO**(기록만)이고,
  그 값은 결과 회수(A.9) 때 `.zip`에 함께 보낸다. 계수 줄 하나라도 FAIL이면 멈추고 표를 kj에게 보낸다.

### A.4 환경 점검 — V13 기록 재현 (FAIL이면 이 컴퓨터에서 튜닝을 시작하지 않는다)
맥 기록(retune_v3 V13 평가 0, 저장소의 `results/arena/tuning/env_check/`)을 이 컴퓨터에서 다시 계산해 비교한다.
- 작업자 수 N: `python -m control.arena_suggest_workers 0.6`의 값을 쓰되 3을 넘기지 않는다.
- 소요 시간: 맥 기준 작업자 3개로 약 4~7분, 순차로 약 13분이다.

```powershell
python -m control.arena_tune_repro --controller V13 --run-dir results/arena/tuning/env_check --index 0 --scenario-workers N 2>&1 | Tee-Object -FilePath env_check_V13.txt
```
- 마지막 줄이 `PASS (rtol 0.001 + identical verdicts); bit-identical: …`이면 합격이다.
- **합격 기준(kj)**: 판정 일치와 수치 rtol 1e-3이다.
  - 판정 일치: 시나리오별 failed, 정지 사유, 논문 판정 사유, 적분기 1% 규칙 판정이 정확히 같아야 한다.
  - 수치: 목적함수와 시나리오별 점수, 창 RMSE(속도·고도), 최대 |ω|의 상대오차가 1e-3 이하여야 한다.
- `bit-identical`은 **기록만 한다.** 다른 OS라 False일 수 있고, False여도 합격이다.
- `FAIL …`이면 **이 컴퓨터에서는 튜닝을 시작하지 않는다.** `env_check_V13.txt`를 kj에게 보낸다.
- Windows에서는 작업자 메모리가 기록되지 않는다(`peak_worker_rss_mib`가 비어 있음). 이는 정상이다.

### A.5 고아 작업자 점검 (1~2분, 처음 한 번만)
튜닝 본체가 비정상으로 죽었을 때(메모리 부족, 강제 종료 등) 작업자 프로세스가 스스로 끝나는지 이 컴퓨터에서
확인한다. 작업자가 남으면 M17은 작업자 1개당 약 2 GiB를 계속 쥐고 있게 된다. 맥에서는 확인했다(보고서 19.4절).
Windows에서는 아직 확인하지 않았다.

짧은 튜닝(GSLQR 예산 3, 작업자 2개)을 띄운다.
```powershell
$p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers','GSLQR','--budget','3','--run-dir','orphan_check','--scenario-workers','2' -PassThru -WindowStyle Hidden
```
40초 뒤 관련 python 목록을 본다. 튜닝 본체(명령줄에 `orphan_check`)와 작업자(명령줄에 `multiprocessing`)가 보여야 한다.
```powershell
Start-Sleep 40; Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Select-Object ProcessId,ParentProcessId,@{n='cmd';e={$_.CommandLine.Substring(0,[math]::Min(80,$_.CommandLine.Length))}}
```
**튜닝 본체만** 강제로 죽인다. venv의 중계 프로세스 아래에 있는 진짜 인터프리터다. 중계가 없으면 그 자신이다.
```powershell
$tune = Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -like '*orphan_check*' }; $inner = $tune | Where-Object { $tune.ProcessId -contains $_.ParentProcessId }; if (-not $inner) { $inner = $tune }; Stop-Process -Id $inner.ProcessId -Force; $inner.ProcessId
```
15초 뒤 다시 본다.
```powershell
Start-Sleep 15; Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Select-Object ProcessId,ParentProcessId
```
- **비어 있어야 한다.** 작업자가 스스로 끝나고, 중계 프로세스도 따라 끝난다. 이 컴퓨터에서 다른 python을 돌리고 있지 않다는 가정이다.
- 남아 있으면 그 목록을 kj에게 보내고, `taskkill /PID <번호> /T /F`로 정리한다. 이 경우 튜닝은 시작하지 말고 kj 답을 기다린다.
- 확인이 끝나면 점검 폴더를 지운다.
```powershell
Remove-Item -Recurse -Force orphan_check
```

### A.6 시작 (맡은 제어기 하나만)
작업자 수를 먼저 계산한다. 인자는 작업자 1개당 메모리[GiB]로, **M17은 2.0**(kj 기준), **F13은 1.2**다. F13 값은 맥 실측 0.84 GiB에 여유를 둔 것이다(보고서 19.5절).
```powershell
python -m control.arena_suggest_workers 2.0
```
- 계산식: (가용 메모리 × 0.7) / 작업자당 메모리, CPU − 1, 18 가운데 가장 작은 값이다.
- 하이퍼스레딩 컴퓨터는 논리 CPU가 물리 코어의 2배로 잡힌다. 작업 관리자 > 성능 > CPU의 **'코어'** 값을 `--physical-cores`로 주면 그 값으로 계산한다. 예: `python -m control.arena_suggest_workers 2.0 --physical-cores 6`
- 다른 프로그램을 끈 상태에서 튜닝 직전에 잰다.

M17 시작 명령이다(아래 N을 위에서 나온 수로 바꾼다). F13을 맡은 컴퓨터는 `M17`을 `F13`으로 바꾼다. 인자 안과 파일 이름에 모두 들어 있다.
```powershell
$stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers','M17','--budget','120','--config','configs/arena_v2.json','--run-dir','results/arena/tuning/tune5','--scenario-workers','N' -PassThru -WindowStyle Hidden -RedirectStandardOutput "tune5_M17_$stamp.log" -RedirectStandardError "tune5_M17_$stamp.err"; $p.Id | Out-File -Encoding ascii tune_M17.pid; $p.Id
```
- 창을 닫아도 계속 돈다(`-WindowStyle Hidden`으로 따로 띄운 프로세스라서).
- 튜닝이 도는 동안 **시스템 절전은 코드가 막는다**(Windows `SetThreadExecutionState`, 화면 꺼짐은 막지 않음, 끝나면 자동 해제). 단, 학교 정책의 강제 재시작, Windows 업데이트, 야간 종료, 자동 로그오프는 막지 못한다. 그때는 맨 위 상자대로 재개한다.
- 재개할 때도 같은 명령을 쓴다. 로그 파일 이름에 시각이 붙어 이전 로그를 덮어쓰지 않는다.
- `--scenario-workers`는 결과를 바꾸지 않는다(순차와 비트 동일, 보고서 18·19절). 재개할 때 N을 바꿔도 된다.

### A.7 살아 있는지·메모리 확인
```powershell
Get-Process -Id (Get-Content tune_M17.pid)
```
```powershell
Get-Content (Get-ChildItem tune5_M17_*.log | Sort-Object LastWriteTime | Select-Object -Last 1) -Tail 1
```
```powershell
[math]::Round((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1MB, 1)
```
- 첫 명령이 오류(프로세스 없음)면 멈춘 것이다. `.err` 파일 끝을 kj에게 보내고 재개한다.
- 셋째 줄은 남은 메모리[GiB]다. 1 GiB 밑이면 A.8로 멈추고 N을 줄여 재개한다.

### A.8 멈추기
중계 프로세스까지 **트리 전체**를 끝낸다. venv의 python.exe는 진짜 인터프리터를 자식으로 띄우는 중계 프로그램이라, `Stop-Process`로 그 PID만 죽이면 튜닝이 계속 돈다.
```powershell
taskkill /PID (Get-Content tune_M17.pid) /T /F
```

### A.9 결과 회수 (학교 컴퓨터 → kj)
끝났는지 확인한다. `"status": "complete"`와 `"spent": 120`이 나와야 한다.
```powershell
Select-String '"spent"|"status"' results\arena\tuning\tune5\M17.record.json
```
```powershell
Compress-Archive -Path results\arena\tuning\tune5\M17.record.json, results\arena\tuning\tune5\M17.jsonl, env_check_V13.txt, setup_env_A3.txt, tune5_M17_*.log, tune5_M17_*.err -DestinationPath "tune5_M17_$env:COMPUTERNAME.zip"
```
이 `.zip` 하나를 kj에게 보낸다.

### A.10 합치기 + I-4 검사 (맥에서, V13·F13·M17이 모두 끝난 뒤)
```bash
unzip -j tune5_M17_<컴퓨터이름>.zip M17.record.json M17.jsonl -d results/arena/tuning/tune5/
```
F13(·GSLQR)도 같은 방식으로 푼다. 승계하는 V13·CPID 기록은 retune_v3에서 **복사**한다(바이트 그대로 —
승계 규칙이 기록 파일 sha256을 본다). V13 승계 증명(`python3 -m control.tuning_carryover --controller V13 --run-dir results/arena/tuning/retune_v3`)이 먼저 끝나 있어야 한다.
```bash
cp results/arena/tuning/retune_v3/V13.record.json results/arena/tuning/retune_v3/V13.jsonl results/arena/tuning/retune_v3/CPID.record.json results/arena/tuning/retune_v3/CPID.jsonl results/arena/tuning/tune5/
```
그다음 검사한다(설정은 arena_v2 — 승계 표에 있는 V13·CPID만 옛 설정 해시가 허용된다).
```bash
python3 -m control.arena_tune --summarize --config configs/arena_v2.json --run-dir results/arena/tuning/tune5
```
- `i4_violations`가 빈 목록이어야 한다. 예산, 시나리오, 목적함수, 탐색 방식이 5종 모두 같다는 뜻이다. 위반이 있으면 보고서에 그대로 적고 멈춘다.
- `integrator_limit.flagged`는 보고서의 '결정 필요' 항목으로 올린다.
- 로그와 `env_check_V13.txt`는 커밋하지 않는다(관례). 그 안의 bit_identical과 실측 시간은 보고서에 옮긴다.

### A.11 마무리 (학교 컴퓨터)
```powershell
cd $HOME; Remove-Item -Recurse -Force fast-drone-shard
```
A.1에서 만든 읽기 전용 토큰을 GitHub에서 폐기한다.

---

### A.12 맥에서 미리 시험한 결과(2026-09-27, 당시 Linux용 절차 기준)

절 A 전체(clone→venv→pip install→환경변수→`setup_env.py`→작은 튜닝(예산 3)→결과 회수→
`--summarize`)를 별도 git worktree(가상의 "학교 컴퓨터")로 한 번 돌렸다. `pip install -r
requirements-lock.txt`가 `python==3.13.7` 줄 때문에 실패하는 버그를 여기서 찾아 고쳤다(락 파일에서
그 줄을 빼고 `setup_env.py`가 따로 확인하게 바꿈, 커밋 `54ebfe0`). 고친 뒤에는 전 과정이 통과했고,
사전값 목적함수(0.7847247648921318)가 원래 맥 실행과 정확히 같았다(같은 코드·설정·게인이면 기대되는 값).

## B. 본 실험 분산 — 학교 Windows 컴퓨터들, 태그 `<MAIN_TAG>`(튜닝 확정 뒤 kj가 정함)

> **재시작·로그오프 뒤 이어 돌리기**
> PowerShell을 새로 열고 아래 세 줄을 실행한 다음 **B.6의 시작 명령을 그대로 다시 실행**한다.
> 끝난 시행은 건너뛰고, 끊긴 시행은 처음부터 다시 돈다(시행마다 결과 파일 하나, 임시 파일 → 이름 바꾸기로 저장).
> 같은 컴퓨터에서 같은 조각 번호로만 한다.
> ```powershell
> cd $HOME\fast-drone-main
> Set-ExecutionPolicy -Scope Process Bypass; .\.venv\Scripts\Activate.ps1
> $env:OMP_NUM_THREADS="1"; $env:OPENBLAS_NUM_THREADS="1"; $env:VECLIB_MAXIMUM_THREADS="1"; $env:MKL_NUM_THREADS="1"; $env:PYTHONUTF8="1"
> ```
>
> **진행 확인**(언제든)
> ```powershell
> python -m control.main_distributed status --shards results/main/shards.json --output results/main/runs
> ```
> 조각마다 `끝난 수/전체`와 남은 예상 시간(맥 기준)이 나온다. 숫자가 오래 그대로면 B.7로 프로세스가 살아 있는지 본다.

**규칙(kj)**
- 본 실험은 **전부 Windows**에서 돌린다. 맥 결과와 섞지 않는다(합치기가 플랫폼을 검사해 거부한다).
- **참여하는 모든 컴퓨터는 먼저 B.4(= A.4 재현 점검)를 통과**해야 한다.
- 시행 하나 = (묶음, 시나리오, 제어기). **한 시행은 한 컴퓨터에서 끝까지** 돈다. 조각은 맥에서 예상 시간 기준으로 고르게 나눠 태그에 넣어 둔다(`results/main/shards.json`).
- 조각 하나 = 프로세스 하나(시행을 순차로 돈다). 한 컴퓨터가 조각 여러 개를 맡으려면 프로세스를 여러 개 띄운다 — 개수는 B.5의 작업자 수 계산을 넘기지 않는다(M17 시행이 들어 있어 **프로세스당 2.0 GiB**로 계산).
- 코드는 태그로 받은 깨끗한 트리 그대로 쓴다. 조각 실행 중에 코드를 바꾸면 합치기가 '코드 상태가 섞임'으로 거부한다.
- 교차 확인은 **다른 Windows 컴퓨터**에서 한다(같은 컴퓨터면 거부).

**맥에서 미리 할 일(kj, 태그 전)**
1. `configs/main_experiment.json`의 튜닝 해시·승계 표 해시를 채우고 `python3 -m control.main_experiment --plan`의 가드가 '통과'인지 본다.
2. 조각을 만든다(N = 참여 컴퓨터들의 프로세스 수 합계):
   ```bash
   python3 -m control.main_distributed make-shards --n N --out results/main/shards.json
   ```
3. `results/main/shards.json`을 커밋하고 태그 `<MAIN_TAG>`를 단다. 컴퓨터마다 맡을 조각 번호를 정해 둔다.

### B.1 받기
```powershell
Set-PSReadLineOption -HistorySaveStyle SaveNothing
```
```powershell
cd $HOME
```
```powershell
git clone --branch <MAIN_TAG> --depth 1 https://<READ_ONLY_TOKEN>@github.com/protkjj/fast-drone.git fast-drone-main
```
```powershell
cd fast-drone-main
```
```powershell
git config credential.helper ""
```
```powershell
git describe --tags --exact-match
```
마지막 명령은 `<MAIN_TAG>`를 출력해야 한다.

### B.2 파이썬 환경
A.2와 같다(`py -3.13 -m venv .venv` → `Set-ExecutionPolicy -Scope Process Bypass` → `.\.venv\Scripts\Activate.ps1` →
`python -m pip install -r requirements-lock.txt` → 환경변수 한 줄).

### B.3 설정·모델 점검
```powershell
python scripts\setup_env.py --commit $(git rev-parse HEAD) --config configs/arena_v2.json --expect-config-sha256 9729aec6ff8b54499c96d38c850fedf305970caad5bb2256e948dfa667ea32ca 2>&1 | Tee-Object -FilePath setup_env_B3.txt
```
- 종료코드(`$LASTEXITCODE`)가 0이 아니면 멈추고 `setup_env_B3.txt`를 kj에게 보낸다. 판정 기준은 A.3과 같다(제어기 모델은 계수 rtol 1e-8, sha는 기록만).

### B.4 환경 점검 — V13 기록 재현 (**참여 조건**, FAIL이면 이 컴퓨터는 본 실험에 참여하지 않는다)
A.4와 같은 명령이다. 이미 A.4를 통과한 컴퓨터라도 이 태그로 한 번 더 한다.
```powershell
python -m control.arena_tune_repro --controller V13 --run-dir results/arena/tuning/env_check --index 0 --scenario-workers N 2>&1 | Tee-Object -FilePath env_check_V13.txt
```
- 마지막 줄이 `PASS (rtol 0.001 + identical verdicts)`이면 참여한다. 이 명령의 소요 시간(맥 순차 약 13분)은 예상 시간 보정에 쓴다:
  `speed_ratio = (이 컴퓨터 순차 소요) / (맥 순차 소요)` → 맥에서 `python3 -m control.main_experiment --plan --speed-ratio <값>`.

### B.5 작업자 수(이 컴퓨터가 띄울 프로세스 수)
```powershell
python -m control.arena_suggest_workers 2.0
```
- 다른 프로그램을 끈 상태에서 잰다. 하이퍼스레딩 컴퓨터는 `--physical-cores <코어 수>`를 준다(A.6과 같다).

### B.6 시작 (맡은 조각 번호 I마다 한 번씩)
아래 `I`를 맡은 조각 번호로 바꾼다. 파일 이름에도 들어 있다.
```powershell
$stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.main_distributed','run-shard','--shards','results/main/shards.json','--index','I','--output','results/main/runs' -PassThru -WindowStyle Hidden -RedirectStandardOutput "main_shardI_$stamp.log" -RedirectStandardError "main_shardI_$stamp.err"; $p.Id | Out-File -Encoding ascii main_shardI.pid; $p.Id
```
- 재개할 때도 같은 명령이다(끝난 시행은 건너뛴다). 로그 이름에 시각이 붙어 덮어쓰지 않는다.
- 시작하자마자 거부되면(`refusing to run the shard`) 가드 사유가 `.err`에 있다 — 설정·튜닝 해시가 조각 파일과 다르다는 뜻이니 kj에게 보낸다.

### B.7 살아 있는지·메모리 확인
```powershell
Get-Process -Id (Get-Content main_shardI.pid)
```
```powershell
[math]::Round((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1MB, 1)
```
- 프로세스가 없으면 `.err` 끝을 kj에게 보내고 맨 위 상자대로 재개한다. 남은 메모리가 1 GiB 밑이면 B.8로 하나를 멈춘다.

### B.8 멈추기
```powershell
taskkill /PID (Get-Content main_shardI.pid) /T /F
```
멈춘 시행은 결과 파일이 없어서, 재개하면 처음부터 다시 돈다.

### B.9 결과 회수 (학교 컴퓨터 → kj)
진행 확인에서 맡은 조각이 모두 `끝난 수 = 전체`인지 본 뒤 묶는다(맡은 조각마다 `shardI` 폴더를 넣는다).
```powershell
Compress-Archive -Path results\main\runs\shardI, setup_env_B3.txt, env_check_V13.txt, main_shardI_*.log, main_shardI_*.err -DestinationPath "main_shardI_$env:COMPUTERNAME.zip"
```

### B.10 합치기 (맥, 모든 조각이 모인 뒤)
컴퓨터마다 받은 `.zip`을 `results/main/runs_in/<컴퓨터이름>/`에 푼다. 그다음:
```bash
python3 -m control.main_distributed merge --shards results/main/shards.json --inputs results/main/runs_in/* --out results/main/merged
```
- 마지막 줄이 `MERGED`여야 한다. `MERGE REFUSED`면 사유 목록(누락·중복·모르는 시행·해시 불일치·지문 누락·플랫폼·코드 상태)을 보고서에 적고, 누락 시행은 그 조각을 맡은 컴퓨터에서 재개한다.
- 플랫폼 기본값이 Windows다. 맥 결과가 섞이면 거부된다.

### B.11 교차 확인 (맥 → 다른 Windows 컴퓨터 → 맥)
1. 맥에서 뽑는다(합친 결과에서 돌린 시행의 5%, 고정 시드):
   ```bash
   python3 -m control.main_distributed cross-select --shards results/main/shards.json --merged results/main/merged --out results/main/cross_shards.json
   ```
2. `cross_shards.json`을 **그 시행들을 돌리지 않은 Windows 컴퓨터**로 보낸다(같은 태그로 받은 폴더의 `results\main\`에 넣는다). 그 컴퓨터에서:
   ```powershell
   python -m control.main_distributed run-shard --shards results/main/cross_shards.json --index 0 --output results/main/cross
   ```
   ```powershell
   Compress-Archive -Path results\main\cross -DestinationPath "main_cross_$env:COMPUTERNAME.zip"
   ```
3. 맥에서 `results/main/cross_in/`에 풀고 비교한다:
   ```bash
   python3 -m control.main_distributed cross-compare --cross-shards results/main/cross_shards.json --merged results/main/merged --rerun results/main/cross_in
   ```
   - `CROSS-CHECK PASS`(판정 일치 + rtol 1e-3)여야 한다. 시행마다 `bit_identical`도 기록된다(다른 컴퓨터라 False여도 합격).
   - 같은 컴퓨터가 원래 돌린 시행이 섞이면 그 시행은 'same computer'로 실패한다 — 그 시행만 다른 컴퓨터에서 다시 돌린다.

### B.12 마무리 (학교 컴퓨터)
```powershell
cd $HOME; Remove-Item -Recurse -Force fast-drone-main
```
B.1의 읽기 전용 토큰을 GitHub에서 폐기한다.

### B.13 맥에서 미리 시험한 결과(2026-09-28 밤)
`results/arena/e2_maccheck/REPORT.md` — 조각 실행 → `kill -9` 중단 → 이어 돌리기 → 합치기 → 교차 확인을 실제 시행 46개로 돌렸다.
합치기는 Windows 요구에서 맥 결과를 거부했고, 코드 상태가 섞인 조각도 실제로 잡아 거부했다. 깨끗한 코드로 다시 돌려 46/46 합쳤고,
교차 확인은 같은 컴퓨터라 거부, 맥 점검 전용 옵션으로는 2/2 비트 동일 PASS였다. **Windows에서는 아직 돌려 보지 않았다** —
첫 컴퓨터에서 B.4·B.6이 이 절차 자체의 시험이다.
