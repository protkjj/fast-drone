# 학교 전산실 — 센서 설정 튜닝 `tune-final-7` 점검 + 시작 (2026-10-01 저녁)

**계획 A**: 태그 `tune-final-7`이 있으면 이 문서대로 점검 후 튜닝 시작.
**계획 B**: 태그가 없으면 `results/SCHOOL_CHECK_SENSOR_2026-10-01.md`(1~11, `a44c718`)로 점검만 하고 튜닝은 시작하지 않는다.

- 저장소: `https://github.com/leo11dk/fast-drone-sensor-fusion` (공개, 토큰 불필요)
- 태그: `tune-final-7` = 커밋 `87039e946ce122c9f89bb89a4127c086ac459e90`(태그 객체 `074fd616…`), 동욱님 저장소 `main`과 같음
- 설정: `configs/arena_tune7.json`, config_sha256 = `e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801`
- 배분(kj): 컴퓨터 1 = M17, 2 = F13, 3 = V13, 4 = GSLQR + CPID. F13·M17은 다른 컴퓨터. 한 제어기는 한 컴퓨터에서 끝까지.
- 명령은 PowerShell에서 한 줄씩. FAIL이면 그 컴퓨터는 멈추고 화면·파일을 kj에게. 다른 컴퓨터는 계속해도 된다.
- 한 컴퓨터 안에서는 무거운 단계를 동시에 돌리지 않는다(시간 측정이 틀어진다).
- 숫자(코어 8, 작업자 수)는 **PC-63(i7-10700 물리 코어 8, 메모리 15.9 GB, 여유 8.9 GB)** 기준으로 채웠다. 전산실 컴퓨터가 같은 사양이면 그대로 복사해 쓴다.
  사양이 다르면(2번 결과의 `NumberOfCores`나 여유 메모리가 다르면) 5번을 그 컴퓨터 값으로 다시 돌려 작업자 수를 바꾼다.
- `<주제>`만 자리표시로 남겼다(휴대폰 알림 주제 이름 — 저장소에 적지 않음). 그대로 치지 말고 kj의 주제 이름으로 바꾼다.

## ★★ 한 줄로 끝 (2026-10-01 저녁, 가장 권장)
PowerShell을 열고 **그 컴퓨터가 맡은 제어기 줄 하나만** 붙여 넣는다. Python 설치·받기·가상환경·점검·튜닝 시작·휴대폰 알림을 스크립트가 다 한다(동욱님 저장소 `school/tune7_all.ps1`, 공개).
`<주제>`만 kj 알림 주제 이름으로 바꾼다.

- 컴퓨터 1 (M17):
```powershell
irm https://raw.githubusercontent.com/leo11dk/fast-drone-sensor-fusion/main/school/tune7_all.ps1 -OutFile $HOME\tune7_all.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\tune7_all.ps1 -Controller M17 -Topic <주제>
```
- 컴퓨터 2 (F13): 위 줄에서 `M17` → `F13`
- 컴퓨터 3 (V13): `M17` → `V13`
- 컴퓨터 4: `GSLQR`로 한 번 실행하고, 끝나면 `CPID`로 한 번 더(두 번째는 점검을 건너뛰고 바로 시작)

동작 요약
- 점검(약 15~25분)에서 하나라도 FAIL이면 **튜닝을 시작하지 않고** 휴대폰에 FAIL 알림 → `fds\tune7_check_<PC>_<시각>\` 폴더를 kj에게.
- ALL PASS면 바로 튜닝 시작 + 감시(첫 평가·30회마다·완료·2시간 정체 알림). 창을 닫아도 계속 돈다.
- **재부팅·로그오프 뒤에는 같은 한 줄을 다시** 붙여 넣으면 된다(이미 통과한 점검·설치는 건너뛰고 튜닝만 이어서, 이미 돌고 있으면 두 번 시작하지 않음).
- 이미 아래 절차로 5번까지 한 컴퓨터도 이 한 줄을 그대로 쓰면 된다(있는 `fds`·`.venv`를 재사용).
- ⚠️ 맥에서 만들어 PowerShell로 직접 시험하지 못했다. 오류 화면이 나오면 그대로 kj에게.

---

## ★ 빠른 절차 — 스크립트 두 개로 (2026-10-01 추가, 권장)
명령을 하나씩 치지 않는다. 점검 스크립트가 태그·설정·작업자 수·재현 3종·고아 작업자를 **모두 돌리고 결과를 파일로 남긴 뒤** 맨 끝에 `ALL PASS`/`FAIL`과 이 컴퓨터용 시작 명령을 보여 준다.

**A. Python 3.13·Git이 없으면** 아래 0번만 먼저 한다.
**B. 받기 + 가상환경**(컴퓨터마다 한 번):
```powershell
cd $HOME; git clone https://github.com/leo11dk/fast-drone-sensor-fusion.git fds; cd $HOME\fds; git checkout tune-final-7
```
```powershell
py -3.13 -m venv .venv; .\.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
```
(이미 `fds`·`.venv`가 있으면 B는 건너뛴다.)

**C. 스크립트 세 개 받기**(브라우저, GitHub 로그인 상태): 아래 각 주소 → **Raw** → `Ctrl+S` → `C:\Users\USER\` 에 같은 이름으로 저장(**`fds` 폴더 안에 두지 않는다**)
- `https://github.com/protkjj/fast-drone/blob/protkjj/arena-completion/results/tune7_school_check.ps1`
- `https://github.com/protkjj/fast-drone/blob/protkjj/arena-completion/results/tune7_start.ps1`
- `https://github.com/protkjj/fast-drone/blob/protkjj/arena-completion/results/tune7_notify.ps1`
```powershell
Unblock-File $HOME\tune7_school_check.ps1, $HOME\tune7_start.ps1, $HOME\tune7_notify.ps1
```

**D. 점검**(약 15~25분, 끝나면 휴대폰에 `ALL PASS`/`FAIL` 알림):
```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\tune7_school_check.ps1 -Topic <주제>
```
- 결과: `$HOME\fds\tune7_check_<컴퓨터>_<시각>\summary.json`(+ 각 단계 로그). FAIL이면 그 폴더를 kj에게.

**E. 튜닝 시작**(ALL PASS인 컴퓨터만, 맡은 제어기로 — 작업자 수·환경변수·알림 감시를 스크립트가 알아서):
```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\tune7_start.ps1 -Controller M17 -Topic <주제>
```
- 컴퓨터 2는 `F13`, 3은 `V13`, 4는 `GSLQR`을 먼저 실행하고 이어서 `CPID`.
- 재시작·로그오프 뒤 재개도 **같은 명령**(같은 컴퓨터).
- `<주제>`는 kj 휴대폰 알림 주제 이름으로 바꾼다(저장소에 적지 않음).
- ⚠️ 이 스크립트들은 맥에서 만들었고 PowerShell로 시험해 보지 못했다. 오류가 나면 화면을 그대로 kj에게 보내고, 그동안은 아래 수동 절차를 쓴다.

---

## (예비) 수동 절차 — 스크립트가 막힐 때만

## 0. 준비 확인 (컴퓨터마다 맨 처음) — 2026-10-01 추가: 학교 PC에 Python이 없었음
```powershell
git --version
```
```powershell
py -0
```
- `git`이 없으면 Git for Windows를 설치한다(https://git-scm.com/download/win, 기본값으로 설치).
- `py -0` 목록에 `-V:3.13`이 없거나 `py`가 없으면 **Python 3.13.7**을 사용자 계정에 설치한다(관리자 권한 불필요):
```powershell
Invoke-WebRequest https://www.python.org/ftp/python/3.13.7/python-3.13.7-amd64.exe -OutFile $HOME\python-3.13.7-amd64.exe
```
```powershell
Start-Process $HOME\python-3.13.7-amd64.exe -ArgumentList '/quiet','InstallAllUsers=0','PrependPath=0','Include_launcher=1','Include_test=0' -Wait
```
```powershell
py -3.13 --version
```
→ `Python 3.13.7`. `py`를 못 찾으면 PowerShell 창을 닫고 새로 연다. 다운로드가 막히면 브라우저로 python.org에서 3.13.7 "Windows installer (64-bit)"를 받아 **Install Now**.
- ⚠️ **재부팅하면 설치·파일이 지워지는 복원 프로그램**(딥프리즈, 하드디스크 보호 등)이 있는지 확인한다. 있으면 튜닝 기록(`fds\results\…\tune7`)이 사라질 수 있으니 **튜닝 시작 전에 kj에게 알린다**.

## 1. 받기
```powershell
cd $HOME
```
```powershell
git clone https://github.com/leo11dk/fast-drone-sensor-fusion.git fds
```
(이미 `fds`가 있으면 위 대신 `cd $HOME\fds` → `git fetch origin --tags`)
```powershell
cd $HOME\fds
```
```powershell
git checkout tune-final-7
```
```powershell
git describe --tags --exact-match
```
→ `tune-final-7`이 나와야 한다. `git rev-parse HEAD`는 `87039e946ce122c9f89bb89a4127c086ac459e90`이어야 한다.

## 2. 사양 기록
```powershell
& { "computer: $env:COMPUTERNAME"; (Get-CimInstance Win32_OperatingSystem).Caption; Get-CimInstance Win32_Processor | Format-List Name,NumberOfCores,NumberOfLogicalProcessors | Out-String; "total_mem_GiB: " + [math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory/1GB,1); "free_mem_GiB: " + [math]::Round((Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory/1MB,1) } | Tee-Object -FilePath check_spec.txt
```
`NumberOfCores`(물리 코어)가 8이 아니면 아래 5번의 `8`을 그 값으로 바꾼다.

## 3. 파이썬 환경 (이미 `.venv`가 있으면 활성화·환경변수 줄만)
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
```powershell
$env:OMP_NUM_THREADS="1"; $env:OPENBLAS_NUM_THREADS="1"; $env:VECLIB_MAXIMUM_THREADS="1"; $env:MKL_NUM_THREADS="1"; $env:PYTHONUTF8="1"
```

## 4. 설정·모델 점검
```powershell
python scripts\setup_env.py --commit $(git rev-parse HEAD) --config configs\arena_tune7.json --expect-config-sha256 e7ef609705ea447c2082cf063517a2a1b31598421f624851d1a81911c36f8801 2>&1 | Tee-Object -FilePath check_setup_env.txt
```
→ 마지막 줄 `실행해도 좋음(PASS)`. 패키지 WARN은 괜찮다.

## 5. 작업자 수
```powershell
python -m control.arena_suggest_workers 0.6 --physical-cores 8
```
```powershell
python -m control.arena_suggest_workers 2.0 --physical-cores 8
```
PC-63 결과: 0.6 → **7**, 2.0 → **3**(F13용 1.2 → **5**). 같은 사양이면 아래 숫자 그대로. 다르면 아래 작업자 수를 그 결과로 바꾼다.

## 6. 재현 점검 (하나라도 FAIL이면 이 컴퓨터는 튜닝하지 않는다)
센서 재현 기준 두 개(각 약 3분):
```powershell
$t = Measure-Command { python scripts\sensor_reproduce.py --output results\tune7_repro_projected *> check_repro_projected.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_repro_projected_time.txt
```
```powershell
$t = Measure-Command { python scripts\sensor_reproduce.py --reference scripts\data\sensor_legacy_reproduction_tune7.json --output results\tune7_repro_legacy *> check_repro_legacy.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_repro_legacy_time.txt
```
재현 기준 판정 확인 — 결과 폴더의 `comparison.json`(합격 = `"reproduction_pass": true`, `"problems": []`. `trajectory_bit_identical`은 false여도 됨):
```powershell
Select-String '"reproduction_pass"|"trajectory_bit_identical"|"problems"' results\tune7_repro_projected\comparison.json
```
```powershell
Select-String '"reproduction_pass"|"trajectory_bit_identical"|"problems"' results\tune7_repro_legacy\comparison.json
```
- ⚠️ 위 재현 명령을 **다시 돌리면** 결과 폴더가 이미 있어서 `reproduction output must be new or empty`로 멈춘다(로그 파일도 이 오류로 덮어써짐). 다시 할 때는 폴더 이름을 바꾼다(예: `results\tune7_repro_projected_2`).
센서 포함 튜닝 경로(맥 순차 약 18분, 작업자 3개로 약 6~10분 — 재현 점검은 작업자 3개 상한):
```powershell
$t = Measure-Command { python -m control.arena_tune_repro --config configs/arena_tune7.json --controller V13 --run-dir results/arena/tuning/env_check_tune7 --index 0 --scenario-workers 3 *> check_env_tune7.txt }; "$($t.TotalSeconds) s" | Tee-Object -FilePath check_env_tune7_time.txt
```
```powershell
Get-Content check_env_tune7.txt -Tail 2
```
- 합격: 재현 기준은 판정 일치 + rtol 1e-3(해시 검사 포함 통과, legacy는 원래 '모델 범위 실패'가 똑같이 나와야 함). 튜닝 경로는 마지막 줄 `PASS (rtol 0.001 + identical verdicts)`. `bit-identical` False여도 합격.
- 시간은 **벽시계**로 적는다(센서 설정은 시행마다 사전 구간 약 12 s가 붙는데 기록의 `wall_seconds`에는 안 잡힘). 맥 순차: 튜닝 경로 V13 1,064 s.

## 7. 고아 작업자 점검 (컴퓨터마다 한 번, 1~2분)
`results/SCHOOL_CHECK_SENSOR_2026-10-01.md` 9번 그대로(이 시험의 PID 기준, `left:` 비어 있어야 합격).

## 8. 튜닝 시작 (그 컴퓨터가 맡은 제어기 줄만 실행)
작업자 수(PC-63 기준): M17 **3**, F13 **5**, V13 **7**, 컴퓨터 4는 GSLQR **4** + CPID **3**.

**컴퓨터 1 — M17**
```powershell
$c = 'M17'; $stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers',$c,'--budget','120','--config','configs/arena_tune7.json','--run-dir','results/arena/tuning/tune7','--scenario-workers','3' -PassThru -WindowStyle Hidden -RedirectStandardOutput "tune7_${c}_$stamp.log" -RedirectStandardError "tune7_${c}_$stamp.err"; $p.Id | Out-File -Encoding ascii "tune_$c.pid"; $p.Id
```
**컴퓨터 2 — F13**
```powershell
$c = 'F13'; $stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers',$c,'--budget','120','--config','configs/arena_tune7.json','--run-dir','results/arena/tuning/tune7','--scenario-workers','5' -PassThru -WindowStyle Hidden -RedirectStandardOutput "tune7_${c}_$stamp.log" -RedirectStandardError "tune7_${c}_$stamp.err"; $p.Id | Out-File -Encoding ascii "tune_$c.pid"; $p.Id
```
**컴퓨터 3 — V13**
```powershell
$c = 'V13'; $stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers',$c,'--budget','120','--config','configs/arena_tune7.json','--run-dir','results/arena/tuning/tune7','--scenario-workers','7' -PassThru -WindowStyle Hidden -RedirectStandardOutput "tune7_${c}_$stamp.log" -RedirectStandardError "tune7_${c}_$stamp.err"; $p.Id | Out-File -Encoding ascii "tune_$c.pid"; $p.Id
```
**컴퓨터 4 — GSLQR과 CPID(두 줄 모두)**
```powershell
$c = 'GSLQR'; $stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers',$c,'--budget','120','--config','configs/arena_tune7.json','--run-dir','results/arena/tuning/tune7','--scenario-workers','4' -PassThru -WindowStyle Hidden -RedirectStandardOutput "tune7_${c}_$stamp.log" -RedirectStandardError "tune7_${c}_$stamp.err"; $p.Id | Out-File -Encoding ascii "tune_$c.pid"; $p.Id
```
```powershell
$c = 'CPID'; $stamp = Get-Date -Format yyyyMMdd_HHmm; $p = Start-Process .\.venv\Scripts\python.exe -ArgumentList '-m','control.arena_tune','--controllers',$c,'--budget','120','--config','configs/arena_tune7.json','--run-dir','results/arena/tuning/tune7','--scenario-workers','3' -PassThru -WindowStyle Hidden -RedirectStandardOutput "tune7_${c}_$stamp.log" -RedirectStandardError "tune7_${c}_$stamp.err"; $p.Id | Out-File -Encoding ascii "tune_$c.pid"; $p.Id
```
- 진행 확인(예: M17 — 다른 제어기는 이름만 바꿈). 첫 평가가 끝나면(`spent` 1) 벽시계 시간을 kj에게:
```powershell
Select-String '"spent"|"status"' results\arena\tuning\tune7\M17.record.json; (Get-Item results\arena\tuning\tune7\M17.jsonl).LastWriteTime
```
- 튜닝 중 `fds`의 `control/`·`models/team_light/control/`·`configs/`에 아무것도 추가·수정하지 않는다. 결과·로그 저장은 괜찮다.
- 멈추기(예: M17): `taskkill /PID (Get-Content tune_M17.pid) /T /F`. 재개: 그 컴퓨터의 8번 명령을 그대로.
- 재시작·로그오프 뒤: 새 PowerShell → `cd $HOME\fds; Set-ExecutionPolicy -Scope Process Bypass; .\.venv\Scripts\Activate.ps1` → 3번 환경변수 줄 → 8번 명령.

## 9. 점검 결과 묶기 (튜닝 시작 뒤, 컴퓨터마다)
```powershell
Compress-Archive -Path check_*.txt, results\tune7_repro_projected, results\tune7_repro_legacy -DestinationPath "tune7_check_$env:COMPUTERNAME.zip"
```

## 10. 휴대폰 알림 (ntfy, 튜닝 시작 뒤 컴퓨터마다)
알림: 감시 시작 / 첫 평가 완료 / 30회마다 / **120회 완료** / **2시간 넘게 새 평가 없음**(재시작·멈춤 의심). 메시지에는 제어기 이름·횟수·시각만 들어간다(ntfy.sh는 공개 서버).

**휴대폰(한 번만)**: ntfy 앱 설치 → `+` → 주제 이름 입력 → 구독. 주제는 남이 못 맞힐 이름으로(예: `kj-tune7-` 뒤에 무작위 글자 8개). 아래 `<주제>`에 그 이름을 쓴다.

**컴퓨터마다**
1. 브라우저(GitHub 로그인)로 `https://github.com/protkjj/fast-drone/blob/protkjj/arena-completion/results/tune7_notify.ps1` → **Raw** → `Ctrl+S` → `C:\Users\USER\tune7_notify.ps1`로 저장(PC-63 사용자 이름 `USER` 기준)(**`fds` 폴더 안에 두지 않는다**).
2. 내려받은 파일 차단 해제:
```powershell
Unblock-File $HOME\tune7_notify.ps1
```
3. 시험 알림 — 휴대폰에 `notify test OK`가 와야 한다(안 오면 `$HOME\tune7_notify_M17.err`처럼 그 제어기 이름의 파일을 본다. 학교망이 막으면 알림은 포기하고 튜닝은 그대로 둔다):
```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\tune7_notify.ps1 -Controller M17 -Topic <주제> -Test
```
3·4번 명령의 `M17`은 그 컴퓨터가 맡은 제어기로 바꾼다(F13 / V13 / GSLQR / CPID).

4. 감시 시작(창을 닫아도 계속 돈다, 컴퓨터 4는 `GSLQR`, `CPID`로 두 번):
```powershell
Start-Process powershell -WindowStyle Hidden -ArgumentList '-NoProfile','-ExecutionPolicy','Bypass','-File',"$HOME\tune7_notify.ps1",'-Controller','M17','-Topic','<주제>'
```
- 휴대폰에 `watcher started`가 오면 된다.
- 컴퓨터가 재시작되면 튜닝 재개(8번)와 함께 4번도 다시 한다.
- 예산 연장(180회)으로 이어 돌릴 때도 4번을 다시 한다(120회 완료에서 감시가 끝나므로).

## 판정표 (컴퓨터마다)
| 컴퓨터 | 맡은 제어기 | 4 점검 | 6 재현 기준 2개 | 6 튜닝 경로 | 7 고아 | 작업자 N | 6 튜닝 경로 벽시계 | 첫 평가 벽시계 |
|---|---|---|---|---|---|---|---|---|
| 1 | M17 | | | | | | | |
| 2 | F13 | | | | | | | |
| 3 | V13 | | | | | | | |
| 4 | GSLQR+CPID | | | | | | | |
