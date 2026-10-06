# main7_chain.ps1 - one command per school PC: finish this PC's tune7 tuning at 180, then run its part of the
# main experiment by itself (kj decision 2026-10-06: overlap, no file moves between PCs).
#
# One line (PowerShell), with the controllers this PC tuned:
#   irm https://raw.githubusercontent.com/leo11dk/fast-drone-sensor-fusion/main/school/main7_chain.ps1 -OutFile $HOME\main7_chain.ps1; powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\main7_chain.ps1 -Controllers M17 -Topic <ntfy topic>
#   PC-61: -Controllers M17   PC-62: -Controllers F13   PC-63: -Controllers V13,CPID   PC-64: -Controllers GSLQR
#
# It starts a hidden background process and returns; you can close the window. The background process:
#   1. makes sure each controller's tuning reaches 180 (starts or resumes tune7_all.ps1 -Budget 180;
#      a run still going at 120 is left alone and extended to 180 when it finishes)
#   2. copies the final records to results\arena\tuning\tune7_final\ and writes this PC's main spec
#      (school/main7_make_spec.py: seeds 1000-1019, ladder 1000-1009, ladder only with V13)
#   3. makes shards and runs them in parallel (control.main_distributed run-shard), restarting any that die
#   4. keeps Windows from sleeping while it works (screen may turn off) and pushes progress to the phone
# Re-running the same line after a reboot/logoff resumes everything (finished trials are kept).
# Never edits control/, models/ or configs/ and never moves the git checkout (tuning records pin those hashes).
param(
    [Parameter(Mandatory = $true)][string[]]$Controllers,
    [string]$Topic = '',
    [switch]$Run,
    [int]$PollSeconds = 300
)

$ErrorActionPreference = 'Continue'
$ExpectedHead = '87039e946ce122c9f89bb89a4127c086ac459e90'
$Budget = 180
$Raw = 'https://raw.githubusercontent.com/leo11dk/fast-drone-sensor-fusion/main/school'
$Fds = Join-Path $HOME 'fds'
$Vpy = Join-Path $Fds '.venv\Scripts\python.exe'
$Work = Join-Path $HOME 'main7'
$Gib = @{ M17 = 2.0; F13 = 1.2; V13 = 0.6; GSLQR = 0.6; CPID = 0.6 }
$Order = @('V13', 'M17', 'F13', 'GSLQR', 'CPID')

# accept "V13,CPID" as well as V13 CPID
$Controllers = @($Controllers | ForEach-Object { $_ -split ',' } | ForEach-Object { $_.Trim() } | Where-Object { $_ })
foreach ($c in $Controllers) { if ($Order -notcontains $c) { Write-Host "FAIL: unknown controller $c"; exit 1 } }
$Controllers = @($Order | Where-Object { $Controllers -contains $_ })
$Tag = ($Controllers -join '_')
$Pc = $env:COMPUTERNAME

function Stamp { Get-Date -Format 'MM-dd HH:mm' }
function Say([string]$Text) {
    $line = "[{0}] {1}" -f (Get-Date -Format 'MM-dd HH:mm:ss'), $Text
    Write-Host $line
    if (Test-Path $Work) { Add-Content -Path (Join-Path $Work "chain_$Tag.log") -Value $line }
}
function Send-Note([string]$Message) {
    if ($Topic) {
        try { Invoke-RestMethod -Method Post -Uri "https://ntfy.sh/$Topic" -Body "$Pc main7 $Tag`: $Message $(Stamp)" -TimeoutSec 20 | Out-Null }
        catch { Say "ntfy failed: $($_.Exception.Message)" }
    }
}
function Stop-With([string]$Text) { Say "FAIL: $Text"; Send-Note "FAIL $Text"; exit 1 }

New-Item -ItemType Directory -Force -Path $Work | Out-Null
if (-not (Test-Path $Vpy)) { Stop-With 'fds\.venv not found - run tune7_all.ps1 on this PC first' }

# ---------------- launcher: start the hidden background run and return ----------------
if (-not $Run) {
    $others = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*main7_chain.ps1*-Run*' -and $_.CommandLine -like "*-Controllers $($Controllers -join ',') *" })
    if ($others.Count -gt 0) { Say "main7 chain for $Tag is already running (pid $($others[0].ProcessId)) - nothing to do"; exit 0 }
    $argList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $PSCommandPath, '-Run', '-Controllers', ($Controllers -join ','), '-PollSeconds', "$PollSeconds")
    if ($Topic) { $argList += @('-Topic', $Topic) }
    Start-Process powershell -WindowStyle Hidden -ArgumentList $argList
    Say "main7 chain started in the background for $Tag on $Pc. Log: $Work\chain_$Tag.log. You can close this window."
    exit 0
}

# ---------------- background run ----------------
Set-Location $Fds
$head = (git rev-parse HEAD).Trim()
if ($head -ne $ExpectedHead) { Stop-With "fds is at $head, not tune-final-7 - run tune7_all.ps1 once to restore it" }
$env:OMP_NUM_THREADS = '1'; $env:OPENBLAS_NUM_THREADS = '1'; $env:VECLIB_MAXIMUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'; $env:PYTHONUTF8 = '1'

# keep the system awake while this process lives (screen may still turn off; logoff/reboot still stop it)
try {
    Add-Type -Namespace Main7 -Name Power -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint flags);'
    [Main7.Power]::SetThreadExecutionState([uint32]2147483649) | Out-Null
} catch { Say "could not set keep-awake: $($_.Exception.Message)" }

$Helper = Join-Path $Work 'main7_make_spec.py'
$TuneAll = Join-Path $HOME 'tune7_all.ps1'
try {
    Invoke-WebRequest -Uri "$Raw/main7_make_spec.py" -OutFile $Helper -UseBasicParsing
    Invoke-WebRequest -Uri "$Raw/tune7_all.ps1" -OutFile $TuneAll -UseBasicParsing
} catch { if (-not (Test-Path $Helper) -or -not (Test-Path $TuneAll)) { Stop-With "download failed: $($_.Exception.Message)" } }

$TuneDir = Join-Path $Fds 'results\arena\tuning\tune7'
$FinalDir = Join-Path $Fds 'results\arena\tuning\tune7_final'
$MainDir = Join-Path $Fds 'results\main7'
New-Item -ItemType Directory -Force -Path $FinalDir, $MainDir | Out-Null
$cores = (Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores -Sum).Sum

function Get-Record([string]$C) {
    $p = Join-Path $TuneDir "$C.record.json"
    if (-not (Test-Path $p)) { return $null }
    try { return (Get-Content $p -Raw | ConvertFrom-Json) } catch { return $null }
}
function Test-Final([string]$C) {
    $r = Get-Record $C
    return ($null -ne $r) -and ($r.status -eq 'complete') -and ([int]$r.budget -eq $Budget) -and ([int]$r.spent -eq $Budget)
}
function Test-Tuning([string]$C) {
    $procs = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*arena_tune*--controllers $C *" })
    return ($procs.Count -gt 0)
}
function Start-Tuning([string]$C) {
    Say "starting/resuming $C tuning to $Budget"
    $argList = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $TuneAll, '-Controller', $C, '-Budget', "$Budget")
    if ($Topic) { $argList += @('-Topic', $Topic) }
    # WaitForExit waits for tune7_all itself only (Start-Process -Wait would also wait for the tuning and watcher it starts)
    $p = Start-Process powershell -WindowStyle Hidden -ArgumentList $argList -PassThru
    $p.WaitForExit()
    Say "tune7_all $C exited with $($p.ExitCode)"
}

# ---------------- 1. tuning to 180 ----------------
Send-Note "chain started (tuning to $Budget, then main)"
$restarts = @{}
foreach ($c in $Controllers) { $restarts[$c] = 0 }
while ($true) {
    $pending = @($Controllers | Where-Object { -not (Test-Final $_) })
    if ($pending.Count -eq 0) { break }
    foreach ($c in $pending) {
        if (Test-Tuning $c) { continue }
        $r = Get-Record $c
        $spent = if ($null -ne $r) { [int]$r.spent } else { 0 }
        if ($restarts[$c] -ge 5) { Stop-With "$c tuning keeps stopping at $spent - check this PC" }
        $restarts[$c] += 1
        Start-Tuning $c
    }
    Start-Sleep -Seconds $PollSeconds
}
Say "tuning final at $Budget for $($Controllers -join ', ')"
Send-Note "tuning done at $Budget, preparing main"

# ---------------- 2. final records + spec ----------------
foreach ($c in $Controllers) {
    foreach ($ext in @('record.json', 'jsonl')) {
        $src = Join-Path $TuneDir "$c.$ext"
        $dst = Join-Path $FinalDir "$c.$ext"
        if (-not (Test-Path $dst)) { Copy-Item $src $dst }
        elseif ((Get-FileHash $src).Hash -ne (Get-FileHash $dst).Hash) { Stop-With "$dst differs from the tuning output - do not edit tune7_final by hand" }
    }
}
$specRel = "results/main7/$($Pc)_$Tag.spec.json"
$shardsRel = "results/main7/$($Pc)_$Tag.shards.json"
$outDir = Join-Path $MainDir "$($Pc)_$Tag"
$spec = Join-Path $Fds $specRel
$shards = Join-Path $Fds $shardsRel
if (-not (Test-Path $spec)) {
    & $Vpy $Helper --root $Fds --controllers @Controllers --out $spec *> (Join-Path $Work "spec_$Tag.log")
    if ($LASTEXITCODE -ne 0) { Stop-With "main spec could not be made (see $Work\spec_$Tag.log)" }
}
$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Say "disk free $freeGb GB"
if ($freeGb -lt 5) { Stop-With "only $freeGb GB free on C: - main results need several GB" }
if ($freeGb -lt 20) { Send-Note "WARNING disk free only $freeGb GB (main may need ~10 GB)" }

# ---------------- 3. shards ----------------
$perWorker = ($Controllers | ForEach-Object { $Gib[$_] } | Measure-Object -Maximum).Maximum
$text = (& $Vpy -m control.arena_suggest_workers $perWorker --physical-cores $cores 2>&1 | Out-String)
$workers = 1
if ($text -match 'scenario-workers\s+(\d+)') { $workers = [int]$Matches[1] }
if (-not (Test-Path $shards)) {
    & $Vpy -m control.main_distributed make-shards --spec $specRel --n $workers --out $shardsRel *> (Join-Path $Work "shards_$Tag.log")
    if ($LASTEXITCODE -ne 0) { Stop-With "make-shards refused (see $Work\shards_$Tag.log)" }
}
# per-shard trial counts via Python (Windows PowerShell's ConvertFrom-Json stops at about 2 MB; shards files reach 3 MB)
$counts = (& $Vpy -c "import json,sys; d=json.load(open(sys.argv[1], encoding='utf-8')); print(' '.join(str(len(s['trials'])) for s in d['shards']))" $shards | Out-String).Trim()
$want = @($counts -split '\s+' | Where-Object { $_ } | ForEach-Object { [int]$_ })
$n = $want.Count
if ($n -lt 1) { Stop-With 'could not read the shards file' }
$total = ($want | Measure-Object -Sum).Sum
Say "shards $n, trials $total, out $outDir"
Send-Note "main started: $n workers, $total trials, disk $freeGb GB"

function Get-Done {
    $d = 0
    for ($i = 0; $i -lt $n; $i++) {
        $folder = Join-Path $outDir "shard$i\trials"
        if (Test-Path $folder) {
            $d += @(Get-ChildItem $folder -Filter '*.json' | Where-Object { $_.Name -notlike '*.navigation.json' }).Count
        }
    }
    return $d
}
function Test-Shard([int]$I) {
    $procs = @(Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*run-shard*$($Pc)_$Tag.shards.json*--index $I *" })
    return ($procs.Count -gt 0)
}
function Start-Shard([int]$I) {
    $log = Join-Path $Work "shard_$($Tag)_$I.log"
    $err = Join-Path $Work "shard_$($Tag)_$I.err"
    Start-Process $Vpy -WindowStyle Hidden -RedirectStandardOutput $log -RedirectStandardError $err -ArgumentList '-m', 'control.main_distributed', 'run-shard', '--shards', $shardsRel, '--index', "$I", '--output', "results/main7/$($Pc)_$Tag" | Out-Null
}
function Get-ShardLeft([int]$I) {
    $folder = Join-Path $outDir "shard$I\trials"
    if (-not (Test-Path $folder)) { return $want[$I] }
    $have = @(Get-ChildItem $folder -Filter '*.json' | Where-Object { $_.Name -notlike '*.navigation.json' }).Count
    return ($want[$I] - $have)
}

# ---------------- 4. run and watch ----------------
$shardRestarts = @{}
for ($i = 0; $i -lt $n; $i++) { $shardRestarts[$i] = 0 }
$lastTenth = -1
$lastDone = -1
$lastChange = Get-Date
$stallSent = $false
while ($true) {
    $running = 0
    for ($i = 0; $i -lt $n; $i++) {
        if ((Get-ShardLeft $i) -le 0) { continue }
        if (Test-Shard $i) { $running += 1; continue }
        if ($shardRestarts[$i] -ge 5) { Send-Note "shard $i keeps stopping - see $Work\shard_$($Tag)_$i.err"; $shardRestarts[$i] = -100; continue }
        if ($shardRestarts[$i] -lt 0) { continue }
        if ($shardRestarts[$i] -gt 0) { Say "restarting shard $i (attempt $($shardRestarts[$i]))" }
        $shardRestarts[$i] += 1
        Start-Shard $i
        $running += 1
    }
    $done = Get-Done
    if ($done -ne $lastDone) { $lastDone = $done; $lastChange = Get-Date; $stallSent = $false }
    $tenth = [math]::Floor(10 * $done / [math]::Max(1, $total))
    if ($tenth -gt $lastTenth) {
        if ($lastTenth -ge 0) { Send-Note "main $done/$total trials" }
        $lastTenth = $tenth
    }
    if ($done -ge $total) { break }
    if ($running -eq 0) { Stop-With "no shard is running and $($total - $done) trials are left - see $Work" }
    if (((New-TimeSpan -Start $lastChange -End (Get-Date)).TotalMinutes -gt 180) -and -not $stallSent) {
        Send-Note "no new main trial for 3 h - check this PC"; $stallSent = $true
    }
    Start-Sleep -Seconds $PollSeconds
}
$freeGb = [math]::Round((Get-PSDrive C).Free / 1GB, 1)
Say "main finished: $done/$total trials in $outDir (disk free $freeGb GB)"
Send-Note "MAIN DONE $done/$total trials, results in fds\results\main7\$($Pc)_$Tag"
exit 0
