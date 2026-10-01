# tune7 start: begin (or resume) one controller's tuning on this PC, optionally with phone notifications.
# Usage: powershell -NoProfile -ExecutionPolicy Bypass -File $HOME\tune7_start.ps1 -Controller M17 [-Workers 3] [-Topic <ntfy topic>] [-Budget 120]
# Run the same command again to resume after a reboot (same PC only). Workers are computed from free memory if not given.
param(
    [Parameter(Mandatory = $true)][ValidateSet('M17', 'F13', 'V13', 'GSLQR', 'CPID')][string]$Controller,
    [int]$Workers = 0,
    [string]$Topic = '',
    [int]$Budget = 120
)

$fds = Join-Path $HOME 'fds'
$py = Join-Path $fds '.venv\Scripts\python.exe'
if (-not (Test-Path $py)) { Write-Host 'FAIL: $HOME\fds\.venv not found'; exit 1 }
Set-Location $fds
$tag = (git describe --tags --exact-match 2>$null)
if ($tag -ne 'tune-final-7') { Write-Host "FAIL: fds is not at tag tune-final-7 (got '$tag')"; exit 1 }

$env:OMP_NUM_THREADS = '1'; $env:OPENBLAS_NUM_THREADS = '1'; $env:VECLIB_MAXIMUM_THREADS = '1'
$env:MKL_NUM_THREADS = '1'; $env:PYTHONUTF8 = '1'

if ($Workers -le 0) {
    $gib = @{ M17 = '2.0'; F13 = '1.2'; V13 = '0.6'; GSLQR = '0.6'; CPID = '0.6' }[$Controller]
    $cores = (Get-CimInstance Win32_Processor | Measure-Object -Property NumberOfCores -Sum).Sum
    $text = (& $py -m control.arena_suggest_workers $gib --physical-cores $cores 2>&1 | Out-String)
    $w = 1
    if ($text -match 'scenario-workers\s+(\d+)') { $w = [int]$Matches[1] }
    if ($Controller -eq 'GSLQR') { $w = [math]::Max(1, [math]::Ceiling($w / 2)) }
    if ($Controller -eq 'CPID') { $w = [math]::Max(1, [math]::Floor($w / 2)) }
    $Workers = $w
}

$stamp = Get-Date -Format yyyyMMdd_HHmm
$log = Join-Path $fds "tune7_${Controller}_$stamp.log"
$err = Join-Path $fds "tune7_${Controller}_$stamp.err"
$p = Start-Process $py -ArgumentList '-m', 'control.arena_tune', '--controllers', $Controller, '--budget', "$Budget", '--config', 'configs/arena_tune7.json', '--run-dir', 'results/arena/tuning/tune7', '--scenario-workers', "$Workers" -PassThru -WindowStyle Hidden -RedirectStandardOutput $log -RedirectStandardError $err
$p.Id | Out-File -Encoding ascii (Join-Path $fds "tune_$Controller.pid")
Write-Host "started $Controller budget $Budget workers $Workers pid $($p.Id)"
Write-Host "log $log"
Write-Host "stop:  taskkill /PID $($p.Id) /T /F"

if ($Topic) {
    $watcher = Join-Path $HOME 'tune7_notify.ps1'
    if (Test-Path $watcher) {
        Start-Process powershell -WindowStyle Hidden -ArgumentList '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $watcher, '-Controller', $Controller, '-Topic', $Topic
        Write-Host 'phone watcher started (expect "watcher started" on the phone)'
    } else {
        Write-Host "no $watcher - phone notifications skipped"
    }
}
