<#
.SYNOPSIS
  Execute remaining wall30x3 Original SMARM B-sweep captures (22 runs).
.DESCRIPTION
  Reuses phase2r_noattest_r1 and phase3r_B2048_r1. Never overwrites existing tags
  or legacy phase2w_/phase3w_ logs. Stops on validation failure.
#>
$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
$Root = (Resolve-Path (Join-Path $here "..\..\..")).Path
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$BuildRoot = Join-Path $Root "thesis\03_rt_experiment\builds"

function Invoke-BuildProfile {
  param([string]$Mode)
  # gcc warnings on stderr must not abort under ErrorAction Stop
  $prev = $ErrorActionPreference
  $ErrorActionPreference = "Continue"
  try {
    if ($Mode -eq "noattest") {
      & (Join-Path $here "build_rt_profile.ps1") -Profile 10Hz10Hz -RunLen wall30
    } else {
      $B = [int]($Mode.Substring(1))
      & (Join-Path $here "build_rt_profile.ps1") -Profile 10Hz10Hz -RunLen wall30 -WithOriginalSmarm -B $B
    }
    if ($LASTEXITCODE -ne 0) { throw "build failed $Mode exit=$LASTEXITCODE" }
  } finally {
    $ErrorActionPreference = $prev
  }
}

function Test-RunPresent([string]$tag) {
  return (Test-Path (Join-Path $LogDir "${tag}_uart_capture.txt"))
}

function Assert-NoClobber([string]$tag) {
  if (Test-RunPresent $tag) {
    throw "REFUSE overwrite existing tag $tag"
  }
  foreach ($leg in @("phase2w_", "phase3w_", "phase2_10Hz10Hz_", "phase3_10Hz10Hz_")) {
    if ($tag.StartsWith($leg.TrimEnd("_"))) { throw "REFUSE legacy tag $tag" }
  }
}

function Confirm-Build([string]$mode) {
  if ($mode -eq "noattest") {
    $dir = Join-Path $BuildRoot "10Hz10Hz_wall30_noattest"
    $needS = $false
    $expectB = $null
    $expectP3 = "0"
  } else {
    $B = [int]($mode.Substring(1))
    $dir = Join-Path $BuildRoot "10Hz10Hz_wall30_original_B$B"
    $needS = $true
    $expectB = "$B"
    $expectP3 = "1"
  }
  if (!(Test-Path (Join-Path $dir "NonSecure.elf"))) { throw "Missing NS ELF $dir" }
  if ($needS -and !(Test-Path (Join-Path $dir "Secure.elf"))) { throw "Missing Secure ELF $dir" }
  $meta = Get-Content (Join-Path $dir "run_meta.txt") -Raw
  if ($meta -notmatch "PHASE3_WITH_ORIGINAL_SMARM=$expectP3") {
    throw "PHASE3 mismatch in $dir"
  }
  if ($meta -notmatch "run=wall30") { throw "run!=wall30 in $dir" }
  if ($needS) {
    $cfg = Get-Content (Join-Path $dir "attest_overhead_cfg.h") -Raw
    if ($cfg -notmatch "USE_SAU_APPROACH\s+0") { throw "SAU!=0 $dir" }
    if ($cfg -notmatch "USE_SNAP\s+0") { throw "SNAP!=0 $dir" }
    if ($cfg -notmatch "ATTEST_BLOCK_SIZE\s+$expectB\b") { throw "B mismatch cfg $dir" }
    if ($meta -notmatch "ATTEST_BLOCK_SIZE=$expectB") { throw "B mismatch meta $dir" }
  }
  Write-Host "BUILD_OK $mode -> $dir"
  return $dir
}

function Assert-CaptureOk([string]$tag, [string]$mode) {
  $uart = Join-Path $LogDir "${tag}_uart_capture.txt"
  $sonar = Join-Path $LogDir "${tag}_sonar_timing.csv"
  $wheel = Join-Path $LogDir "${tag}_wheel_timing.csv"
  foreach ($p in @($uart, $sonar, $wheel)) {
    if (!(Test-Path $p)) { throw "MISSING $p" }
    if ((Get-Item $p).Length -lt 100) { throw "TOO_SMALL $p" }
  }
  $t = Get-Content $uart -Raw
  if ($t -notmatch "warmup_us=2000000") { throw "$tag bad warmup" }
  if ($t -notmatch "duration_us=30000000") { throw "$tag bad duration" }
  if ($t -notmatch "T0=(\d+).*Tend=(\d+)") {
    # Tend may be on next lines; use separate
  }
  $m0 = [regex]::Match($t, "T0=(\d+)")
  $m1 = [regex]::Match($t, "Tend=(\d+)")
  if (-not $m0.Success -or -not $m1.Success) { throw "$tag missing T0/Tend" }
  $delta = [int64]$m1.Groups[1].Value - [int64]$m0.Groups[1].Value
  if ($delta -ne 30000000) { throw "$tag TIM2 delta=$delta != 30000000" }
  if ($t -notmatch "sonar_ovf=0") { throw "$tag sonar_ovf nonzero" }
  if ($t -notmatch "wheel_ovf=0") { throw "$tag wheel_ovf nonzero" }
  if ($t -notmatch "Idle\.") { throw "$tag missing Idle" }
  if ($t -match "HardFault|SecureFault") { throw "$tag fault marker" }
  if ($mode -eq "noattest") {
    if ($t -notmatch "No-attest") { throw "$tag expected No-attest banner" }
  } else {
    $B = $mode.Substring(1)
    if ($t -notmatch "B=$B") { throw "$tag expected B=$B" }
    if ($t -notmatch "USE_SAU=0") { throw "$tag USE_SAU!=0" }
    if ($t -notmatch "USE_SNAP=0") { throw "$tag USE_SNAP!=0" }
  }
  # sensor: allow empty status column check via python quickly
  $py = @"
import csv
from pathlib import Path
sp=Path(r'$sonar'); wp=Path(r'$wheel')
sr=list(csv.DictReader(sp.open(encoding='utf-8')))
wr=list(csv.DictReader(wp.open(encoding='utf-8')))
assert len(sr)>0 and len(wr)>0
err=sum(1 for r in sr if int(r.get('status','0'))!=0)
print(f'rows={len(sr)}/{len(wr)} sonar_status_err={err}')
if err: raise SystemExit('sensor_status_errors')
"@
  python -c $py
  if ($LASTEXITCODE -ne 0) { throw "$tag sensor validation failed" }
  Write-Host "VALIDATE_OK $tag"
}

function Invoke-One([string]$mode, [int]$run) {
  $tag = if ($mode -eq "noattest") { "phase2r_noattest_r$run" } else { "phase3r_$($mode)_r$run" }
  Assert-NoClobber $tag
  Confirm-Build $mode | Out-Null
  Write-Host "==== CAPTURE $mode run=$run tag=$tag ===="
  $prev = $ErrorActionPreference
  $ErrorActionPreference = "Continue"
  try {
    & (Join-Path $here "capture_wall30x3.ps1") -Mode $mode -Run $run
    if ($LASTEXITCODE -ne 0) { throw "capture failed $tag exit=$LASTEXITCODE" }
  } finally {
    $ErrorActionPreference = $prev
  }
  Assert-CaptureOk $tag $mode
}

# Plan: rebuild each config then missing runs only
$plan = @(
  @{ Mode = "noattest"; Rebuild = $true; Runs = @(2, 3) },
  @{ Mode = "B64"; Rebuild = $true; Runs = @(1, 2, 3) },
  @{ Mode = "B128"; Rebuild = $true; Runs = @(1, 2, 3) },
  @{ Mode = "B256"; Rebuild = $true; Runs = @(1, 2, 3) },
  @{ Mode = "B512"; Rebuild = $true; Runs = @(1, 2, 3) },
  @{ Mode = "B1024"; Rebuild = $true; Runs = @(1, 2, 3) },
  @{ Mode = "B2048"; Rebuild = $true; Runs = @(2, 3) },
  @{ Mode = "B4096"; Rebuild = $true; Runs = @(1, 2, 3) }
)

$done = @()
foreach ($step in $plan) {
  $mode = $step.Mode
  Write-Host "`n######## CONFIG $mode ########"
  if ($step.Rebuild) {
    Invoke-BuildProfile $mode
  }
  Confirm-Build $mode | Out-Null
  foreach ($r in $step.Runs) {
    Invoke-One $mode $r
    $done += "$mode/r$r"
  }
}

Write-Host "`nCAMPAIGN_CAPTURES_DONE count=$($done.Count)"
$done | ForEach-Object { Write-Host "  $_" }
