<#
.SYNOPSIS
  EnergySweep_20261024_A — SENSOR-ATTACHED SYNTHETIC-TASK GROSS BOARD ENERGY
  Builds/flashes/captures FNB58+UART for the 21-cell matrix (+ optional Original B1024 drift checks).
#>
param(
  [switch]$SkipBuildIfPresent,
  [switch]$DriftChecksOnly,
  [string]$OnlyMode = "",
  [int]$OnlyB = 0,
  [int]$TimeoutBaseSec = 300
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$BuildScript = Join-Path $PSScriptRoot "build_energy_pilot.ps1"
$SyncPy = Join-Path $Root "thesis\03_rt_experiment\energy_fnb58\scripts\fnb58_sync_capture.py"
$IntegPy = Join-Path $Root "thesis\03_rt_experiment\energy_fnb58\scripts\fnb58_integrate_rounds.py"
$CampRoot = Join-Path $Root "thesis\03_rt_experiment\energy_fnb58\EnergySweep_20261024_A"
$RawRoot = Join-Path $CampRoot "raw"
$AnRoot = Join-Path $CampRoot "analysis"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

New-Item -ItemType Directory -Force -Path $RawRoot, $AnRoot | Out-Null

$protocol = "SENSOR-ATTACHED_SYNTHETIC-TASK_GROSS_BOARD_ENERGY"
$modes = @("original", "prng", "fpe")
$Bs = @(64, 128, 256, 512, 1024, 2048, 4096)

function Get-Timeout([string]$mode, [int]$B) {
  # Table I no-RT means: PRNG B64 ~8.3s/rd, FPE B64 ~3.9s/rd; Energy + NT similar order
  $per = switch ($mode) {
    "prng" {
      switch ($B) { 64 { 10.0 }; 128 { 3.0 }; 256 { 1.5 }; default { 1.0 } }
    }
    "fpe" {
      switch ($B) { 64 { 5.0 }; 128 { 2.8 }; 256 { 1.8 }; default { 1.0 } }
    }
    default {
      switch ($B) { 64 { 1.0 }; 128 { 0.8 }; default { 0.7 } }
    }
  }
  [int]([Math]::Max($TimeoutBaseSec, 60 + 10 * $per * 12))
}

function Invoke-Cell {
  param([string]$ShuffleMode, [int]$B, [string]$CellKind = "matrix")

  $tag = "Energy512_NT1000_${ShuffleMode}_B$B"
  $cellName = if ($CellKind -eq "matrix") { "${ShuffleMode}_B$B" } else { "drift_${CellKind}_${ShuffleMode}_B$B" }
  $outDir = Join-Path $RawRoot $cellName
  $capTag = "energy_${ShuffleMode}_B$B"
  if ($CellKind -ne "matrix") { $capTag = "drift_${CellKind}_${ShuffleMode}_B$B" }

  if ((Test-Path (Join-Path $outDir "${capTag}_fnb58.csv")) -and (Test-Path (Join-Path $outDir "${capTag}_uart_ts.csv"))) {
    Write-Host "SKIP already captured: $cellName"
    return
  }

  $Build = Join-Path $Root "thesis\03_rt_experiment\builds\$tag"
  $needBuild = $true
  if ($SkipBuildIfPresent -and (Test-Path (Join-Path $Build "Secure.elf")) -and (Test-Path (Join-Path $Build "NonSecure.elf"))) {
    $needBuild = $false
    Write-Host "Reuse build $tag"
  }
  if ($needBuild) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File $BuildScript -ShuffleMode $ShuffleMode -B $B
    if ($LASTEXITCODE -ne 0) { throw "Build failed $tag" }
  }

  $sElf = Join-Path $Build "Secure.elf"
  $nsElf = Join-Path $Build "NonSecure.elf"
  $meta = Join-Path $Build "run_meta.txt"
  if (!(Test-Path $sElf) -or !(Test-Path $nsElf)) { throw "Missing ELF $tag" }

  New-Item -ItemType Directory -Force -Path $outDir | Out-Null
  Copy-Item $meta (Join-Path $outDir "run_meta.txt") -Force
  Add-Content (Join-Path $outDir "run_meta.txt") "experiment_id=EnergySweep_20261024_A"
  Add-Content (Join-Path $outDir "run_meta.txt") "protocol_label=$protocol"
  Add-Content (Join-Path $outDir "run_meta.txt") "cell_kind=$CellKind"
  Add-Content (Join-Path $outDir "run_meta.txt") "sensors=ATTACHED_ELECTRICAL_LOAD_ONLY"
  Add-Content (Join-Path $outDir "run_meta.txt") "PULSE_SMOKE_TEST=0"
  Add-Content (Join-Path $outDir "run_meta.txt") "flash=PERFORMED_EnergySweep_20261024_A"

  Write-Host "FLASH $tag ($CellKind)"
  & $prog -c port=SWD mode=UR -d $sElf
  if ($LASTEXITCODE -ne 0) { throw "Secure flash failed $tag" }
  & $prog -c port=SWD mode=UR -d $nsElf
  if ($LASTEXITCODE -ne 0) { throw "NS flash failed $tag" }

  $to = Get-Timeout $ShuffleMode $B
  Write-Host "CAPTURE $cellName timeout=${to}s"
  python $SyncPy --out-dir $outDir --tag $capTag --uart-port COM3 --timeout-s $to --reset-swd
  if ($LASTEXITCODE -ne 0) { throw "Sync capture failed $cellName exit=$LASTEXITCODE" }

  $roundsCsv = Join-Path $AnRoot "${cellName}_rounds.csv"
  python $IntegPy `
    --uart-ts (Join-Path $outDir "${capTag}_uart_ts.csv") `
    --meter (Join-Path $outDir "${capTag}_fnb58.csv") `
    --out $roundsCsv
  if ($LASTEXITCODE -ne 0) { throw "Integrate failed $cellName" }
  Write-Host "OK $cellName"
}

# Idle baseline (no flash)
$idleDir = Join-Path $RawRoot "idle_baseline_5s"
if (-not (Test-Path (Join-Path $idleDir "idle_5s.csv"))) {
  New-Item -ItemType Directory -Force -Path $idleDir | Out-Null
  Write-Host "Idle 5s FNB58 baseline..."
  python (Join-Path $Root "thesis\03_rt_experiment\energy_fnb58\scripts\fnb58_hid_log.py") `
    --out (Join-Path $idleDir "idle_5s.csv") --duration 5
  @"
experiment_id=EnergySweep_20261024_A
protocol_label=$protocol
note=idle_board_with_sensors_attached_NT_may_or_may_not_be_running_depends_on_prior_image
captured_utc=$((Get-Date).ToUniversalTime().ToString('o'))
"@ | Set-Content (Join-Path $idleDir "idle_meta.txt")
}

if ($OnlyMode -ne "" -and $OnlyB -gt 0) {
  Invoke-Cell -ShuffleMode $OnlyMode -B $OnlyB -CellKind "matrix"
  exit 0
}

if (-not $DriftChecksOnly) {
  # Start drift check
  Invoke-Cell -ShuffleMode "original" -B 1024 -CellKind "start"

  foreach ($B in $Bs) {
    foreach ($m in $modes) {
      Invoke-Cell -ShuffleMode $m -B $B -CellKind "matrix"
    }
    if ($B -eq 512) {
      Invoke-Cell -ShuffleMode "original" -B 1024 -CellKind "mid"
    }
  }

  # End drift check
  Invoke-Cell -ShuffleMode "original" -B 1024 -CellKind "end"
} else {
  Invoke-Cell -ShuffleMode "original" -B 1024 -CellKind "start"
}

Write-Host "Sweep driver finished. Aggregate via analysis script."
