<#
.SYNOPSIS
  Build Secure+NS for dummy-task FAR harness (PULSE_SMOKE_TEST=0).
  Archives ELF + cfg under builds/FAR1000Hz_{mode}_B{B}[_farref]/
  Restores PULSE_SMOKE_TEST=1 after build so Wheel corpus tooling stays default.
#>
param(
  [ValidateSet("original", "prng", "fpe")][string]$ShuffleMode = "original",
  [ValidateSet(64, 128, 256, 512, 1024, 2048, 4096)][int]$B = 1024,
  [switch]$FarRef  # IRQ-on HMAC reference (ORIGINAL only)
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$Cfg = Join-Path $Root "Secure_nsclib\attest_overhead_cfg.h"
$Main = Join-Path $Root "NonSecure\Src\main.c"
$SecureDebug = Join-Path $Root "STM32CubeIDE\Secure\Debug"
$NsDebug = Join-Path $Root "STM32CubeIDE\NonSecure\Debug"
$GccBin = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin"
$MakeBin = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin"
$env:PATH = "$GccBin;$MakeBin;" + $env:PATH

$modeNum = switch ($ShuffleMode) { "original" { 0 } "prng" { 1 } "fpe" { 2 } }
if ($FarRef -and $ShuffleMode -ne "original") { throw "FarRef only with original" }
$farRefVal = if ($FarRef) { 1 } else { 0 }
$tag = if ($FarRef) { "FAR1000Hz_${ShuffleMode}_B${B}_farref" } else { "FAR1000Hz_${ShuffleMode}_B$B" }
$OutDir = Join-Path $Root "thesis\03_rt_experiment\builds\$tag"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

Write-Host "=== FAR build $tag ==="

# Save + force FAR harness (TABLE_I off → PULSE_SMOKE_TEST derives to 0 via build... wait:
# FAR needs PULSE_SMOKE=0 and TABLE_I=0. With TABLE_I=0, PULSE defaults to 1.
# Override: temporarily set TABLE_I_RUNTIME=0 and force PULSE via rewriting the else branch only.
$mainText = Get-Content -Raw -Path $Main
if ($mainText -notmatch '(?m)^#ifndef TABLE_I_RUNTIME\r?\n#define TABLE_I_RUNTIME\s+\d+') {
  throw "TABLE_I_RUNTIME ifndef block missing"
}
$mainText = [regex]::Replace($mainText,
  '(?m)^#ifndef TABLE_I_RUNTIME\r?\n#define TABLE_I_RUNTIME\s+\d+',
  "#ifndef TABLE_I_RUNTIME`r`n#define TABLE_I_RUNTIME  0")
# Force pulse smoke off for FAR: rewrite both arms to 0 for this build only.
$mainText = [regex]::Replace($mainText, '(?m)^#define PULSE_SMOKE_TEST\s+\d+', '#define PULSE_SMOKE_TEST  0')
Set-Content -Path $Main -Value $mainText -NoNewline

$ct = Get-Content -Raw -Path $Cfg
if ($ct -match '(?m)^#ifndef ATTEST_TABLE_I_REGION\r?\n#define ATTEST_TABLE_I_REGION\s+\d+') {
  $ct = [regex]::Replace($ct,
    '(?m)^#ifndef ATTEST_TABLE_I_REGION\r?\n#define ATTEST_TABLE_I_REGION\s+\d+',
    "#ifndef ATTEST_TABLE_I_REGION`r`n#define ATTEST_TABLE_I_REGION  0")
}
$ct = [regex]::Replace($ct, '(?m)^#define\s+USE_SAU_APPROACH\s+\d+', '#define USE_SAU_APPROACH  0')
$ct = [regex]::Replace($ct, '(?m)^#define\s+USE_SNAP\s+\d+', '#define USE_SNAP          0')
$ct = [regex]::Replace($ct, '(?m)^#define\s+ATTEST_BLOCK_SIZE\s+\d+.*$', "#define ATTEST_BLOCK_SIZE  $B")
$ct = [regex]::Replace($ct, '(?m)^#ifndef ATTEST_SHUFFLE_MODE\r?\n#define ATTEST_SHUFFLE_MODE\s+\S+',
  "#ifndef ATTEST_SHUFFLE_MODE`r`n#define ATTEST_SHUFFLE_MODE  $modeNum")
if ($ct -match '(?m)^#ifndef FAR_REF_NO_IRQ_DISABLE\r?\n#define FAR_REF_NO_IRQ_DISABLE\s+\d+') {
  $ct = [regex]::Replace($ct, '(?m)^#ifndef FAR_REF_NO_IRQ_DISABLE\r?\n#define FAR_REF_NO_IRQ_DISABLE\s+\d+',
    "#ifndef FAR_REF_NO_IRQ_DISABLE`r`n#define FAR_REF_NO_IRQ_DISABLE  $farRefVal")
} elseif ($ct -match '(?m)^#define\s+FAR_REF_NO_IRQ_DISABLE\s+\d+') {
  $ct = [regex]::Replace($ct, '(?m)^#define\s+FAR_REF_NO_IRQ_DISABLE\s+\d+',
    "#define FAR_REF_NO_IRQ_DISABLE  $farRefVal")
} else { throw "FAR_REF_NO_IRQ_DISABLE missing in cfg" }
Set-Content -Path $Cfg -Value $ct -NoNewline

$force = @(
  (Join-Path $NsDebug "Application\User\main.o"),
  (Join-Path $SecureDebug "Application\User\secure_nsc.o"),
  (Join-Path $SecureDebug "secure_nsclib.o"),
  (Join-Path $SecureDebug "Application\User\Speck\ff1_speck.o")
)
foreach ($o in $force) { if (Test-Path $o) { Remove-Item -Force $o } }

function Invoke-Make([string]$dir) {
  Push-Location $dir
  $prev = $ErrorActionPreference
  $ErrorActionPreference = "Continue"
  try {
    & make -j8 all
    if ($LASTEXITCODE -ne 0) { throw "make failed in $dir" }
  } finally {
    $ErrorActionPreference = $prev
    Pop-Location
  }
}

try {
  Invoke-Make $SecureDebug
  Invoke-Make $NsDebug
  $nsElf = Join-Path $NsDebug "FreeRTOS_SecureIOToggle_TrustZone_NonSecure.elf"
  $sElf = Join-Path $SecureDebug "FreeRTOS_SecureIOToggle_TrustZone_Secure.elf"
  Copy-Item $nsElf (Join-Path $OutDir "NonSecure.elf") -Force
  Copy-Item $sElf (Join-Path $OutDir "Secure.elf") -Force
  Copy-Item $Cfg (Join-Path $OutDir "attest_overhead_cfg.h") -Force
  $sSha = (Get-FileHash $sElf -Algorithm SHA256).Hash
  $nSha = (Get-FileHash $nsElf -Algorithm SHA256).Hash
  @"
tag=$tag
protocol=dummy_FAR_1000Hz
PULSE_SMOKE_TEST=0
TARGET_FREQ_HZ=1000
ATTEST_SHUFFLE_MODE=$modeNum
ATTEST_SHUFFLE_MODE_NAME=$ShuffleMode
ATTEST_BLOCK_SIZE=$B
ATTEST_IRQ_HMAC_MASK=1
FAR_REF_NO_IRQ_DISABLE=$farRefVal
USE_SAU_APPROACH=0
USE_SNAP=0
ATTEST_REGION=0x08060000/0x20000
TIM2_prescaler=799
TIM2_TICKS_PER_SEC=137500
rounds=10
secure_sha256=$sSha
ns_sha256=$nSha
built_utc=$((Get-Date).ToUniversalTime().ToString('o'))
"@ | Set-Content (Join-Path $OutDir "run_meta.txt")
  Write-Host "OK -> $OutDir"
} finally {
  # Restore Wheel/RT defaults: TABLE_I=0 and PULSE arms 0/1 as designed
  $mt = Get-Content -Raw -Path $Main
  $mt = [regex]::Replace($mt,
    '(?m)^#ifndef TABLE_I_RUNTIME\r?\n#define TABLE_I_RUNTIME\s+\d+',
    "#ifndef TABLE_I_RUNTIME`r`n#define TABLE_I_RUNTIME  0")
  # Restore #if TABLE_I / #else pulse arms
  $mt = [regex]::Replace($mt,
    '(?ms)^#if TABLE_I_RUNTIME\r?\n#define PULSE_SMOKE_TEST\s+\d+\r?\n#else\r?\n#define PULSE_SMOKE_TEST\s+\d+\r?\n#endif',
    "#if TABLE_I_RUNTIME`r`n#define PULSE_SMOKE_TEST  0`r`n#else`r`n#define PULSE_SMOKE_TEST  1`r`n#endif")
  Set-Content -Path $Main -Value $mt -NoNewline
  $ct2 = Get-Content -Raw -Path $Cfg
  if ($ct2 -match '(?m)^#ifndef ATTEST_TABLE_I_REGION\r?\n#define ATTEST_TABLE_I_REGION\s+\d+') {
    $ct2 = [regex]::Replace($ct2,
      '(?m)^#ifndef ATTEST_TABLE_I_REGION\r?\n#define ATTEST_TABLE_I_REGION\s+\d+',
      "#ifndef ATTEST_TABLE_I_REGION`r`n#define ATTEST_TABLE_I_REGION  0")
  }
  if ($ct2 -match '(?m)^#ifndef FAR_REF_NO_IRQ_DISABLE\r?\n#define FAR_REF_NO_IRQ_DISABLE\s+\d+') {
    $ct2 = [regex]::Replace($ct2, '(?m)^#ifndef FAR_REF_NO_IRQ_DISABLE\r?\n#define FAR_REF_NO_IRQ_DISABLE\s+\d+',
      "#ifndef FAR_REF_NO_IRQ_DISABLE`r`n#define FAR_REF_NO_IRQ_DISABLE  0")
  } elseif ($ct2 -match '(?m)^#define\s+FAR_REF_NO_IRQ_DISABLE\s+\d+') {
    $ct2 = [regex]::Replace($ct2, '(?m)^#define\s+FAR_REF_NO_IRQ_DISABLE\s+\d+',
      "#define FAR_REF_NO_IRQ_DISABLE  0")
  }
  $ct2 = [regex]::Replace($ct2, '(?m)^#ifndef ATTEST_SHUFFLE_MODE\r?\n#define ATTEST_SHUFFLE_MODE\s+\S+',
    "#ifndef ATTEST_SHUFFLE_MODE`r`n#define ATTEST_SHUFFLE_MODE  0")
  $ct2 = [regex]::Replace($ct2, '(?m)^#define\s+ATTEST_BLOCK_SIZE\s+\d+.*$',
    '#define ATTEST_BLOCK_SIZE  1024  /* Stage-1 default; matrix: 64..4096 */')
  Set-Content -Path $Cfg -Value $ct2 -NoNewline
  Write-Host "Restored TABLE_I=0, PULSE arms, FAR_REF=0, shuffle=original, B=1024 (Wheel defaults)."
}
