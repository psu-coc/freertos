<#
.SYNOPSIS
  Build NonSecure (+ Secure if attest) for a selectable RT sensor profile.
.DESCRIPTION
  Does NOT flash or open COM3.
  Forces heap_4.o rebuild and asserts ucHeap == 32768.
.PARAMETER Profile
  10Hz10Hz (default) or Legacy5_1
.PARAMETER RunLen
  smoke20 | 60s (legacy job-count) | wall60 (legacy 60s obs) | wall30 (2s warm-up + 30s obs)
.PARAMETER WithOriginalSmarm
  If set, PHASE3_WITH_ORIGINAL_SMARM=1 and rebuilds Secure for given -B
.PARAMETER ShuffleMode
  original | prng | fpe  (ATTEST_SHUFFLE_MODE). Implies WithOriginalSmarm when not noattest.
.PARAMETER B
  ATTEST_BLOCK_SIZE when attest enabled (default 1024)
.PARAMETER DigestProbe
  If set, RT_DIGEST_PROBE=1 and output dir is digest_probe_{mode}_B{B} (Gate A only).
#>
param(
  [ValidateSet("10Hz10Hz", "Legacy5_1", "Wheel100Hz", "Wheel1000Hz")][string]$Profile = "10Hz10Hz",
  [ValidateSet("smoke20", "60s", "wall60", "wall30", "wall3")][string]$RunLen = "wall30",
  [switch]$WithOriginalSmarm,
  [ValidateSet("original", "prng", "fpe", "none")][string]$ShuffleMode = "none",
  [ValidateSet(64, 128, 256, 512, 1024, 2048, 4096)][int]$B = 1024,
  [switch]$DigestProbe
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$ProfH = Join-Path $Root "NonSecure\Inc\rt_experiment_profile.h"
$Cfg = Join-Path $Root "Secure_nsclib\attest_overhead_cfg.h"
$SecureDebug = Join-Path $Root "STM32CubeIDE\Secure\Debug"
$NsDebug = Join-Path $Root "STM32CubeIDE\NonSecure\Debug"

$modeNum = switch ($ShuffleMode) {
  "original" { 0 }
  "prng"     { 1 }
  "fpe"      { 2 }
  default    { -1 }
}
if ($ShuffleMode -ne "none") {
  $WithOriginalSmarm = $true
}
$tag = if ($DigestProbe) {
  if ($ShuffleMode -eq "none") { throw "DigestProbe requires -ShuffleMode" }
  "digest_probe_${ShuffleMode}_B$B"
} elseif (-not $WithOriginalSmarm) {
  "noattest"
} elseif ($ShuffleMode -eq "none" -or $ShuffleMode -eq "original") {
  "original_B$B"
} else {
  "${ShuffleMode}_B$B"
}
$OutDir = Join-Path $Root "thesis\03_rt_experiment\builds\${Profile}_${RunLen}_$tag"
if ($DigestProbe) {
  $OutDir = Join-Path $Root "thesis\03_rt_experiment\builds\digest_probe_${ShuffleMode}_B$B"
}
$GccBin = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin"
$MakeBin = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin"
$env:PATH = "$GccBin;$MakeBin;" + $env:PATH

$profEnum = switch ($Profile) {
  "10Hz10Hz"    { "RT_PROFILE_10HZ_10HZ" }
  "Wheel100Hz"  { "RT_PROFILE_WHEEL_100HZ" }
  "Wheel1000Hz" { "RT_PROFILE_WHEEL_1000HZ_WALL3" }
  default       { "RT_PROFILE_LEGACY_5HZ_1HZ" }
}
if ($Profile -eq "Wheel1000Hz" -and $RunLen -eq "wall30") {
  Write-Host "NOTE: Wheel1000Hz uses wall3 (3s obs); coercing RunLen=wall3"
  $RunLen = "wall3"
}
$runEnum = switch ($RunLen) {
  "smoke20" { "RT_RUN_SMOKE" }
  "wall60"  { "RT_RUN_WALL_OBS" }  # profile macros define duration; tree uses wall30 constants
  "wall30"  { "RT_RUN_WALL_OBS" }
  "wall3"   { "RT_RUN_WALL_OBS" }
  default   { "RT_RUN_FULL" }  # 60s legacy job-count
}
$p3 = if ($WithOriginalSmarm) { 1 } else { 0 }
if ($modeNum -lt 0 -and $WithOriginalSmarm) { $modeNum = 0 }

Write-Host "=== RT profile build Profile=$Profile RunLen=$RunLen SMARM=$p3 ShuffleMode=$ShuffleMode($modeNum) B=$B ==="

# Rewrite selectable defaults in profile header (keeps LEGACY macros intact).
$ph = Get-Content -Raw -Path $ProfH
$ph = [regex]::Replace($ph, '(?m)^#ifndef RT_SENSOR_PROFILE\r?\n#define RT_SENSOR_PROFILE\s+\w+',
  "#ifndef RT_SENSOR_PROFILE`r`n#define RT_SENSOR_PROFILE  $profEnum")
$ph = [regex]::Replace($ph, '(?m)^#ifndef RT_RUN_LEN\r?\n#define RT_RUN_LEN\s+\w+',
  "#ifndef RT_RUN_LEN`r`n#define RT_RUN_LEN  $runEnum")
$probeVal = if ($DigestProbe) { 1 } else { 0 }
if ($ph -match '(?m)^#ifndef RT_DIGEST_PROBE\r?\n#define RT_DIGEST_PROBE\s+\d+') {
  $ph = [regex]::Replace($ph, '(?m)^#ifndef RT_DIGEST_PROBE\r?\n#define RT_DIGEST_PROBE\s+\d+',
    "#ifndef RT_DIGEST_PROBE`r`n#define RT_DIGEST_PROBE  $probeVal")
} else {
  throw "rt_experiment_profile.h missing RT_DIGEST_PROBE default"
}
Set-Content -Path $ProfH -Value $ph -NoNewline

# main.c PHASE3 default: ensure define in main matches switch via compile flag file
# Inject by rewriting the ifndef default in main.c
$main = Join-Path $Root "NonSecure\Src\main.c"
$mt = Get-Content -Raw -Path $main
$mt = [regex]::Replace($mt,
  '(?m)^#ifndef PHASE3_WITH_ORIGINAL_SMARM\r?\n#define PHASE3_WITH_ORIGINAL_SMARM\s+\d+',
  "#ifndef PHASE3_WITH_ORIGINAL_SMARM`r`n#define PHASE3_WITH_ORIGINAL_SMARM  $p3")
Set-Content -Path $main -Value $mt -NoNewline

if ($WithOriginalSmarm) {
  $ct = Get-Content -Raw -Path $Cfg
  $ct = [regex]::Replace($ct, '(?m)^#define\s+USE_SAU_APPROACH\s+\d+', '#define USE_SAU_APPROACH  0')
  $ct = [regex]::Replace($ct, '(?m)^#define\s+USE_SNAP\s+\d+', '#define USE_SNAP          0')
  $ct = [regex]::Replace($ct, '(?m)^#define\s+ATTEST_BLOCK_SIZE\s+\d+', "#define ATTEST_BLOCK_SIZE  $B")
  if ($ct -match '(?m)^#ifndef ATTEST_SHUFFLE_MODE\r?\n#define ATTEST_SHUFFLE_MODE\s+\S+') {
    $ct = [regex]::Replace($ct, '(?m)^#ifndef ATTEST_SHUFFLE_MODE\r?\n#define ATTEST_SHUFFLE_MODE\s+\S+',
      "#ifndef ATTEST_SHUFFLE_MODE`r`n#define ATTEST_SHUFFLE_MODE  $modeNum")
  } else {
    throw "attest_overhead_cfg.h missing #ifndef ATTEST_SHUFFLE_MODE default"
  }
  Set-Content -Path $Cfg -Value $ct -NoNewline
}

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Copy-Item $ProfH (Join-Path $OutDir "rt_experiment_profile.h") -Force
if (Test-Path $Cfg) { Copy-Item $Cfg (Join-Path $OutDir "attest_overhead_cfg.h") -Force }

$forceRebuild = @(
  (Join-Path $NsDebug "Middlewares\FreeRTOS\heap_4.o"),
  (Join-Path $NsDebug "Middlewares\FreeRTOS\cmsis_os2.o"),
  (Join-Path $NsDebug "Application\User\main.o")
)
if ($WithOriginalSmarm) {
  $forceRebuild += (Join-Path $SecureDebug "Application\User\secure_nsc.o")
  $forceRebuild += (Join-Path $SecureDebug "secure_nsclib.o")
  $forceRebuild += (Join-Path $SecureDebug "Application\User\Speck\ff1_speck.o")
  $forceRebuild += (Join-Path $SecureDebug "Application\User\Speck\speck.o")
}
foreach ($o in $forceRebuild) {
  if (Test-Path $o) { Remove-Item -Force $o; Write-Host "Removed $o" }
}

function Invoke-Make([string]$dir) {
  Push-Location $dir
  $prevEa = $ErrorActionPreference
  $ErrorActionPreference = "Continue"  # gcc warnings on stderr != failure
  try {
    & make -j8 all
    if ($LASTEXITCODE -ne 0) { throw "make failed in $dir" }
  } finally {
    $ErrorActionPreference = $prevEa
    Pop-Location
  }
}

if ($WithOriginalSmarm) { Invoke-Make $SecureDebug }
Invoke-Make $NsDebug

$nsElf = Join-Path $NsDebug "FreeRTOS_SecureIOToggle_TrustZone_NonSecure.elf"
$nmLine = & arm-none-eabi-nm -S $nsElf | Select-String "ucHeap"
$parts = ($nmLine.ToString() -split '\s+') | Where-Object { $_ -ne '' }
$heapSize = [Convert]::ToInt32($parts[1], 16)
Write-Host "ucHeap=$heapSize"
if ($heapSize -ne 32768) { throw "ucHeap must be 32768, got $heapSize" }

$bssLine = & arm-none-eabi-size $nsElf
Copy-Item $nsElf (Join-Path $OutDir "NonSecure.elf") -Force
if ($WithOriginalSmarm) {
  Copy-Item (Join-Path $SecureDebug "FreeRTOS_SecureIOToggle_TrustZone_Secure.elf") (Join-Path $OutDir "Secure.elf") -Force
}
$bssLine | Tee-Object (Join-Path $OutDir "size.txt")

$endSym = & arm-none-eabi-nm $nsElf | Select-String " _end$"
$estack = & arm-none-eabi-nm $nsElf | Select-String " _estack$"
@"
profile=$Profile
run=$RunLen
PHASE3_WITH_ORIGINAL_SMARM=$p3
ATTEST_SHUFFLE_MODE=$modeNum
ATTEST_SHUFFLE_MODE_NAME=$ShuffleMode
ATTEST_BLOCK_SIZE=$B
ATTEST_IRQ_HMAC_MASK=1
USE_SAU_APPROACH=0
USE_SNAP=0
ATTEST_REGION=0x08060000/0x20000
US_IRQ_MASK_DURING_ECHO=0
heap_ns_bytes=32768
nm_ucHeap=$heapSize
_end=$endSym
_estack=$estack
built_utc=$((Get-Date).ToUniversalTime().ToString('o'))
flash=NOT_PERFORMED
DMR=QUARANTINED
RT_DIGEST_PROBE=$probeVal
"@ | Set-Content (Join-Path $OutDir "run_meta.txt")

Write-Host "OK -> $OutDir"
Write-Host "Flash/COM3 intentionally NOT performed."
