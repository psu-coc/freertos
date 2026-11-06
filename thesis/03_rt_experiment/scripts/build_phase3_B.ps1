<#
.SYNOPSIS
  Rebuild Secure + NonSecure for one Original SMARM block size B.
.DESCRIPTION
  Sets ATTEST_BLOCK_SIZE in Secure_nsclib/attest_overhead_cfg.h (keeps USE_SAU=0,
  USE_SNAP=0), force-deletes heap_4.o to avoid stale 8 KiB ucHeap, builds both
  images, verifies ucHeap size == 32 KiB, copies ELFs + cfg snapshot under
  thesis/03_rt_experiment/builds/original_B<B>/.

  Does NOT flash or open COM3.
.PARAMETER B
  Block size: 64, 128, 256, 512, 1024, 2048, 4096
.PARAMETER SkipBuild
  Only rewrite cfg / copy metadata (no make)
#>
param(
  [Parameter(Mandatory = $true)][ValidateSet(64, 128, 256, 512, 1024, 2048, 4096)]
  [int]$B,
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$Cfg = Join-Path $Root "Secure_nsclib\attest_overhead_cfg.h"
$SecureDebug = Join-Path $Root "STM32CubeIDE\Secure\Debug"
$NsDebug = Join-Path $Root "STM32CubeIDE\NonSecure\Debug"
$OutDir = Join-Path $Root "thesis\03_rt_experiment\builds\original_B$B"
$GccBin = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin"
$MakeBin = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin"
$env:PATH = "$GccBin;$MakeBin;" + $env:PATH

Write-Host "=== Phase3 Original SMARM build B=$B ==="
Write-Host "Root=$Root"

# --- rewrite ATTEST_BLOCK_SIZE; force Original SMARM flags ---
$text = Get-Content -Raw -Path $Cfg
if ($text -notmatch 'USE_SAU_APPROACH') { throw "cfg missing USE_SAU_APPROACH: $Cfg" }
$text = [regex]::Replace($text, '(?m)^#define\s+USE_SAU_APPROACH\s+\d+', '#define USE_SAU_APPROACH  0')
$text = [regex]::Replace($text, '(?m)^#define\s+USE_SNAP\s+\d+', '#define USE_SNAP          0')
$text = [regex]::Replace($text, '(?m)^#define\s+ATTEST_BLOCK_SIZE\s+\d+', "#define ATTEST_BLOCK_SIZE  $B")
Set-Content -Path $Cfg -Value $text -NoNewline
Write-Host "Updated $Cfg -> B=$B USE_SAU=0 USE_SNAP=0"

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
Copy-Item $Cfg (Join-Path $OutDir "attest_overhead_cfg.h") -Force

if ($SkipBuild) {
  Write-Host "SkipBuild set; done."
  exit 0
}

# --- prevent stale heap_4.o (previously linked as 8 KiB ucHeap) ---
$forceRebuild = @(
  (Join-Path $NsDebug "Middlewares\FreeRTOS\heap_4.o"),
  (Join-Path $NsDebug "Middlewares\FreeRTOS\cmsis_os2.o"),
  (Join-Path $NsDebug "Application\User\main.o"),
  (Join-Path $SecureDebug "Application\User\secure_nsc.o"),
  (Join-Path $SecureDebug "secure_nsclib.o")
)
foreach ($o in $forceRebuild) {
  if (Test-Path $o) {
    Remove-Item -Force $o
    Write-Host "Removed for rebuild: $o"
  }
}

function Invoke-Make([string]$dir, [string]$label) {
  Push-Location $dir
  try {
    Write-Host "--- make $label ---"
    & make -j8 all
    if ($LASTEXITCODE -ne 0) { throw "make failed in $dir (exit $LASTEXITCODE)" }
  } finally {
    Pop-Location
  }
}

Invoke-Make $SecureDebug "Secure"
Invoke-Make $NsDebug "NonSecure"

$nsElf = Join-Path $NsDebug "FreeRTOS_SecureIOToggle_TrustZone_NonSecure.elf"
$sElf = Join-Path $SecureDebug "FreeRTOS_SecureIOToggle_TrustZone_Secure.elf"
if (!(Test-Path $nsElf) -or !(Test-Path $sElf)) { throw "ELF missing after build" }

# --- verify ucHeap == 32 KiB ---
$nmLine = & arm-none-eabi-nm -S $nsElf | Select-String "ucHeap"
if (-not $nmLine) { throw "ucHeap symbol not found in NS ELF" }
$parts = ($nmLine.ToString() -split '\s+') | Where-Object { $_ -ne '' }
# nm -S: addr size type name  -> size is hex field index 1
$heapSize = [Convert]::ToInt32($parts[1], 16)
Write-Host "ucHeap size = $heapSize (expect 32768)"
if ($heapSize -ne 32768) {
  throw "FATAL: ucHeap is $heapSize bytes; refuse to ship stale/wrong heap (need 32768)"
}

Copy-Item $sElf (Join-Path $OutDir "Secure.elf") -Force
Copy-Item $nsElf (Join-Path $OutDir "NonSecure.elf") -Force
& arm-none-eabi-size $sElf $nsElf | Tee-Object -FilePath (Join-Path $OutDir "size.txt")

@"
mode=Original_SMARM
USE_SAU_APPROACH=0
USE_SNAP=0
ATTEST_BLOCK_SIZE=$B
ATTEST_DATA_BASE=0x08060000
TOTAL_SIZE=0x20000
US_IRQ_MASK_DURING_ECHO=0
SMARM_per_block_irq_mask=1
sonar_period_ms=200
wheel_period_ms=1000
attest_priority=osPriorityLow
heap_ns_bytes=32768
built_utc=$((Get-Date).ToUniversalTime().ToString('o'))
"@ | Set-Content -Path (Join-Path $OutDir "run_meta.txt")

Write-Host "OK: artifacts in $OutDir"
Write-Host "Flash/COM3 intentionally NOT performed."
