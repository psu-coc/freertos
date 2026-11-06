<#
.SYNOPSIS
  Build Secure+NS for Access Table I runtime (512 KiB @ 0x08000000, no NormalTask).
  Archives ELF + cfg under builds/TableI_{mode}_B{B}/.
  Restores TABLE_I_RUNTIME=0 and ATTEST_TABLE_I_REGION=0 after build.
#>
param(
  [ValidateSet("original", "prng", "fpe")][string]$ShuffleMode = "original",
  [ValidateSet(64, 128, 256, 512, 1024, 2048, 4096)][int]$B = 1024
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
$tag = "TableI_${ShuffleMode}_B$B"
$OutDir = Join-Path $Root "thesis\03_rt_experiment\builds\$tag"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

Write-Host "=== Table I build $tag ==="

function Set-DefineInIfndefBlock([string]$text, [string]$name, [string]$value) {
  $pat = "(?m)^#ifndef\s+$name\r?\n#define\s+$name\s+\S+"
  if ($text -notmatch $pat) { throw "Missing #ifndef/#define block for $name" }
  return [regex]::Replace($text, $pat, "#ifndef $name`r`n#define $name  $value")
}

try {
  $mainText = Get-Content -Raw -Path $Main
  $mainText = Set-DefineInIfndefBlock $mainText "TABLE_I_RUNTIME" "1"
  Set-Content -Path $Main -Value $mainText -NoNewline

  $ct = Get-Content -Raw -Path $Cfg
  $ct = Set-DefineInIfndefBlock $ct "ATTEST_TABLE_I_REGION" "1"
  $ct = Set-DefineInIfndefBlock $ct "ATTEST_SHUFFLE_MODE" "$modeNum"
  $ct = [regex]::Replace($ct, '(?m)^#define\s+USE_SAU_APPROACH\s+\d+', '#define USE_SAU_APPROACH  0')
  $ct = [regex]::Replace($ct, '(?m)^#define\s+USE_SNAP\s+\d+', '#define USE_SNAP          0')
  $ct = [regex]::Replace($ct, '(?m)^#define\s+ATTEST_BLOCK_SIZE\s+\d+.*$', "#define ATTEST_BLOCK_SIZE  $B")
  if ($ct -match '(?m)^#ifndef FAR_REF_NO_IRQ_DISABLE\r?\n#define FAR_REF_NO_IRQ_DISABLE\s+\d+') {
    $ct = [regex]::Replace($ct, '(?m)^#ifndef FAR_REF_NO_IRQ_DISABLE\r?\n#define FAR_REF_NO_IRQ_DISABLE\s+\d+',
      "#ifndef FAR_REF_NO_IRQ_DISABLE`r`n#define FAR_REF_NO_IRQ_DISABLE  0")
  }
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

  Invoke-Make $SecureDebug
  Invoke-Make $NsDebug

  $nsElf = Join-Path $NsDebug "FreeRTOS_SecureIOToggle_TrustZone_NonSecure.elf"
  $sElf = Join-Path $SecureDebug "FreeRTOS_SecureIOToggle_TrustZone_Secure.elf"
  Copy-Item $nsElf (Join-Path $OutDir "NonSecure.elf") -Force
  Copy-Item $sElf (Join-Path $OutDir "Secure.elf") -Force
  Copy-Item $Cfg (Join-Path $OutDir "attest_overhead_cfg.h") -Force
  Copy-Item $Main (Join-Path $OutDir "main.c.snippet_check") -Force

  $sSha = (Get-FileHash $sElf -Algorithm SHA256).Hash
  $nSha = (Get-FileHash $nsElf -Algorithm SHA256).Hash
  $git = (git -C $Root rev-parse HEAD 2>$null)
  @"
tag=$tag
protocol=Access_TableI_runtime_512KiB_noRT
TABLE_I_RUNTIME=1
ATTEST_TABLE_I_REGION=1
PULSE_SMOKE_TEST=0
NormalTask=0
ATTEST_SHUFFLE_MODE=$modeNum
ATTEST_SHUFFLE_MODE_NAME=$ShuffleMode
ATTEST_BLOCK_SIZE=$B
ATTEST_IRQ_HMAC_MASK=1
FAR_REF_NO_IRQ_DISABLE=0
USE_SAU_APPROACH=0
USE_SNAP=0
ATTEST_REGION=0x08000000/0x80000
ATTEST_PASSES=1
TIM2_prescaler=799
TIM2_TICKS_PER_SEC=137500
SYSCLK_HZ=110000000
rounds=10
secure_sha256=$sSha
ns_sha256=$nSha
git_rev=$git
built_utc=$((Get-Date).ToUniversalTime().ToString('o'))
"@ | Set-Content (Join-Path $OutDir "run_meta.txt")
  Write-Host "OK -> $OutDir"
  Write-Host "secure_sha256=$sSha"
  Write-Host "ns_sha256=$nSha"
}
finally {
  $mt = Get-Content -Raw -Path $Main
  $mt = Set-DefineInIfndefBlock $mt "TABLE_I_RUNTIME" "0"
  Set-Content -Path $Main -Value $mt -NoNewline
  $ct2 = Get-Content -Raw -Path $Cfg
  $ct2 = Set-DefineInIfndefBlock $ct2 "ATTEST_TABLE_I_REGION" "0"
  $ct2 = Set-DefineInIfndefBlock $ct2 "ATTEST_SHUFFLE_MODE" "0"
  $ct2 = [regex]::Replace($ct2, '(?m)^#define\s+ATTEST_BLOCK_SIZE\s+\d+.*$',
    '#define ATTEST_BLOCK_SIZE  1024  /* Stage-1 default; matrix: 64..4096 */')
  Set-Content -Path $Cfg -Value $ct2 -NoNewline
  Write-Host "Restored TABLE_I_RUNTIME=0, ATTEST_TABLE_I_REGION=0, shuffle=original, B=1024."
}
