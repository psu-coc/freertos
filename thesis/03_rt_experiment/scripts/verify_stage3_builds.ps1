<#
.SYNOPSIS
  Offline verify all 14 Stage-3 PRNG/FPE wall30 B-ladder builds (no flash).
#>
param()
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$od = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin\arm-none-eabi-objdump.exe"
$size = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.100.202509120712\tools\bin\arm-none-eabi-size.exe"
$Bs = @(64, 128, 256, 512, 1024, 2048, 4096)
$modes = @("prng", "fpe")
$fail = 0
$rows = @()

function Test-IrqAroundHmac([string]$elf) {
  $tmp = Join-Path $env:TEMP ("stage3_" + [guid]::NewGuid().ToString("N") + ".s")
  & $od -d $elf 2>$null | Set-Content -Path $tmp -Encoding utf8
  $raw = [IO.File]::ReadAllBytes($tmp)
  if ($raw.Length -ge 3 -and $raw[0] -eq 0xEF) { $raw = $raw[3..($raw.Length-1)] }
  $text = [Text.Encoding]::UTF8.GetString($raw)
  $idx = $text.IndexOf("<__acle_se_SECURE_ShuffledHMAC_secure>:")
  if ($idx -lt 0) { return $false }
  $chunk = $text.Substring($idx, [Math]::Min(120000, $text.Length - $idx))
  # Require cpsid ... hmac_sha256_update ... cpsie with B in mov #imm near update
  if ($chunk -notmatch "cpsid\s+i") { return $false }
  if ($chunk -notmatch "cpsie\s+i") { return $false }
  if ($chunk -notmatch "hmac_sha256_update") { return $false }
  # Placement: cpsid before bl hmac, cpsie after (within ~40 lines of each other in dump)
  $lines = $chunk -split "`n"
  $ok = $false
  for ($i = 0; $i -lt $lines.Count; $i++) {
    if ($lines[$i] -match "cpsid\s+i") {
      for ($j = $i + 1; $j -lt [Math]::Min($i + 25, $lines.Count); $j++) {
        if ($lines[$j] -match "bl\s+\S+\s+<hmac_sha256_update>") {
          for ($k = $j + 1; $k -lt [Math]::Min($j + 15, $lines.Count); $k++) {
            if ($lines[$k] -match "cpsie\s+i") { $ok = $true; break }
          }
        }
        if ($ok) { break }
      }
    }
    if ($ok) { break }
  }
  Remove-Item $tmp -Force -ErrorAction SilentlyContinue
  return $ok
}

foreach ($m in $modes) {
  foreach ($B in $Bs) {
    $dir = Join-Path $Root "thesis\03_rt_experiment\builds\10Hz10Hz_wall30_${m}_B$B"
    $s = Join-Path $dir "Secure.elf"
    $ns = Join-Path $dir "NonSecure.elf"
    $meta = Join-Path $dir "run_meta.txt"
    $status = "PASS"
    $notes = @()
    if (!(Test-Path $s) -or !(Test-Path $ns) -or !(Test-Path $meta)) {
      $status = "FAIL"; $notes += "missing ELF/meta"; $fail++
    } else {
      $mt = Get-Content -Raw $meta
      if ($mt -notmatch "ATTEST_SHUFFLE_MODE_NAME=$m") { $status = "FAIL"; $notes += "mode meta"; $fail++ }
      if ($mt -notmatch "ATTEST_BLOCK_SIZE=$B") { $status = "FAIL"; $notes += "B meta"; $fail++ }
      if ($mt -notmatch "ATTEST_IRQ_HMAC_MASK=1") { $status = "FAIL"; $notes += "IRQ meta"; $fail++ }
      if ($mt -notmatch "USE_SAU_APPROACH=0" -or $mt -notmatch "USE_SNAP=0") { $status = "FAIL"; $notes += "SAU/SNAP"; $fail++ }
      if ($mt -notmatch "ATTEST_REGION=0x08060000/0x20000") { $status = "FAIL"; $notes += "region"; $fail++ }
      if ($mt -notmatch "DMR=QUARANTINED") { $status = "FAIL"; $notes += "DMR"; $fail++ }
      if ($mt -notmatch "nm_ucHeap=32768") { $status = "FAIL"; $notes += "heap"; $fail++ }
      $irq = Test-IrqAroundHmac $s
      if (-not $irq) { $status = "FAIL"; $notes += "IRQ placement"; $fail++ }
      $sz = & $size $s 2>$null | Select-Object -Last 1
      $hs = (Get-FileHash $s -Algorithm SHA256).Hash.Substring(0, 16)
      $hns = (Get-FileHash $ns -Algorithm SHA256).Hash.Substring(0, 16)
      $rows += [pscustomobject]@{
        Mode = $m; B = $B; Status = $status; SecureSha16 = $hs; NsSha16 = $hns
        SizeLine = ($sz -replace '\s+', ' ').Trim(); Notes = ($notes -join ';')
      }
      Write-Host ("{0} B{1}: {2} S={3} NS={4} {5}" -f $m, $B, $status, $hs, $hns, ($notes -join ';'))
    }
  }
}

$out = Join-Path $Root "thesis\03_rt_experiment\analysis\stage3_bsweep_readiness"
New-Item -ItemType Directory -Force -Path $out | Out-Null
$csv = Join-Path $out "BUILD_VERIFY_14.csv"
$rows | Export-Csv -NoTypeInformation -Path $csv
Write-Host "wrote $csv fails=$fail"
if ($fail -gt 0) { exit 1 } else { exit 0 }
