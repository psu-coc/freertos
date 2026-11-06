<#
.SYNOPSIS
  Flash + capture one wall30 PRNG/FPE campaign run (Stage 3).
.DESCRIPTION
  Tags: phase3r_{prng|fpe}_B{B}_r{1..3}
  NEVER overwrites: Original phase3r_B*_r*, Stage-2 smoke phase3r_smoke_*,
  or an existing campaign capture for the same tag.
  Does NOT run unless invoked explicitly after Stage-3 authorization.
.PARAMETER ShuffleMode
  prng | fpe
.PARAMETER Mode
  B64 | B128 | B256 | B512 | B1024 | B2048 | B4096
.PARAMETER Run
  Repetition index 1..3
#>
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet("prng", "fpe")][string]$ShuffleMode,
  [Parameter(Mandatory = $true)]
  [ValidateSet("B64", "B128", "B256", "B512", "B1024", "B2048", "B4096")][string]$Mode,
  [Parameter(Mandatory = $true)]
  [ValidateRange(1, 3)][int]$Run
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

$B = [int]($Mode.Substring(1))
$Build = Join-Path $Root "thesis\03_rt_experiment\builds\10Hz10Hz_wall30_${ShuffleMode}_B$B"
$tag = "phase3r_${ShuffleMode}_B${B}_r$Run"

# Hard refuse Original campaign / smoke / legacy collisions
if ($tag -match '^phase3r_B\d+_r') {
  throw "Refusing Original campaign-style tag $tag"
}
if ($tag -match '^phase3r_smoke_') {
  throw "Refusing Stage-2 smoke tag collision $tag"
}
foreach ($legacy in @("phase2w_", "phase3w_", "phase2_10Hz10Hz_", "phase3_10Hz10Hz_")) {
  if ($tag.StartsWith($legacy.TrimEnd('_'))) { throw "Refusing legacy tag $tag" }
}

$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
if (!(Test-Path $nsElf)) { throw "Missing NS ELF in $Build - build first" }
if (!(Test-Path $sElf)) { throw "Missing Secure ELF in $Build - build first" }

$meta = Join-Path $Build "run_meta.txt"
if (!(Test-Path $meta)) { throw "Missing run_meta.txt in $Build" }
$metaText = Get-Content -Raw $meta
if ($metaText -notmatch "ATTEST_SHUFFLE_MODE_NAME=$ShuffleMode") {
  throw "Build meta shuffle mismatch for $Build"
}
if ($metaText -notmatch "ATTEST_BLOCK_SIZE=$B") {
  throw "Build meta B mismatch for $Build"
}
if ($metaText -notmatch "ATTEST_IRQ_HMAC_MASK=1") {
  throw "Build meta IRQ_HMAC must be 1"
}
if ($metaText -notmatch "USE_SAU_APPROACH=0" -or $metaText -notmatch "USE_SNAP=0") {
  throw "Build meta requires USE_SAU=0 USE_SNAP=0"
}

# No-overwrite: any existing UART for this tag aborts
$uartOut = Join-Path $LogDir "${tag}_uart_capture.txt"
if (Test-Path $uartOut) {
  throw "Capture already exists: $uartOut - refuse overwrite (bump Run or archive)"
}

# Also refuse if Stage-2 smoke is being "promoted" by alias - smoke tags are different.
# Policy: Stage-2 B1024 smoke is NEVER counted as campaign r1/r2/r3.

Write-Host "STAGE3 CAPTURE: mode=$ShuffleMode $Mode Run=$Run tag=$tag"
Write-Host "Secure SHA16=$((Get-FileHash $sElf -Algorithm SHA256).Hash.Substring(0,16))"
Write-Host "NS     SHA16=$((Get-FileHash $nsElf -Algorithm SHA256).Hash.Substring(0,16))"
Write-Host "REVIEW GATE: invoke only after Stage-3 authorization."

& $prog -c port=SWD mode=UR -d $sElf
if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

Copy-Item $meta (Join-Path $LogDir "${tag}_run_meta.txt") -Force
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "flash=PERFORMED_stage3_bsweep"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "tag=$tag"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "secure_sha16=$((Get-FileHash $sElf -Algorithm SHA256).Hash.Substring(0,16))"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "ns_sha16=$((Get-FileHash $nsElf -Algorithm SHA256).Hash.Substring(0,16))"

# Same timeout policy as Original wall30 capture
$timeoutSec = if ($B -ge 4096) { 120 } elseif ($B -ge 2048) { 90 } else { 90 }

python -c @"
import serial, time, subprocess, sys
from pathlib import Path
tag = r'$tag'
base = Path(r'$LogDir')
out = base / f'{tag}_uart_capture.txt'
prog = r'$prog'
timeout = $timeoutSec
if out.exists():
    print('REFUSE overwrite', out); sys.exit(2)
port = serial.Serial('COM3', 115200, timeout=0.2)
buf, t0, done = [], time.time(), False
print(f'listening {tag} timeout={timeout}s ...', flush=True)
subprocess.run([prog, '-c', 'port=SWD', 'mode=UR', '-rst', '-hardRst'], capture_output=True)
time.sleep(0.3)
while time.time() - t0 < timeout:
    n = port.in_waiting
    if n:
        chunk = port.read(n).decode('utf-8', errors='replace')
        buf.append(chunk)
        text = ''.join(buf)
        if 'END obs_per_second.csv' in text and 'END wheel' in text:
            if any(m in text for m in ('PHASE3 done', 'baseline done', 'Idle.')):
                time.sleep(0.5)
                if port.in_waiting:
                    buf.append(port.read(port.in_waiting).decode('utf-8', errors='replace'))
                done = True
                break
    else:
        time.sleep(0.05)
port.close()
text = ''.join(buf)
out.write_text(text, encoding='utf-8')
print(f'saved {out.name} bytes={len(text)} done={done}')
if not done:
    print('INCOMPLETE capture (timeout or missing END markers)', file=sys.stderr)
    sys.exit(3)

def extract(begin, end, dest):
    i, j = text.find(begin), text.find(end)
    if i < 0 or j <= i:
        print('MISSING', begin.strip()); return False
    dest.write_text(text[i+len(begin):j].replace('\r\n','\n').strip()+'\n', encoding='utf-8')
    return True
ok = True
ok &= extract('BEGIN obs_per_second.csv\r\n', 'END obs_per_second.csv', base/f'{tag}_obs_per_second.csv')
ok &= extract('BEGIN sonar_timing.csv\r\n', 'END sonar_timing.csv', base/f'{tag}_sonar_timing.csv')
ok &= extract('BEGIN wheel_timing.csv\r\n', 'END wheel_timing.csv', base/f'{tag}_wheel_timing.csv')
# overflow fields appear in obs header when present
ovf_s = 'sonar_ovf=' in text
ovf_w = 'wheel_ovf=' in text
print(f'extract_ok={ok} sonar_ovf_field={ovf_s} wheel_ovf_field={ovf_w}')
if not ok:
    sys.exit(4)
"@

if ($LASTEXITCODE -ne 0) { throw "UART capture/parse failed exit=$LASTEXITCODE" }
Write-Host "Done $tag - analyze with analyze_bsweep_wall30.py --shuffle-mode $ShuffleMode"
