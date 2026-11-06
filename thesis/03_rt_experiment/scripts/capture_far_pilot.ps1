<#
.SYNOPSIS
  Flash archived FAR build and capture 10-round UART on COM3.
  Tags: far_pilot_1000_{mode}_B{B}[_farref]
#>
param(
  [ValidateSet("original", "prng", "fpe")][string]$ShuffleMode = "original",
  [ValidateSet(64, 128, 256, 512, 1024, 2048, 4096)][int]$B = 1024,
  [switch]$FarRef,
  [switch]$SkipBuild,
  [int]$TimeoutSec = 90
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$BuildScript = Join-Path $PSScriptRoot "build_far_pilot.ps1"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

$tag = if ($FarRef) { "far_pilot_1000_${ShuffleMode}_B${B}_farref" } else { "far_pilot_1000_${ShuffleMode}_B$B" }
$buildTag = if ($FarRef) { "FAR1000Hz_${ShuffleMode}_B${B}_farref" } else { "FAR1000Hz_${ShuffleMode}_B$B" }
$Build = Join-Path $Root "thesis\03_rt_experiment\builds\$buildTag"
$uartOut = Join-Path $LogDir "${tag}_uart_capture.txt"
if (Test-Path $uartOut) { throw "Refuse overwrite: $uartOut" }

if (-not $SkipBuild) {
  if ($FarRef) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File $BuildScript `
      -ShuffleMode $ShuffleMode -B $B -FarRef
  } else {
    & powershell -NoProfile -ExecutionPolicy Bypass -File $BuildScript `
      -ShuffleMode $ShuffleMode -B $B
  }
  if ($LASTEXITCODE -ne 0) { throw "Build failed" }
}

$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
$meta = Join-Path $Build "run_meta.txt"
if (!(Test-Path $nsElf) -or !(Test-Path $sElf)) { throw "Missing ELF in $Build" }
$metaText = Get-Content -Raw $meta
if ($metaText -notmatch "protocol=dummy_FAR_1000Hz") { throw "Not a FAR build" }
if ($metaText -notmatch "ATTEST_SHUFFLE_MODE_NAME=$ShuffleMode") { throw "Shuffle mismatch" }
if ($metaText -notmatch "ATTEST_BLOCK_SIZE=$B") { throw "B mismatch" }
$wantRef = if ($FarRef) { 1 } else { 0 }
if ($metaText -notmatch "FAR_REF_NO_IRQ_DISABLE=$wantRef") { throw "FarRef mismatch" }

Write-Host "FLASH $tag"
& $prog -c port=SWD mode=UR -d $sElf
if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

Copy-Item $meta (Join-Path $LogDir "${tag}_run_meta.txt") -Force
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "tag=$tag"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "flash=PERFORMED_far_pilot"

python -c @"
import serial, time, subprocess, sys, re
from pathlib import Path
tag = r'$tag'
base = Path(r'$LogDir')
out = base / f'{tag}_uart_capture.txt'
prog = r'$prog'
timeout = $TimeoutSec
mode = r'$ShuffleMode'
B = $B
farref = $wantRef
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
        if 'FAR mean=' in text and 'Done. Press RESET' in text:
            time.sleep(0.3)
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
    print('INCOMPLETE', file=sys.stderr); sys.exit(2)
if 'SecureFault' in text or 'HardFault' in text:
    print('FAULT', file=sys.stderr); sys.exit(3)
# expect 10 Round lines with FAR=
rounds = re.findall(r'Round (\d+): Runtime=(\d+) ms, NS=(\d+)/(\d+) activations, FAR=(\d+)\.(\d{4})', text)
if len(rounds) < 10:
    print(f'expected 10 FAR rounds, got {len(rounds)}', file=sys.stderr); sys.exit(4)
m = re.search(r'FAR mean=(\d+)\.(\d{4}), min=(\d+)\.(\d{4}), max=(\d+)\.(\d{4})', text)
if not m:
    print('missing FAR mean summary', file=sys.stderr); sys.exit(5)
# t_disabled per round (avg/max cycles); align with Round lines
tdis = re.findall(
    r't_disabled avg=(\d+) cy \(.*?\)\, max=(\d+) cy',
    text)
if len(tdis) < 10:
    print(f'expected 10 t_disabled lines, got {len(tdis)}', file=sys.stderr); sys.exit(6)
# write CSV
csv = base / f'{tag}_rounds.csv'
lines = ['round,runtime_ms,actual,expected,far,tdis_avg_cy,tdis_max_cy']
for (r, rt, act, exp, fi, ff), (ta, tm) in zip(rounds, tdis):
    far = int(fi) + int(ff) / 10000.0
    lines.append(f'{r},{rt},{act},{exp},{far:.4f},{ta},{tm}')
csv.write_text('\n'.join(lines) + '\n', encoding='utf-8')
mean = int(m.group(1)) + int(m.group(2)) / 10000.0
print(f'FAR_mean={mean:.4f} rounds={len(rounds)} mode={mode} B={B} farref={farref}')
print('CAPTURE OK', tag)
"@
if ($LASTEXITCODE -ne 0) { throw "Capture failed exit=$LASTEXITCODE" }
Write-Host "OK $tag"
