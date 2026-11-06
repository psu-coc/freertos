<#
.SYNOPSIS
  Flash archived Table I build and capture 10-round UART on COM3.
  Tags: table_i_{mode}_B{B}
#>
param(
  [ValidateSet("original", "prng", "fpe")][string]$ShuffleMode = "original",
  [ValidateSet(64, 128, 256, 512, 1024, 2048, 4096)][int]$B = 1024,
  [switch]$SkipBuild,
  [int]$TimeoutSec = 180
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$LogDir = Join-Path $Root "thesis\03_rt_experiment\table_i_runtime\logs"
$BuildScript = Join-Path $PSScriptRoot "build_table_i.ps1"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

$tag = "table_i_${ShuffleMode}_B$B"
$buildTag = "TableI_${ShuffleMode}_B$B"
$Build = Join-Path $Root "thesis\03_rt_experiment\builds\$buildTag"
$uartOut = Join-Path $LogDir "${tag}_uart_capture.txt"
if (Test-Path $uartOut) { throw "Refuse overwrite: $uartOut" }

# Longer timeout for small-B PRNG/FPE (Access: B64 PRNG ~9.3 s/round)
if ($TimeoutSec -eq 180) {
  if ($B -le 128 -and $ShuffleMode -ne "original") { $TimeoutSec = 300 }
  if ($B -eq 64 -and $ShuffleMode -eq "prng") { $TimeoutSec = 420 }
}

if (-not $SkipBuild) {
  & powershell -NoProfile -ExecutionPolicy Bypass -File $BuildScript `
    -ShuffleMode $ShuffleMode -B $B
  if ($LASTEXITCODE -ne 0) { throw "Build failed" }
}

$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
$meta = Join-Path $Build "run_meta.txt"
if (!(Test-Path $nsElf) -or !(Test-Path $sElf)) { throw "Missing ELF in $Build" }
$metaText = Get-Content -Raw $meta
if ($metaText -notmatch "protocol=Access_TableI_runtime_512KiB_noRT") { throw "Not a Table I build" }
if ($metaText -notmatch "ATTEST_SHUFFLE_MODE_NAME=$ShuffleMode") { throw "Shuffle mismatch" }
if ($metaText -notmatch "ATTEST_BLOCK_SIZE=$B") { throw "B mismatch" }
if ($metaText -notmatch "ATTEST_REGION=0x08000000/0x80000") { throw "Region mismatch" }
if ($metaText -notmatch "NormalTask=0") { throw "NormalTask must be 0" }

Write-Host "FLASH $tag"
& $prog -c port=SWD mode=UR -d $sElf
if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

Copy-Item $meta (Join-Path $LogDir "${tag}_run_meta.txt") -Force
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "tag=$tag"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "flash=PERFORMED_table_i"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "timeout_sec=$TimeoutSec"

python -c @"
import serial, time, subprocess, sys, re, statistics
from pathlib import Path
tag = r'$tag'
base = Path(r'$LogDir')
out = base / f'{tag}_uart_capture.txt'
prog = r'$prog'
timeout = $TimeoutSec
mode = r'$ShuffleMode'
B = $B
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
        if 'SUMMARY Table I runtime' in text and 'Done. Press RESET' in text:
            time.sleep(0.3)
            if port.in_waiting:
                buf.append(port.read(port.in_waiting).decode('utf-8', errors='replace'))
            done = True
            break
        if 'ABORT:' in text or 'SecureFault' in text or 'HardFault' in text:
            time.sleep(0.2)
            if port.in_waiting:
                buf.append(port.read(port.in_waiting).decode('utf-8', errors='replace'))
            break
    else:
        time.sleep(0.05)
port.close()
text = ''.join(buf)
out.write_text(text, encoding='utf-8')
print(f'saved {out.name} bytes={len(text)} done={done}')
if 'ABORT:' in text:
    print('ABORT in UART', file=sys.stderr); sys.exit(3)
if 'SecureFault' in text or 'HardFault' in text:
    print('FAULT', file=sys.stderr); sys.exit(3)
if not done:
    print('INCOMPLETE', file=sys.stderr); sys.exit(2)
if '|M|=512KiB @0x08000000' not in text and 'Table I runtime' not in text:
    print('missing Table I banner', file=sys.stderr); sys.exit(4)
if 'no NormalTask' not in text:
    print('missing no-NormalTask marker', file=sys.stderr); sys.exit(5)
rounds = re.findall(
    r'Round (\d+): Runtime=(\d+) ms, NS=(\d+) \(no NormalTask\)',
    text)
if len(rounds) < 10:
    print(f'expected 10 Table I rounds, got {len(rounds)}', file=sys.stderr); sys.exit(6)
for r, rt, ns in rounds[:10]:
    if int(ns) != 0:
        print(f'round {r} NS={ns} != 0', file=sys.stderr); sys.exit(7)
rts = [int(rt) for _, rt, _ in rounds[:10]]
csv = base / f'{tag}_rounds.csv'
lines = ['round,runtime_ms,ns_activations']
for r, rt, ns in rounds[:10]:
    lines.append(f'{r},{rt},{ns}')
csv.write_text('\n'.join(lines) + '\n', encoding='utf-8')
mean = statistics.fmean(rts)
med = statistics.median(rts)
stdev = statistics.pstdev(rts) if len(rts) > 1 else 0.0
print(f'mean={mean:.1f} median={med:.1f} min={min(rts)} max={max(rts)} pstdev={stdev:.3f}')
print('CAPTURE OK', tag)
"@
if ($LASTEXITCODE -ne 0) { throw "Capture failed exit=$LASTEXITCODE" }
Write-Host "CAPTURE OK $tag"
