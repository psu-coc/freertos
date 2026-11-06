<#
.SYNOPSIS
  Build (if needed) + flash Secure/NS + capture one Wheel1000Hz wall3 attest run.
  Tags: phase4r_w1000_{original|prng|fpe}_B{B}_r{N}  — never overwrites.
#>
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet("original", "prng", "fpe")][string]$ShuffleMode,
  [Parameter(Mandatory = $true)]
  [ValidateSet(64, 128, 256, 512, 1024, 2048, 4096)][int]$B,
  [Parameter(Mandatory = $true)]
  [ValidateRange(1, 3)][int]$Run,
  [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$BuildScript = Join-Path $PSScriptRoot "build_rt_profile.ps1"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

$tag = "phase4r_w1000_${ShuffleMode}_B${B}_r$Run"
$Build = Join-Path $Root "thesis\03_rt_experiment\builds\Wheel1000Hz_wall3_${ShuffleMode}_B$B"
$uartOut = Join-Path $LogDir "${tag}_uart_capture.txt"

if (Test-Path $uartOut) { throw "Refuse overwrite: $uartOut" }
foreach ($bad in @("phase2r_", "phase3r_", "phase2w_", "phase3w_", "phase4r_w100_")) {
  if ($tag.StartsWith($bad) -and $tag -notmatch "w1000") { throw "Refusing corpus tag collision $tag" }
}

if (-not $SkipBuild) {
  Write-Host "BUILD Wheel1000Hz $ShuffleMode B=$B"
  & powershell -NoProfile -ExecutionPolicy Bypass -File $BuildScript `
    -Profile Wheel1000Hz -RunLen wall3 -ShuffleMode $ShuffleMode -B $B
  if ($LASTEXITCODE -ne 0) { throw "Build failed" }
}

$sElf = Join-Path $Build "Secure.elf"
$nsElf = Join-Path $Build "NonSecure.elf"
$meta = Join-Path $Build "run_meta.txt"
if (!(Test-Path $nsElf)) { throw "Missing $nsElf" }
if (!(Test-Path $sElf)) { throw "Missing $sElf" }
$metaText = Get-Content -Raw $meta
if ($metaText -notmatch "profile=Wheel1000Hz") { throw "Build not Wheel1000Hz" }
if ($metaText -notmatch "PHASE3_WITH_ORIGINAL_SMARM=1") { throw "Attest not enabled" }
if ($metaText -notmatch "ATTEST_SHUFFLE_MODE_NAME=$ShuffleMode") { throw "Shuffle mismatch" }
if ($metaText -notmatch "ATTEST_BLOCK_SIZE=$B") { throw "B mismatch" }
if ($metaText -notmatch "ATTEST_IRQ_HMAC_MASK=1") { throw "IRQ_HMAC mismatch" }

$sSha = (Get-FileHash $sElf -Algorithm SHA256).Hash.Substring(0, 16)
$nSha = (Get-FileHash $nsElf -Algorithm SHA256).Hash.Substring(0, 16)
Write-Host "CAPTURE $tag Secure=$sSha NS=$nSha"

& $prog -c port=SWD mode=UR -d $sElf
if ($LASTEXITCODE -ne 0) { throw "Secure flash failed" }
& $prog -c port=SWD mode=UR -d $nsElf
if ($LASTEXITCODE -ne 0) { throw "NS flash failed" }

Copy-Item $meta (Join-Path $LogDir "${tag}_run_meta.txt") -Force
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "flash=PERFORMED_wheel1000_m6"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "tag=$tag"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "secure_sha16=$sSha"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "ns_sha16=$nSha"

# wall3: warm 2s + obs 3s + dump; large B may stretch dump wait
$timeoutSec = if ($B -ge 4096) { 90 } elseif ($B -ge 2048) { 70 } else { 55 }

python -c @"
import serial, time, subprocess, sys, re
from pathlib import Path
tag = r'$tag'
base = Path(r'$LogDir')
out = base / f'{tag}_uart_capture.txt'
prog = r'$prog'
timeout = $timeoutSec
mode = r'$ShuffleMode'
B = $B
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
        if 'END wheel_timing.csv' in text and 'PHASE3 done' in text:
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
    print('INCOMPLETE', file=sys.stderr); sys.exit(3)
for s in ('Wheel1000Hz', 'wall3', 'AttestTask', f'B={B}', 'BEGIN wheel_timing.csv', 'tick_delta='):
    if s not in text:
        print('missing', s, file=sys.stderr); sys.exit(4)
if mode == 'original':
    if 'Original SMARM' not in text and 'shuffle_mode=original' not in text:
        if 'Original SMARM' not in text:
            print('mode mismatch original', file=sys.stderr); sys.exit(5)
else:
    want = {'prng': 'shuffle_mode=prng', 'fpe': 'shuffle_mode=fpe'}[mode]
    if want not in text:
        print('mode mismatch', want, file=sys.stderr); sys.exit(5)
if 'SecureFault' in text or 'HardFault' in text:
    print('FAULT', file=sys.stderr); sys.exit(6)
if 'incomplete=1' in text:
    print('attest incomplete=1', file=sys.stderr); sys.exit(7)

def extract(begin, end, dest):
    i, j = text.find(begin), text.find(end)
    if i < 0 or j <= i:
        print('MISSING', begin.strip()); return False
    dest.write_text(text[i+len(begin):j].replace('\r\n','\n').strip()+'\n', encoding='utf-8')
    return True
ok = extract('BEGIN wheel_timing.csv\r\n', 'END wheel_timing.csv', base/f'{tag}_wheel_timing.csv')
extract('BEGIN obs_per_second.csv\r\n', 'END obs_per_second.csv', base/f'{tag}_obs_per_second.csv')
if not ok:
    sys.exit(8)
m = re.search(r'wheel_n=(\d+).*wheel_ovf=(\d+)', text)
td = re.search(r'tick_delta=(\d+)', text)
print(f\"wheel_n={m.group(1) if m else '?'} ovf={m.group(2) if m else '?'} tick_delta={td.group(1) if td else '?'}\")
print('CAPTURE OK', tag)
"@
if ($LASTEXITCODE -ne 0) { throw "Capture failed exit=$LASTEXITCODE" }
Write-Host "OK $tag"
