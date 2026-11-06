<#
.SYNOPSIS
  M2 smoke: flash Wheel100Hz wall30 NoAttest NS image and capture UART.
  Never overwrites an existing tag. Does not touch phase2r_/phase3r_ corpus.
#>
param(
  [ValidateRange(0, 99)][int]$Run = 0,
  [int]$TimeoutSec = 100
)
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$Build = Join-Path $Root "thesis\03_rt_experiment\builds\Wheel100Hz_wall30_noattest"
$Elf = Join-Path $Build "NonSecure.elf"
$LogDir = Join-Path $Root "thesis\03_rt_experiment\logs"
$prog = "C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.300.202508131133\tools\bin\STM32_Programmer_CLI.exe"

if (!(Test-Path $Elf)) { throw "Missing ELF: $Elf" }

$tag = if ($Run -eq 0) { "phase4r_w100_noattest_smoke" } else { "phase4r_w100_noattest_r$Run" }
$uartOut = Join-Path $LogDir "${tag}_uart_capture.txt"
if (Test-Path $uartOut) { throw "Refuse overwrite: $uartOut" }

Write-Host "Flash NS $Elf"
& $prog -c port=SWD mode=UR -w $Elf 0x08040000 -v -rst
if ($LASTEXITCODE -ne 0) { throw "Flash failed" }

Copy-Item (Join-Path $Build "run_meta.txt") (Join-Path $LogDir "${tag}_run_meta.txt") -Force
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "flash=PERFORMED_NS_ONLY"
Add-Content (Join-Path $LogDir "${tag}_run_meta.txt") "tag=$tag"

python -c @"
import serial, time, subprocess, sys
from pathlib import Path
tag = r'$tag'
base = Path(r'$LogDir')
out = base / f'{tag}_uart_capture.txt'
prog = r'$prog'
timeout = $TimeoutSec
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
        if 'END wheel_timing.csv' in text and ('No-attest done' in text or 'PHASE3 done' in text):
            time.sleep(0.4)
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
    print('INCOMPLETE', file=sys.stderr)
    sys.exit(2)
for s in ('Wheel100Hz', 'BEGIN wheel_timing.csv', 'tick_delta='):
    if s not in text:
        print('missing', s, file=sys.stderr)
        sys.exit(3)
if 'AttestTask:' in text:
    print('UNEXPECTED AttestTask', file=sys.stderr)
    sys.exit(4)
if 'SecureFault' in text or 'HardFault' in text:
    print('FAULT', file=sys.stderr)
    sys.exit(5)

def extract(begin, end, dest):
    i, j = text.find(begin), text.find(end)
    if i < 0 or j <= i:
        print('MISSING', begin.strip())
        return
    dest.write_text(text[i+len(begin):j].replace('\r\n','\n').strip()+'\n', encoding='utf-8')
extract('BEGIN obs_per_second.csv\r\n', 'END obs_per_second.csv', base/f'{tag}_obs_per_second.csv')
extract('BEGIN wheel_timing.csv\r\n', 'END wheel_timing.csv', base/f'{tag}_wheel_timing.csv')
print('SMOKE CAPTURE OK', tag)
"@
if ($LASTEXITCODE -ne 0) { throw "Capture/validate failed exit=$LASTEXITCODE" }
Write-Host "OK $tag"
