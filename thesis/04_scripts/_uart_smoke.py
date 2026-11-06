import serial
import sys
import time

port = "COM3"
baud = 115200
ser = serial.Serial(port, baud, timeout=0.2)
buf = b""
deadline = time.time() + 50
print(f"Reading {port} for up to 50s...", flush=True)
while time.time() < deadline:
    chunk = ser.read(4096)
    if chunk:
        buf += chunk
        sys.stdout.write(chunk.decode("utf-8", errors="replace"))
        sys.stdout.flush()
        if b"Done. Press RESET" in buf:
            time.sleep(0.5)
            chunk = ser.read(4096)
            if chunk:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
            break
ser.close()
if not buf:
    print("NO UART OUTPUT")
    sys.exit(2)
if b"Returned from secure attestation" in buf or b"FAR mean=" in buf:
    sys.exit(0)
print("\n[partial output; attestation may still be running]")
sys.exit(1)
