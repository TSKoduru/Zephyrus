#!/usr/bin/env python3
"""Decisive UART test. Jumper Jetson 40-pin pin 8 (TX) <-> pin 10 (RX), then run.

  bytes returned  -> Jetson UART is fine; the fault is the cable or the Pixhawk.
  nothing returned-> the Jetson RX path itself is broken (pinmux/hardware).

Remove the jumper and reconnect the Pixhawk afterwards.
"""
import serial, sys, time

dev  = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyTHS1'
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
pat  = bytes(range(256)) * 4          # 1024 bytes, easy to eyeball

s = serial.Serial(dev, baud, timeout=0.5)
s.reset_input_buffer(); s.reset_output_buffer()
s.write(pat); s.flush()

got = bytearray(); t0 = time.time()
while time.time() - t0 < 2.0 and len(got) < len(pat):
    got += s.read(4096)
s.close()

print(f"{dev} @ {baud}: sent {len(pat)} B, got {len(got)} B")
if not got:
    print("RESULT: NO LOOPBACK -- nothing arrived on RX.")
    print("        With the jumper fitted this means the Jetson RX path is broken.")
    print("        Without the jumper this test proves nothing; fit it first.")
elif bytes(got[:len(pat)]) == pat:
    print("RESULT: LOOPBACK OK -- Jetson TX and RX both work.")
    print("        The fault is downstream: wiring, wrong port, or the Pixhawk.")
else:
    print("RESULT: PARTIAL/CORRUPT -- data returned but does not match.")
    print(f"        first 32 sent: {pat[:32].hex(' ')}")
    print(f"        first 32 recv: {bytes(got[:32]).hex(' ')}")
    print("        Suggests a baud/framing problem rather than a dead line.")
