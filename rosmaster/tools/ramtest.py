#!/usr/bin/env python3
"""Hardware check of a firmware image started from RAM: identity, telemetry rates, servo readings, loader jump."""
import sys, time
sys.path.insert(0, __file__.rsplit("/rosmaster/", 1)[0])
from rosmaster import link as L

lk = L.Link()
info = lk.hello()
print("HELLO:", info)
if not info:
    print("no identity answer"); sys.exit(2)
time.sleep(4)
names = {L.STATUS: "STATUS", L.SERVO: "SERVO", L.IMU: "IMU", L.BATTERY: "BATTERY", L.LIDAR0: "LIDAR0", L.LIDAR1: "LIDAR1", L.ARM_DIAG: "ARM_DIAG"}
print("frames in ~4 s:", {names.get(k, k): v for k, v in sorted(lk.counts.items())})
if L.BATTERY in lk.latest: print("battery:", L.Link.battery(lk.latest[L.BATTERY][1]))
seen = {}
lk.callbacks.append(lambda k, p: seen.__setitem__(L.Link.servo(p)["joint"], L.Link.servo(p)) if k == L.SERVO else None)
time.sleep(3)
for j in sorted(seen): print("servo", seen[j])
print("parser errors:", lk.parser.errors)
if "--jump" in sys.argv:
    print("sending loader request"); lk.enter_bootloader(); time.sleep(0.5)
lk.close()
