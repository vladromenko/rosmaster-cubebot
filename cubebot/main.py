"""Entry point: python3 -m cubebot.main [--config FILE] [--dry-run] [--diagnose]"""
import argparse
import signal
import sys
import time

from . import config
from .arm_model import ArmModel


def main(argv=None):
    ap = argparse.ArgumentParser(description="Find the yellow cube, put it in the box, take it out again.")
    ap.add_argument("--config", help="JSON file overriding config defaults")
    ap.add_argument("--dry-run", action="store_true", help="read sensors and plan, but publish zero velocity and no arm commands")
    ap.add_argument("--diagnose", action="store_true", help="print sensor/vision diagnostics and exit (no motion)")
    ap.add_argument("--preflight-only", action="store_true",
                    help="move the arm to the search pose and run the camera/arm self-check; the base never moves")
    ap.add_argument("--arm-now", default="90,90,90,90,90,90",
                    help="current commanded arm pose (the firmware reports no joint angles)")
    ap.add_argument("--image-out", default="/tmp/cubebot_diag.png")
    args = ap.parse_args(argv)

    from .ros_io import RosRobot  # imported late so unit tests need no ROS
    cfg = config.load(args.config)
    arm = ArmModel(cfg["arm"]["tcp_offset_m"], optical_rotation=cfg["camera"]["optical_rotation"])

    def log(msg):
        print("%s %s" % (time.strftime("%H:%M:%S"), msg), flush=True)

    io = RosRobot(cfg, arm, log=log, dry_run=args.dry_run or args.diagnose)
    io._arm_cmd = [float(v) for v in args.arm_now.split(",")]
    signal.signal(signal.SIGTERM, lambda *_: io.request_abort("SIGTERM"))
    signal.signal(signal.SIGINT, lambda *_: io.request_abort("SIGINT"))
    try:
        end = time.monotonic() + 15
        while time.monotonic() < end and not all(io.sensors_alive().values()):
            time.sleep(0.2)
        alive = io.sensors_alive()
        log("sensors: %s" % alive)
        if args.diagnose:
            from .diagnose import run
            run(io, cfg, log, args.image_out)
            return 0
        if not all(alive.values()):
            log("refusing to start: sensors missing: %s" % [k for k, v in alive.items() if not v])
            return 2
        from .mission import Mission
        mission = Mission(io, cfg, log)
        if args.preflight_only:
            mission.state = "PREFLIGHT"
            mission.s_preflight()
            from .diagnose import run
            run(io, cfg, log, args.image_out)
            return 0
        ok = mission.run()
        return 0 if ok else 1
    finally:
        io.shutdown()


if __name__ == "__main__":
    sys.exit(main())
