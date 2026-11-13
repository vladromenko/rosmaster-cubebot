#!/usr/bin/env python3
"""Run on the 16 GB Jetson (where LeRobot and the SO101 leader live): stream the leader's joints to the Rosmaster.

    python3 so101_mirror_sender.py --port /dev/ttyACM0 --id my_leader --host explorer.local

The Rosmaster console must have mirror mode switched on (button "Зеркало"); until then the data is ignored.
Uses LeRobot's SO101 leader in degrees; adjust the import if your LeRobot version names it differently.
"""
import argparse
import asyncio
import json
import time

try:
    import websockets
except ImportError:
    raise SystemExit("pip install websockets")


def open_leader(port, ident):
    try:
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    except ImportError as exc:
        raise SystemExit("LeRobot SO101 leader not importable (%s). Edit open_leader() for your LeRobot version." % exc)
    leader = SO101Leader(SO101LeaderConfig(port=port, id=ident))
    leader.connect(calibrate=False)
    return leader


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM1")
    ap.add_argument("--id", default="so101_leader")
    ap.add_argument("--host", default="192.168.0.177")
    ap.add_argument("--hz", type=float, default=30.0)
    args = ap.parse_args()
    leader = open_leader(args.port, args.id)
    while True:
        try:
            async with websockets.connect("ws://%s:8090/mirror" % args.host) as ws:
                print("connected to", args.host)
                while True:
                    action = leader.get_action()
                    msg = {k.split(".")[0]: float(v) for k, v in action.items()}
                    await ws.send(json.dumps(dict(leader=msg, t=time.time())))
                    await asyncio.sleep(1.0 / args.hz)
        except (OSError, websockets.ConnectionClosed) as exc:
            print("link lost (%s); retrying" % exc)
            await asyncio.sleep(1.0)


if __name__ == "__main__":
    asyncio.run(main())
