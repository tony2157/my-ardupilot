#!/usr/bin/env python3
"""
Draw GPS accuracy squares in Gazebo around where ArduPilot believes the drone is.

The squares are Gazebo markers (visual only, no physics). They are centred on ArduPilot's
estimated position (EKF, GLOBAL_POSITION_INT: lat/lon and height above home) converted to Gazebo
world coordinates, level with the ground and turned with the drone's heading. The horizontal offset
shows the GPS error; the vertical offset shows the EKF height error, which follows the GPS height
because the RFSonde GPS parameter files set EK3_SRC1_POSZ = 3. The drone model in Gazebo is always at its true
position, so it wanders inside the squares by the position error:

  red    1 m square    horizontal + vertical (lateral plane)
  orange 60 cm square  horizontal + vertical (lateral plane)

Usage. rfsonde_sitl.sh starts this helper automatically; to run it by hand (SITL accepts this extra
MAVLink connection on TCP 5763; the helper waits for SITL and reconnects if SITL restarts):
  python3 accuracy_squares.py                  # draw the squares until Ctrl+C (removes them)
  python3 accuracy_squares.py --show large     # only the 1 m square (or: small = 60 cm)

Switch the running helper on and off from any terminal:
  python3 accuracy_squares.py --off            # hide the squares
  python3 accuracy_squares.py --on             # show both squares again
  python3 accuracy_squares.py --on --show small
"""
import argparse
import contextlib
import io
import math
import os
import signal
import sys
import time

from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.marker_pb2 import Marker
from gz.msgs.marker_v_pb2 import Marker_V
from gz.transport import Node
from pymavlink import mavutil

NS = "rfsonde_accuracy"
LATLON_TO_M = 0.011131884502145034          # ArduPilot: metres per 1e-7 degree (spherical Earth)
SQUARES = {                                  # name: (side m, bar thickness m, rgb, first marker id)
    "large": (1.0, 0.02, (1.0, 0.0, 0.0), 1),     # marker id 0 means "new marker" to Gazebo, so ids start at 1
    "small": (0.6, 0.015, (1.0, 0.5, 0.0), 11),
}
CONTROL = os.path.expanduser("~/.cache/rfsonde/accuracy_squares")   # "off" or the squares to show


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5763", help="SITL MAVLink connection (default %(default)s)")
    ap.add_argument("--home", default="35.2380802,-97.4631246,358.6",
                    help="Gazebo world origin = SITL home LAT,LON,ALT (default: RFSonde_Norman)")
    ap.add_argument("--home-height", type=float, default=0.195,
                    help="Gazebo height of the drone origin when it sits at home (default %(default)s m)")
    ap.add_argument("--show", default="large,small", help="squares to draw: large, small or large,small")
    ap.add_argument("--rate", type=float, default=10.0, help="update rate in Hz (default %(default)s)")
    ap.add_argument("--targets", default="gui", help="gui (Gazebo window), sensors (cameras) or gui,sensors")
    ap.add_argument("--on", action="store_true", help="tell the running helper to show the --show squares")
    ap.add_argument("--off", "--hide", dest="off", action="store_true", help="tell the running helper to hide the squares")
    return ap.parse_args()


def services(targets):
    names = {"gui": "/marker_array", "sensors": "/sensors/marker_array"}
    return [names[t.strip()] for t in targets.split(",")]


def send(node, svcs, markers):
    req = Marker_V()
    req.marker.extend(markers)
    for svc in svcs:
        node.request(svc, req, Marker_V, Boolean, 1000)


def delete_all(node, svcs):
    m = Marker()
    m.ns = NS
    m.action = Marker.DELETE_ALL
    send(node, svcs, [m])


def bar(mid, x, y, z, yaw, sx, sy, sz, rgb):
    m = Marker()
    m.ns, m.id, m.action, m.type = NS, mid, Marker.ADD_MODIFY, Marker.BOX
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
    for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
        c.r, c.g, c.b, c.a = rgb[0], rgb[1], rgb[2], 1.0
    return m


def square_markers(first_id, cx, cy, cz, yaw, side, t, rgb):
    """Horizontal square (forward/left plane) and vertical square (left/up plane), centred on (cx, cy, cz)."""
    h = side / 2
    cy_, sy_ = math.cos(yaw), math.sin(yaw)
    world = lambda fx, ly: (cx + fx * cy_ - ly * sy_, cy + fx * sy_ + ly * cy_)   # body forward/left -> ENU
    out = []
    # horizontal: two bars along forward axis, two along left axis
    for i, (fx, ly, sx, sy) in enumerate([(0, h, side + t, t), (0, -h, side + t, t), (h, 0, t, side + t), (-h, 0, t, side + t)]):
        x, y = world(fx, ly)
        out.append(bar(first_id + i, x, y, cz, yaw, sx, sy, t, rgb))
    # vertical: in the lateral (left/up) plane through the centre
    for i, (ly, dz, sy, sz) in enumerate([(0, h, side + t, t), (0, -h, side + t, t), (h, 0, t, side + t), (-h, 0, t, side + t)]):
        x, y = world(0, ly)
        out.append(bar(first_id + 4 + i, x, y, cz + dz, yaw, t, sy, sz, rgb))
    return out


def write_control(value):
    os.makedirs(os.path.dirname(CONTROL), exist_ok=True)
    with open(CONTROL, "w") as f:
        f.write(value + "\n")


def read_control(default):
    try:
        with open(CONTROL) as f:
            return f.read().strip() or default
    except OSError:
        return default


def parse_show(text):
    show = [s.strip() for s in text.split(",") if s.strip()]
    for s in show:
        if s not in SQUARES:
            sys.exit(f"unknown square '{s}' (use large, small)")
    return show


def connect(a):
    """Connect to SITL, waiting as long as needed (SITL idles until a ground station connects on 5760)."""
    said = False
    while True:
        try:
            mav = mavutil.mavlink_connection(a.mavlink, retries=1)
            if mav.wait_heartbeat(timeout=5):
                mav.mav.command_long_send(mav.target_system, mav.target_component,
                                          mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                          mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                          1e6 / max(a.rate, 1.0), 0, 0, 0, 0, 0)
                print(f"connected to SITL on {a.mavlink}", flush=True)
                return mav
            mav.close()
        except Exception:
            pass
        if not said:
            print(f"waiting for SITL on {a.mavlink} ...", flush=True)
            said = True
        time.sleep(2)


def main():
    a = parse_args()
    node = Node()
    svcs = services(a.targets)
    parse_show(a.show)
    if a.off:
        write_control("off")
        delete_all(node, svcs)
        print("accuracy squares off")
        return
    if a.on:
        write_control(a.show)
        print(f"accuracy squares on: {a.show}")
        return

    lat0, lon0, alt0 = (float(v) for v in a.home.split(","))
    write_control(a.show)              # a new helper starts with the squares shown

    def stop(*_):
        delete_all(node, svcs)
        print("\naccuracy squares removed")
        sys.exit(0)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    mav = connect(a)
    period, last, last_msg, drawn = 1.0 / a.rate, 0.0, time.time(), None
    while True:
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out):   # pymavlink prints "EOF on TCP socket" on every read
                p = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        except Exception:
            p = None
        lost = "EOF" in out.getvalue()
        if p is None or lost:
            if lost or time.time() - last_msg > 5:  # SITL stopped or restarted
                delete_all(node, svcs)
                drawn = None
                with contextlib.suppress(Exception):
                    mav.close()
                print("lost SITL connection", flush=True)
                mav = connect(a)
                last_msg = time.time()
            continue
        last_msg = time.time()
        if time.time() - last < period:
            continue
        last = time.time()

        wanted = read_control(a.show)
        if wanted != drawn:                      # switched on/off or changed squares
            delete_all(node, svcs)
            drawn = wanted
            print(f"accuracy squares: {wanted}", flush=True)
        if wanted == "off":
            continue
        # inverse of SITL's mapping from Gazebo position to lat/lon (ArduPilot Location::offset)
        lat, lon = p.lat * 1e-7, p.lon * 1e-7
        north = (lat - lat0) * 1e7 * LATLON_TO_M
        east = (lon - lon0) * 1e7 * LATLON_TO_M * math.cos(math.radians((lat + lat0) / 2))
        # height above home: ArduPilot's absolute altitude carries the GPS altitude error from when its origin was set
        up = p.relative_alt * 1e-3 + a.home_height
        yaw = math.radians(90.0 - p.hdg / 100.0) if p.hdg != 65535 else math.pi / 2
        markers = []
        for name in parse_show(wanted):
            side, t, rgb, first_id = SQUARES[name]
            markers += square_markers(first_id, east, north, up, yaw, side, t, rgb)
        send(node, svcs, markers)


if __name__ == "__main__":
    main()
