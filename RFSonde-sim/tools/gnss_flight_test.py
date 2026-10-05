#!/usr/bin/env python3
"""
Automated GNSS test flight for SITL, followed by gnss_eval.py on the new log.

Flight: (optionally load a GPS profile) -> GUIDED takeoff -> hover -> square box pattern -> land.
Uses a spare SITL MAVLink port, so a ground station on TCP 5760 can stay connected.

Usage (SITL running, e.g. started with rfsonde_sitl.sh):
  python3 gnss_flight_test.py --profile rtk
  python3 gnss_flight_test.py --profile standard --hover 300 --box 30 --alt 15
Profiles are the files in RFSonde-sim/config: standard, sbas, rtk (or a path to any .parm file).
"""
import argparse
import glob
import math
import os
import subprocess
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(os.path.dirname(HERE), "config")
M_PER_DEG = 0.011131884502145034 * 1e7


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5762", help="SITL connection (default %(default)s)")
    ap.add_argument("--profile", help="GPS profile to load first: standard, sbas, rtk or a .parm path")
    ap.add_argument("--alt", type=float, default=10.0, help="flight height above home, m (default %(default)s)")
    ap.add_argument("--hover", type=float, default=120.0, help="hover time, s (default %(default)s)")
    ap.add_argument("--box", type=float, default=20.0, help="box side length, m; 0 skips the box (default %(default)s)")
    ap.add_argument("--laps", type=int, default=2, help="box laps (default %(default)s)")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/Github/my-ardupilot/logs"),
                    help="where SITL writes its .BIN logs (default %(default)s)")
    ap.add_argument("--no-eval", action="store_true", help="do not run gnss_eval.py afterwards")
    return ap.parse_args()


def say(text):
    print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)


def load_profile(m, profile):
    path = profile if profile.endswith(".parm") else os.path.join(CONFIG, f"gps-{profile}.parm")
    if not os.path.exists(path):
        sys.exit(f"profile not found: {path}")
    count = 0
    for line in open(path):
        line = line.split("#")[0].strip()
        if not line:
            continue
        name, value = line.replace(",", " ").split()[:2]
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        count += 1
        time.sleep(0.05)
    say(f"loaded {count} parameters from {os.path.basename(path)}")
    time.sleep(3)


def position(m):
    p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    return (p.lat * 1e-7, p.lon * 1e-7, p.relative_alt * 1e-3) if p else None


def goto(m, lat, lon, alt, timeout=60):
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000, int(lat * 1e7), int(lon * 1e7), alt, 0, 0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = position(m)
        if p and math.hypot((p[0] - lat) * M_PER_DEG, (p[1] - lon) * M_PER_DEG * math.cos(math.radians(lat))) < 1.0:
            return True
    return False


def main():
    a = parse_args()
    start = time.time()
    m = mavutil.mavlink_connection(a.mavlink, retries=30)
    say(f"waiting for SITL heartbeat on {a.mavlink} ...")
    m.wait_heartbeat(timeout=120)
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    if a.profile:
        load_profile(m, a.profile)

    say("arming (the EKF can take ~30 s to accept GPS after start-up)")
    t0 = time.time()
    while True:
        m.set_mode("GUIDED")
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
            break
        if time.time() - t0 > 180:
            sys.exit("could not arm within 180 s")
        time.sleep(2)
    say(f"armed; taking off to {a.alt:g} m")
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0, 0, 0, 0, 0, 0, 0, a.alt)
    t0 = time.time()
    while time.time() - t0 < 60:
        p = position(m)
        if p and p[2] > a.alt - 0.5:
            break
    home = position(m)
    say(f"hovering for {a.hover:g} s")
    goto(m, home[0], home[1], a.alt, timeout=5)
    time.sleep(a.hover)

    if a.box > 0:
        s = a.box
        dlat = s / M_PER_DEG
        dlon = s / (M_PER_DEG * math.cos(math.radians(home[0])))
        corners = [(home[0] + dlat, home[1]), (home[0] + dlat, home[1] + dlon), (home[0], home[1] + dlon), home[:2]]
        for lap in range(a.laps):
            say(f"box lap {lap + 1}/{a.laps} ({s:g} m sides)")
            for lat, lon in corners:
                goto(m, lat, lon, a.alt)

    say("landing")
    m.set_mode("LAND")
    t0 = time.time()
    while time.time() - t0 < 120:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.get_srcSystem() == m.target_system and not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            break
    say("landed and disarmed")

    if a.no_eval:
        return
    time.sleep(3)
    logs = [f for f in glob.glob(os.path.join(a.log_dir, "*.BIN")) if os.path.getmtime(f) > start]
    if not logs:
        say(f"no new log in {a.log_dir}; run gnss_eval.py on the flight's .BIN yourself")
        return
    log = max(logs, key=os.path.getmtime)
    say(f"evaluating {log}")
    subprocess.run([sys.executable, os.path.join(HERE, "gnss_eval.py"), log])


if __name__ == "__main__":
    main()
