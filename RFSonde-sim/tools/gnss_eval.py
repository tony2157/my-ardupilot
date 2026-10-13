#!/usr/bin/env python3
"""
Evaluate GNSS and EKF3 position accuracy from a SITL dataflash log (.BIN).

Compares, against the simulator's true position (SIM / SIM2 messages):
  - raw GPS (GPS / GPA): error statistics, reported vs actual accuracy, fix type, correlation time
  - EKF3 estimate (POS): error statistics, how closely it follows the GPS error, noise reduction
  - EKF3 health (XKF4): innovation test ratios for GPS velocity (SV), position (SP) and height (SH)
Statistics are reported for the whole airborne period, for hovering and for moving.

Usage:
  python3 gnss_eval.py LOG.BIN [--skip 30] [--csv errors.csv]
  python3 gnss_eval.py --latest [--log-dir ~/Github/my-ardupilot/logs]
"""
import argparse
import bisect
import glob
import math
import os
import statistics
import sys

from pymavlink import mavutil

M_PER_DEG = 0.011131884502145034 * 1e7      # ArduPilot spherical-earth metres per degree of latitude
GPS_FIX = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
PROFILE_PARAMS = ("SIM_GPS_HBIAS", "SIM_GPS_HWN", "SIM_GPS_VBIAS", "SIM_GPS_VWN", "SIM_GPS_BTAU",
                  "SIM_GPS_VELNSE", "SIM_GPS1_LAG_MS", "SIM_GPS1_HZ", "EK3_SRC1_POSZ", "EK3_POSNE_M_NSE",
                  "EK3_VELNE_M_NSE")


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?", help="dataflash log (.BIN)")
    ap.add_argument("--latest", action="store_true", help="use the newest .BIN in --log-dir")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/Github/my-ardupilot/logs"))
    ap.add_argument("--skip", type=float, default=20.0, help="seconds to ignore after takeoff (default %(default)s)")
    ap.add_argument("--csv", help="write the time-aligned errors to this CSV file")
    ap.add_argument("--all", action="store_true",
                    help="include samples on the ground (e.g. a long stationary run to check the GNSS model)")
    a = ap.parse_args()
    if a.latest:
        logs = sorted(glob.glob(os.path.join(a.log_dir, "*.BIN")), key=os.path.getmtime)
        if not logs:
            sys.exit(f"no .BIN logs in {a.log_dir}")
        a.log = logs[-1]
    if not a.log:
        ap.error("give a log file or --latest")
    return a


def read_log(path):
    m = mavutil.mavlink_connection(path)
    d = {"SIM": [], "SIM2": [], "GPS": [], "GPA": [], "POS": [], "XKF4": [], "PARM": {}}
    while True:
        msg = m.recv_match(type=["SIM", "SIM2", "GPS", "GPA", "POS", "XKF4", "PARM"], blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == "PARM":
            d["PARM"][msg.Name] = msg.Value
        elif t == "GPS" and getattr(msg, "I", 0) != 0:
            continue
        elif t == "GPA" and getattr(msg, "I", 0) != 0:
            continue
        elif t == "XKF4" and getattr(msg, "C", 0) != 0:
            continue
        else:
            d[t].append(msg)
    return d


class Truth:
    """Time-indexed true position (SIM: lat/lon/alt) and velocity / height above home (SIM2)."""

    def __init__(self, sim, sim2):
        self.t = [m.TimeUS * 1e-6 for m in sim]
        self.sim = sim
        self.t2 = [m.TimeUS * 1e-6 for m in sim2]
        self.sim2 = sim2

    @staticmethod
    def _interp(ts, vals, t):
        i = bisect.bisect_left(ts, t)
        if i <= 0 or i >= len(ts):
            return None
        f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        return [a + f * (b - a) for a, b in zip(vals(i - 1), vals(i))]

    def pos(self, t):
        return self._interp(self.t, lambda i: (self.sim[i].Lat, self.sim[i].Lng, self.sim[i].Alt), t)

    def state(self, t):
        return self._interp(self.t2, lambda i: (self.sim2[i].PD, self.sim2[i].VN, self.sim2[i].VE), t)


def ned_error(lat, lon, alt, ref):
    n = (lat - ref[0]) * M_PER_DEG
    e = (lon - ref[1]) * M_PER_DEG * math.cos(math.radians(ref[0]))
    return n, e, -(alt - ref[2])


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float("nan")


def pct(v, p):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))]


def corr_time(series, dt):
    """Time for the autocorrelation of a series to fall below 1/e (rough estimate)."""
    if len(series) < 20:
        return float("nan")
    mu = statistics.mean(series)
    x = [v - mu for v in series]
    var = sum(v * v for v in x)
    if var <= 0:
        return float("nan")
    for lag in range(1, len(x) // 2):
        r = sum(x[i] * x[i + lag] for i in range(len(x) - lag)) / var
        if r < 1 / math.e:
            return lag * dt
    return float("inf")


def high_freq(series, n=5):
    """Series minus its centred moving average: the fast (noise-like) part."""
    out = []
    for i in range(n, len(series) - n):
        out.append(series[i] - statistics.mean(series[i - n:i + n + 1]))
    return out


def correlation(a, b):
    if len(a) < 3:
        return float("nan")
    ma, mb = statistics.mean(a), statistics.mean(b)
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    den = math.sqrt(sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b))
    return num / den if den > 0 else float("nan")


def best_shift(samples, truth, shifts, fix_ok=lambda s: True):
    """Time shift (s) of the truth that minimises the horizontal error while moving (> 1 m/s, airborne)."""
    best = None
    for sh in shifts:
        err = []
        for s in samples:
            if not fix_ok(s):
                continue
            t = s.TimeUS * 1e-6
            st = truth.state(t)
            if st is None or -st[0] < 2.0 or math.hypot(st[1], st[2]) < 1.0:
                continue
            ref = truth.pos(t - sh)
            if ref is None:
                continue
            n, e, _ = ned_error(s.Lat, s.Lng, s.Alt, ref)
            err.append(math.hypot(n, e))
        if len(err) < 30:
            return None
        r = rms(err)
        if best is None or r < best[1]:
            best = (sh, r)
    return best


def summarise(label, rows, out):
    if not rows:
        out.append(f"\n{label}: no samples")
        return
    gn = [r["gn"] for r in rows]; ge = [r["ge"] for r in rows]; gd = [r["gd"] for r in rows]
    kn = [r["kn"] for r in rows]; ke = [r["ke"] for r in rows]; kd = [r["kd"] for r in rows]
    gh = [math.hypot(a, b) for a, b in zip(gn, ge)]
    kh = [math.hypot(a, b) for a, b in zip(kn, ke)]
    out.append(f"\n{label} ({len(rows)} GPS samples, {rows[-1]['t'] - rows[0]['t']:.0f} s)")
    out.append(f"  {'':22s}{'GPS raw':>12s}{'EKF3':>12s}")
    for name, g, k in [("north sigma (m)", gn, kn), ("east sigma (m)", ge, ke), ("down sigma (m)", gd, kd)]:
        out.append(f"  {name:22s}{statistics.pstdev(g):12.3f}{statistics.pstdev(k):12.3f}")
    for name, g, k in [("north mean (m)", gn, kn), ("east mean (m)", ge, ke), ("down mean (m)", gd, kd)]:
        out.append(f"  {name:22s}{statistics.mean(g):12.3f}{statistics.mean(k):12.3f}")
    out.append(f"  {'horizontal RMS (m)':22s}{rms(gh):12.3f}{rms(kh):12.3f}")
    out.append(f"  {'CEP50 (m)':22s}{pct(gh, 50):12.3f}{pct(kh, 50):12.3f}")
    out.append(f"  {'CEP95 (m)':22s}{pct(gh, 95):12.3f}{pct(kh, 95):12.3f}")
    out.append(f"  {'vertical RMS (m)':22s}{rms(gd):12.3f}{rms(kd):12.3f}")
    hf_g = high_freq(gn) + high_freq(ge)
    hf_k = high_freq(kn) + high_freq(ke)
    out.append(f"  {'fast noise RMS (m)':22s}{rms(hf_g):12.3f}{rms(hf_k):12.3f}   (part faster than ~2 s)")
    out.append(f"  EKF follows the GPS error: correlation north {correlation(gn, kn):.2f}, east {correlation(ge, ke):.2f}, "
               f"down {correlation(gd, kd):.2f}")


def main():
    a = parse_args()
    d = read_log(a.log)
    p = d["PARM"]
    # ArduPilot 4.7 moved the per-receiver SITL GPS parameters to SIM_GPS1_*; also accept 4.6 logs
    for name in ("LAG_MS", "HZ"):
        if "SIM_GPS_" + name in p:
            p.setdefault("SIM_GPS1_" + name, p["SIM_GPS_" + name])
    if not d["SIM"] or not d["GPS"] or not d["POS"]:
        sys.exit("log lacks SIM, GPS or POS messages (is it a SITL log of a flight?)")
    truth = Truth(d["SIM"], d["SIM2"])
    # GPS samples are delayed: use the delay measured while moving, else SIM_GPS1_LAG_MS
    lag_cfg = p.get("SIM_GPS1_LAG_MS", 100) * 1e-3
    gps_fit = best_shift(d["GPS"], truth, [i * 0.01 for i in range(0, 31)], lambda g: g.Status >= 3)
    lag = gps_fit[0] if gps_fit else lag_cfg
    ekf_fit = best_shift(d["POS"], truth, [i * 0.01 for i in range(-10, 11)])

    # EKF output (POS) is time-aligned with the truth; GPS samples are delayed by SIM_GPS1_LAG_MS
    pos_t = [m.TimeUS * 1e-6 for m in d["POS"]]
    gpa_t = [m.TimeUS * 1e-6 for m in d["GPA"]]
    rows, fixes, takeoff = [], {}, None
    for g in d["GPS"]:
        t = g.TimeUS * 1e-6
        if g.Status < 3:
            continue
        st = truth.state(t)
        if st is None:
            continue
        airborne = -st[0] > 2.0
        if not airborne and not a.all:
            continue
        takeoff = takeoff if takeoff is not None else t
        if t - takeoff < a.skip:
            continue
        ref_g, ref_k = truth.pos(t - lag), truth.pos(t)
        i = bisect.bisect_left(pos_t, t)
        if ref_g is None or ref_k is None or i >= len(pos_t):
            continue
        k = d["POS"][i]
        gn, ge, gd = ned_error(g.Lat, g.Lng, g.Alt, ref_g)
        kn, ke, kd = ned_error(k.Lat, k.Lng, k.Alt, ref_k)
        j = min(bisect.bisect_left(gpa_t, t), len(gpa_t) - 1)
        acc = d["GPA"][j] if d["GPA"] else None
        rows.append(dict(t=t, gn=gn, ge=ge, gd=gd, kn=kn, ke=ke, kd=kd, speed=math.hypot(st[1], st[2]),
                         hacc=acc.HAcc if acc else float("nan"), vacc=acc.VAcc if acc else float("nan"),
                         sacc=acc.SAcc if acc else float("nan"), fix=g.Status))
        fixes[g.Status] = fixes.get(g.Status, 0) + 1
    if not rows:
        sys.exit("no airborne GPS samples found (the log needs a flight above 2 m, or use --all)")

    out = [f"GNSS / EKF3 evaluation: {a.log}"]
    out.append("profile: " + ", ".join(f"{n}={p[n]:g}" for n in PROFILE_PARAMS if n in p))
    hb, hw, vb, vw = (p.get(n, 0) for n in ("SIM_GPS_HBIAS", "SIM_GPS_HWN", "SIM_GPS_VBIAS", "SIM_GPS_VWN"))
    if hb or hw or vb or vw:
        out.append(f"configured 1-sigma: horizontal {math.hypot(hb, hw):.3f} m per axis, vertical {math.hypot(vb, vw):.3f} m")
    out.append("GPS fix: " + ", ".join(f"{GPS_FIX.get(k, k)} {100 * v / len(rows):.0f}%" for k, v in sorted(fixes.items())))
    out.append(f"GPS compared with the truth {lag * 1000:.0f} ms earlier ("
               + ("measured while moving" if gps_fit else "SIM_GPS1_LAG_MS") + f"); first {a.skip:.0f} s "
               + ("of GPS fix" if a.all else "after takeoff") + " skipped")

    summarise("Whole flight", rows, out)
    summarise("Hovering (speed < 0.5 m/s)", [r for r in rows if r["speed"] < 0.5], out)
    summarise("Moving (speed > 1 m/s)", [r for r in rows if r["speed"] > 1.0], out)

    # reported vs actual accuracy
    gn = [r["gn"] for r in rows]; ge = [r["ge"] for r in rows]; gd = [r["gd"] for r in rows]
    act_h = math.sqrt((statistics.pvariance(gn) + statistics.pvariance(ge)) / 2)
    out.append("\nReported vs actual GPS accuracy (1-sigma)")
    out.append(f"  horizontal per axis: reported {statistics.mean(r['hacc'] for r in rows):.3f} m, actual {act_h:.3f} m")
    out.append(f"  vertical:            reported {statistics.mean(r['vacc'] for r in rows):.3f} m, actual {statistics.pstdev(gd):.3f} m")
    out.append(f"  speed accuracy:      reported {statistics.mean(r['sacc'] for r in rows):.3f} m/s")
    dt = statistics.median(b["t"] - a_["t"] for a_, b in zip(rows, rows[1:]))
    tau = (corr_time(gn, dt) + corr_time(ge, dt)) / 2
    out.append(f"  GPS error correlation time: about {tau:.0f} s"
               + (f" (configured SIM_GPS_BTAU {p['SIM_GPS_BTAU']:g} s; needs a log several times longer to be reliable)"
                  if "SIM_GPS_BTAU" in p else ""))

    if ekf_fit:
        out.append(f"\nTiming (from the moving part of the flight)")
        out.append(f"  GPS delay: {lag * 1000:.0f} ms (SIM_GPS1_LAG_MS {lag_cfg * 1000:.0f} ms)")
        out.append(f"  EKF3 output lag behind the truth: {ekf_fit[0] * 1000:.0f} ms; moving error would be "
                   f"{ekf_fit[1]:.3f} m RMS without it (latency adds about speed x lag)")
    if rows and statistics.mean(r["hacc"] for r in rows) < 0.01:
        out.append("  note: the log stores reported accuracy (GPA) in whole centimetres")

    # EKF3 innovation test ratios (1.0 = measurement rejected)
    xk = [m for m in d["XKF4"] if takeoff is not None and m.TimeUS * 1e-6 >= takeoff + a.skip]
    if xk:
        out.append("\nEKF3 innovation test ratios (0 = perfect agreement, 1 = GPS rejected)")
        for name, field in (("GPS velocity", "SV"), ("GPS position", "SP"), ("height", "SH")):
            v = [getattr(m, field) for m in xk]
            out.append(f"  {name:13s} mean {statistics.mean(v):.2f}  95% {pct(v, 95):.2f}  max {max(v):.2f}  "
                       f"rejected {100 * sum(x > 1 for x in v) / len(v):.1f}%")

    report = "\n".join(out)
    print(report)
    with open(os.path.splitext(a.log)[0] + "_gnss_report.txt", "w") as f:
        f.write(report + "\n")
    if a.csv:
        with open(a.csv, "w") as f:
            f.write("time_s,speed_mps,fix,gps_n,gps_e,gps_d,ekf_n,ekf_e,ekf_d,hacc,vacc,sacc\n")
            for r in rows:
                f.write(f"{r['t']:.3f},{r['speed']:.3f},{r['fix']},{r['gn']:.4f},{r['ge']:.4f},{r['gd']:.4f},"
                        f"{r['kn']:.4f},{r['ke']:.4f},{r['kd']:.4f},{r['hacc']:.4f},{r['vacc']:.4f},{r['sacc']:.4f}\n")
        print(f"\nerrors written to {a.csv}")


if __name__ == "__main__":
    main()
