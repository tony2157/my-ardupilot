#!/bin/bash
# RFSonde RadarSim: the radar service (RFSonde_NF2FF radarsim/) and the companion computer with the emulated
# bladeRF 2.0 micro + GPSDO (RFSonde_Computer, sdr.backend radarsim) for one SITL instance.
#
# Order: gz sim (the world's vehicle carries RFSondeTruthPlugin; GZ_SIM_SYSTEM_PLUGIN_PATH must include the
# ardupilot_gazebo main-RadarSim build), then rfsonde_sitl.sh, then this script. Ctrl+C stops both processes.
#
#   rfsonde_radarsim.sh [-I N] [--scenario FILE] [--beam AZ,EL] [--gain DB] [--out DIR] [--gui]
#
# --gui also opens the RadarSim GUI (rfsonde_radarsim.gui, needs dearpygui in RADARSIM_PY's venv): it changes the
# running radar (frequency, element, orientation, rotation, steering, element errors) and plots its pattern.
#
# Environment: RADARSIM_DIR (RFSonde_NF2FF/radarsim), RADARSIM_PY (its venv python), CC_DIR (RFSonde_Computer),
# CC_PY (its venv python).
set -u
instance=0
scenario=
beam=
gain=12
out=
gui=0
while [ $# -gt 0 ]; do
  case "$1" in
    -I) instance=$2; shift ;;
    -I[0-9]*) instance=${1#-I} ;;
    --scenario) scenario=$2; shift ;;
    --beam) beam=$2; shift ;;
    --gain) gain=$2; shift ;;
    --out) out=$2; shift ;;
    --gui) gui=1 ;;
    *) echo "unknown option $1"; exit 1 ;;
  esac
  shift
done
RADARSIM_DIR=${RADARSIM_DIR:-/mnt/c/Users/sega0000/Github/RFSonde_NF2FF/radarsim}
CC_DIR=${CC_DIR:-/mnt/c/Users/sega0000/Github/RFSonde_Computer}
RADARSIM_PY=${RADARSIM_PY:-$RADARSIM_DIR/venv/bin/python}
CC_PY=${CC_PY:-$CC_DIR/.venv/bin/python}
[ -x "$RADARSIM_PY" ] || RADARSIM_PY=python3
[ -x "$CC_PY" ] || CC_PY=python3
scenario=${scenario:-$RADARSIM_DIR/scenarios/atd_stare.yaml}
out=${out:-$HOME/.cache/rfsonde/radarsim_I$instance}
ring=/dev/shm/rfsonde_truth_I$instance
port=$((15970 + 10 * instance))
mkdir -p "$out/cc/data"

echo "RadarSim: waiting for the Gazebo truth ring $ring ..."
while [ ! -e "$ring" ]; do sleep 1; done

# scenario copy with the beam, and the companion configuration for this instance
"$RADARSIM_PY" - "$scenario" "$out" "$beam" <<'EOF' || exit 1
import math, sys, yaml
sys.path.insert(0, __import__("os").environ.get("RADARSIM_DIR", "/mnt/c/Users/sega0000/Github/RFSonde_NF2FF/radarsim"))
from rfsonde_radarsim.scenario import Scenario, load
c = load(sys.argv[1])
if sys.argv[3]:
    az, el = (float(v) for v in sys.argv[3].split(","))
    c["radar"]["beams"] = {0: {"az_deg": az, "el_deg": el}}
yaml.safe_dump(c, open(sys.argv[2] + "/scenario.yaml", "w"))
r = Scenario(c).radar
b = r.beams[min(r.beams)]
ae, ee = math.radians(b.az_deg), math.radians(b.el_deg)
u = math.cos(ee) * math.sin(ae) * r.axes[0] + math.sin(ee) * r.axes[1] + math.cos(ee) * math.cos(ae) * r.axes[2]
az = math.degrees(math.atan2(u[0], u[1])) % 360
el = math.degrees(math.asin(u[2]))
print(f"RadarSim: {r.name} beam az {az:.2f} el {el:.2f} deg -> MNT1_ARRC_AZTH {(az + 180) % 360:.2f}, MNT1_ARRC_ELEV {el:.2f}")
EOF
CC_DIR="$CC_DIR" "$CC_PY" - "$out" "$instance" "$port" "$gain" <<'EOF' || exit 1
import os, sys, yaml
out, inst, port, gain = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])
cc = os.environ["CC_DIR"]
c = yaml.safe_load(open(os.path.join(cc, "config", "rfsonde_cc_radarsim.yaml")))
c["paths"].update(data=os.path.join(out, "cc", "data"), profiles=os.path.join(out, "cc", "data", "profiles"),
                  waveforms=os.path.join(cc, "config", "radarsim_waveforms"), scripts=os.path.join(cc, "scripts"),
                  ports=os.path.join(cc, "config", "ports_radarsim.yaml"),
                  calibration=os.path.join(cc, "config", "cal", "default.yaml"))
c["link"]["url"] = f"tcp:127.0.0.1:{5762 + 10 * inst}"
c["files"]["port"] = 8077 + inst
c["radarsim"].update(instance=inst, pulse_port=port)
c["defaults"]["sdr"]["rx_gain_db"] = [gain, gain]
yaml.safe_dump(c, open(os.path.join(out, "cc.yaml"), "w"))
EOF

pids=
cleanup() {
  # background jobs of a script ignore SIGINT: stop them with SIGTERM (both close their files), then make sure
  for p in $pids; do kill -TERM "$p" 2>/dev/null; done
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    alive=
    for p in $pids; do kill -0 "$p" 2>/dev/null && alive=1; done
    [ -z "$alive" ] && break
    sleep 1
  done
  for p in $pids; do kill -KILL "$p" 2>/dev/null; done
  exit 0
}
trap cleanup INT TERM

(cd "$RADARSIM_DIR" && exec "$RADARSIM_PY" -m rfsonde_radarsim.service -c "$out/scenario.yaml" --instance "$instance" \
  --out "$out/truth_$(date +%Y%m%d_%H%M%S).h5") > "$out/radarsim.log" 2>&1 &
pids="$pids $!"
(cd "$CC_DIR" && RFSONDE_SIM_CLOCK="$ring" PYTHONPATH="$CC_DIR" exec "$CC_PY" -m rfsonde_cc -c "$out/cc.yaml") \
  > "$out/cc.log" 2>&1 &
pids="$pids $!"
if [ "$gui" = 1 ]; then
  (cd "$RADARSIM_DIR" && exec "$RADARSIM_PY" -m rfsonde_radarsim.gui --instance "$instance" -c "$out/scenario.yaml") \
    > "$out/gui.log" 2>&1 &
  pids="$pids $!"
fi
echo "RadarSim: radar service and companion running (logs and data in $out; companion on SITL tcp $((5762 + 10 * instance)))"
wait
