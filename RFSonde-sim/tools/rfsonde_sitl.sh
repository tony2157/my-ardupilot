#!/bin/bash
# Start RFSonde SITL for the norman_radars Gazebo world (start Gazebo first).
#
# Runs sim_vehicle.py with the RFSonde start location, gimbal and GPS parameters and no MAVProxy
# (connect your ground station to TCP 127.0.0.1:5760), and shows the GPS accuracy squares in the
# Gazebo window by default.
#
# The gimbal is the Gremsy T7 digital twin: the T7 firmware (RFSonde_Gremsy/T7_firmware_RE/src/c)
# running on simulated hardware, MAVLink on SERIAL2 like the real Cube.  The twin is rebuilt from
# the firmware sources at every start (only what changed is recompiled), so a firmware edit takes
# effect at the next launch.  Gazebo: gz sim -v4 -r norman_radars.sdf
#
# Usage: rfsonde_sitl.sh [--servo-gimbal] [--gps PROFILE] [--rtk] [--no-squares] [other sim_vehicle.py options]
#   --servo-gimbal the old 3-axis servo gimbal instead (Gazebo: norman_radars_servo.sdf)
#   --gps PROFILE  GPS profile from RFSonde-sim/config: standard (default), sbas or rtk
#   --rtk          same as --gps rtk
#   --no-squares   do not start the accuracy-squares helper
#   RFSONDE_SQUARES_ARGS  extra options for accuracy_squares.py (environment variable)
#   T7_TWIN_DIR    the T7_twin folder of RFSonde_Gremsy (built with its build.py)
#   RFSONDE_T7_ARGS  extra options for t7_twin, e.g. --console (environment variable)
#
# Switch the squares while running: accuracy_squares.py --off / --on (see RFSonde-sim/README.md).

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(dirname "$TOOLS")"
REPO="$(dirname "$SIM")"

gps=gps-standard.parm
squares=1
t7=1
instance=0
extra=()
prev=
for arg in "$@"; do
  if [ "$prev" = "--gps" ]; then
    gps=gps-$arg.parm; prev=; continue
  fi
  case "$arg" in
    --rtk) gps=gps-rtk.parm ;;
    --gps) ;;
    --gps=*) gps=gps-${arg#--gps=}.parm ;;
    --no-squares) squares=0 ;;
    --servo-gimbal) t7=0 ;;
    *) extra+=("$arg") ;;
  esac
  case "$prev" in -I|--instance) instance=$arg ;; esac
  case "$arg" in -I[0-9]*) instance=${arg#-I} ;; --instance=*) instance=${arg#--instance=} ;; esac
  prev=$arg
done

if [ ! -f "$SIM/config/$gps" ]; then
  echo "unknown GPS profile: $SIM/config/$gps (use standard, sbas or rtk)"; exit 1
fi

helper_pid=
twin_pid=
cleanup() {
  if [ -n "$helper_pid" ]; then
    kill "$helper_pid" 2>/dev/null    # the helper removes the squares when it stops
    wait "$helper_pid" 2>/dev/null
  fi
  if [ -n "$twin_pid" ]; then
    kill "$twin_pid" 2>/dev/null
    wait "$twin_pid" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

gimbal_parm=rfsonde-gimbal.parm
helper_port=$((5763 + 10 * instance))   # SITL serial 2 is free for the helper
if [ "$t7" = 1 ]; then
  gimbal_parm=rfsonde-t7.parm
  helper_port=$((5765 + 10 * instance)) # serial 2 is the gimbal; serial 5 (SERIAL5_PROTOCOL 2)
  twin_dir="${T7_TWIN_DIR:-/mnt/c/Users/sega0000/Github/RFSonde_Gremsy/T7_twin}"
  twin="$twin_dir/build/t7_twin"
  if [ ! -f "$twin_dir/build.py" ]; then
    echo "T7 twin not found in $twin_dir (set T7_TWIN_DIR to RFSonde_Gremsy/T7_twin)"; exit 1
  fi
  # rebuild from the firmware sources: an edited .c/.h is recompiled, nothing else
  echo "RFSonde: building the T7 twin from $twin_dir/../T7_firmware_RE/src/c ..."
  python3 "$twin_dir/build.py" || { echo "RFSonde: T7 twin build failed"; exit 1; }
  cache="$HOME/.cache/rfsonde"
  mkdir -p "$cache"
  eeprom="$cache/t7_eeprom_I$instance.bin"
  if [ ! -f "$eeprom" ]; then
    # a new gimbal board: first-boot calibration, Gremsy's recommended profile, gyro calibration
    echo "RFSonde: preparing the T7 twin's EEPROM (first boot, about 3 minutes) ..."
    python3 "$twin_dir/tools/t7tool.py" factory "$eeprom" || exit 1
  fi
  if [ "$instance" != 0 ]; then
    echo "RFSonde: note: models/rfsonde_t7 talks to the twin on UDP 9150; edit its twin_port for instance $instance"
  fi
  "$twin" --gz --gz-port $((9150 + 10 * instance)) --eeprom "$eeprom" \
    --com1 "tcp:$((5800 + 10 * instance))" --com2 "tcpclient:127.0.0.1:$((5763 + 10 * instance))" \
    --log "$cache/t7_twin_I$instance.csv" $RFSONDE_T7_ARGS > "$cache/t7_twin_I$instance.log" 2>&1 &
  twin_pid=$!
  echo "RFSonde: T7 twin running (log: $cache/t7_twin_I$instance.log, gTune port TCP $((5800 + 10 * instance)))"
fi

if [ "$squares" = 1 ]; then
  log="$HOME/.cache/rfsonde/accuracy_squares.log"
  mkdir -p "$(dirname "$log")"
  python3 "$TOOLS/accuracy_squares.py" --mavlink "tcp:127.0.0.1:$helper_port" \
    $RFSONDE_SQUARES_ARGS > "$log" 2>&1 &
  helper_pid=$!
  echo "RFSonde: accuracy squares on (log: $log). Hide: python3 $TOOLS/accuracy_squares.py --off"
fi

cd "$REPO" || exit 1
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -L RFSonde_Norman \
  --add-param-file="$SIM/config/$gimbal_parm" \
  --add-param-file="$SIM/config/$gps" \
  --no-mavproxy "${extra[@]}"
