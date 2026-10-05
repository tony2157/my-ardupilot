#!/bin/bash
# Start RFSonde SITL for the norman_radars Gazebo world (start Gazebo first).
#
# Runs sim_vehicle.py with the RFSonde start location, gimbal and GPS parameters and no MAVProxy
# (connect your ground station to TCP 127.0.0.1:5760), and shows the GPS accuracy squares in the
# Gazebo window by default.
#
# Usage: rfsonde_sitl.sh [--gps PROFILE] [--rtk] [--no-squares] [other sim_vehicle.py options]
#   --gps PROFILE  GPS profile from RFSonde-sim/config: standard (default), sbas or rtk
#   --rtk          same as --gps rtk
#   --no-squares   do not start the accuracy-squares helper
#   RFSONDE_SQUARES_ARGS  extra options for accuracy_squares.py (environment variable)
#
# Switch the squares while running: accuracy_squares.py --off / --on (see RFSonde-sim/README.md).

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(dirname "$TOOLS")"
REPO="$(dirname "$SIM")"

gps=gps-standard.parm
squares=1
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
cleanup() {
  if [ -n "$helper_pid" ]; then
    kill "$helper_pid" 2>/dev/null    # the helper removes the squares when it stops
    wait "$helper_pid" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

if [ "$squares" = 1 ]; then
  log="$HOME/.cache/rfsonde/accuracy_squares.log"
  mkdir -p "$(dirname "$log")"
  # SITL serial 2 (TCP 5763 + 10 x instance) is free for the helper
  python3 "$TOOLS/accuracy_squares.py" --mavlink "tcp:127.0.0.1:$((5763 + 10 * instance))" \
    $RFSONDE_SQUARES_ARGS > "$log" 2>&1 &
  helper_pid=$!
  echo "RFSonde: accuracy squares on (log: $log). Hide: python3 $TOOLS/accuracy_squares.py --off"
fi

cd "$REPO" || exit 1
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -L RFSonde_Norman \
  --add-param-file="$SIM/config/rfsonde-gimbal.parm" \
  --add-param-file="$SIM/config/$gps" \
  --no-mavproxy "${extra[@]}"
