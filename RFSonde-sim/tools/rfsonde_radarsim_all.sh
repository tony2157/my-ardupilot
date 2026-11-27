#!/bin/bash
# RFSonde RadarSim in one command: Gazebo (norman_radars world with the RadarSim plugins), SITL with the T7 twin
# (rfsonde_sitl.sh with config/radarsim.parm, the RTK GPS profile and no accuracy squares by default), then the
# radar service, the companion with its emulated SDR and the radar GUI (rfsonde_radarsim.sh). Instance 0 only: the
# world's vehicle talks to SITL and to the T7 twin on instance 0's ports.
#
#   rfsonde_radarsim_all.sh [--beam AZ,EL] [--gain DB] [--scenario FILE] [--no-gui] [--gps PROFILE] [--squares]
#                           [--headless] [--world FILE] [--stop-leftovers]
#                           [other rfsonde_sitl.sh / sim_vehicle.py options, e.g. -N]
#   rfsonde_radarsim_all.sh --stop        stop a running RadarSim from any terminal
#
#   --beam AZ,EL     electronic steering of the parked beam off broadside [deg], the scenario's beams.az_deg/el_deg
#                    (+az toward the face's +x: to the left seen from behind the face). Default: the scenario's
#                    beam (atd_stare.yaml: 0,0, broadside). The launcher prints the beam's earth azimuth/elevation
#                    and the MNT1_ARRC_AZTH / MNT1_ARRC_ELEV that point the probe at it.
#   --gain DB        receive gain of the companion's emulated SDR (default 12)
#   --scenario FILE  radar scenario (default RFSonde_NF2FF/radarsim/scenarios/atd_stare.yaml)
#   --no-gui         no radar GUI (start it later: RADARSIM.md, "The radar GUI"; closing that one stops nothing)
#   --gps PROFILE    GPS profile: standard, sbas or rtk (default rtk)
#   --squares        start the GPS accuracy squares (off by default)
#   --headless       Gazebo without its window (gz sim -s)
#   --world FILE     Gazebo world (default norman_radars.sdf)
#   --stop-leftovers first stop RadarSim processes that no launcher or terminal owns any more (they hold the ports
#                    a new run needs), e.g. from a run before the launcher recorded its processes
#
# Then connect Ludifly to TCP 127.0.0.1:5760: SITL starts, and links to Gazebo, once a ground station connects.
#
# Stopping: Ctrl+C here, closing this terminal, closing the Gazebo window, the radar GUI or the ArduCopter window,
# the T7 twin or another part ending, or --stop from another terminal. The launcher then stops every part it started:
# the radar and the companion first (they close their files), then SITL, the T7 twin and Gazebo; it waits up to
# 10 s and kills what is left. Further Ctrl+C presses do not interrupt that. Each run records its processes in
# ~/.cache/rfsonde/radarsim_I0/run.pids: the next start stops whatever a run left behind (a launcher killed with -9),
# and nothing starts while another program holds a port the stack needs or another ArduPilot SITL runs (SITL on
# instance 0 would kill it): it is named, never stopped.
# Logs in ~/.cache/rfsonde/radarsim_I0: gz.log, sitl.log, launcher.log, stop.log (and radarsim.log, cc.log, gui.log),
# the T7 twin's in ~/.cache/rfsonde/t7_twin_I0.log.
# Environment: GZ_ENV (Gazebo environment script, default ~/Github/ardupilot_gazebo/env.sh, sourced when GZ_VERSION
# is unset), RADARSIM_GZ_BUILD (RadarSim plugins, default ~/Github/ardupilot_gazebo-RadarSim/build), and those of
# rfsonde_sitl.sh and rfsonde_radarsim.sh.
set -u

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(dirname "$TOOLS")"

gps=rtk
squares=0
gui=1
headless=0
world=norman_radars.sdf
mode=start
stop_leftovers=0
radar_args=()
sitl_args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --beam|--gain|--scenario|--gps|--world)
      [ $# -ge 2 ] || { echo "$1 needs a value"; exit 1; }
      case "$1" in
        --gps) gps=$2 ;;
        --world) world=$2 ;;
        *) radar_args+=("$1" "$2") ;;
      esac
      shift ;;
    --gps=*) gps=${1#--gps=} ;;
    --rtk) gps=rtk ;;
    --no-gui) gui=0 ;;
    --squares) squares=1 ;;
    --no-squares) squares=0 ;;
    --headless) headless=1 ;;
    --stop) mode=stop ;;
    --stop-leftovers) stop_leftovers=1 ;;
    -I|-I[0-9]*|--instance|--instance=*)
      echo "rfsonde_radarsim_all.sh runs instance 0 only (the world's ports); use the three scripts for others"
      exit 1 ;;
    -h|--help) sed -n '2,/^set -u/{/^set -u/d;s/^# \{0,1\}//;p}' "$0"; exit 0 ;;
    *) sitl_args+=("$1") ;;
  esac
  shift
done

log=$HOME/.cache/rfsonde/radarsim_I0
ring=/dev/shm/rfsonde_truth_I0
pidfile=$log/run.pids
mkdir -p "$log"

# The ports the stack uses: SITL serial 0-5, the twin's gTune port and the companion's file service (TCP); the
# Gazebo<->SITL and Gazebo<->twin links and the radar's pulse and control links (UDP)
TCP_PORTS="5760 5762 5763 5765 5800 8077"
UDP_PORTS="9002 9003 9150 15970 15975"
# RadarSim parts, matched on a process's own command line, program first (an editor or a pager showing one of
# these files never matches): to confirm a recorded group before it is signalled, and to find leftovers
GZ='^(ruby )?[^ ]*gz sim |^[^ ]*gz-sim-main '
PARTS='^(/usr)?/bin/bash [^ ]*rfsonde_(sitl|radarsim)\.sh( |$)'
PARTS+='|^python3? [^ ]*sim_vehicle\.py .*RFSonde_Norman'
PARTS+='|^xterm .* -T ArduCopter .*RFSonde-sim/config/'
PARTS+='|^[^ ]*/arducopter .*RFSonde-sim/config/'
PARTS+='|^python3? [^ ]*/accuracy_squares\.py'
PARTS+='|^[^ ]*/t7_twin --gz'
PARTS+='|^[^ ]*python[0-9.]* -m (rfsonde_radarsim\.(service|gui)|rfsonde_cc -c [^ ]*/radarsim_I0/)'
PARTS+='|^(ruby )?[^ ]*gz sim .*norman_radars'
PARTS+='|^[^ ]*gz-sim-main .*norman_radars'

# ---------------------------------------------------------------- process bookkeeping
# A process is identified by its PID and its start time (/proc/<pid>/stat field 22), so that a PID reused by an
# unrelated program is never signalled. A zombie (ended, not yet collected by its parent) counts as gone.
start_of() { [ -r "/proc/$1/stat" ] && cut -d')' -f2- "/proc/$1/stat" 2>/dev/null | awk '$1 != "Z" {print $20}'; }
same_process() { [ -n "$2" ] && [ "$(start_of "$1")" = "$2" ]; }
# true while any process of process group $1 is a RadarSim part
group_is_ours() {
  ps -eo pgid=,args= | PARTS="$PARTS" awk -v g="$1" '
    $1 == g {a = $0; sub(/^ *[0-9]+ /, "", a); if (a ~ ENVIRON["PARTS"]) f = 1} END {exit !f}'
}
group_alive() { ps -eo pgid=,stat= | awk -v g="$1" '$1 == g && $2 !~ /^Z/ {f = 1} END {exit !f}'; }

# Stop process groups and single processes: SIGTERM, up to $wait s, then SIGKILL. Arguments: wait, then
# "g:PGID" / "p:PID:START" items. Signalled are the groups that hold a RadarSim part and the processes that are still
# the same ones when the stop starts; a group whose last parts ended first still gets its SIGKILL.
item_alive() {
  local pid st
  case "$1" in
    g:*) group_alive "${1#g:}" ;;
    p:*) IFS=: read -r _ pid st <<< "$1"; same_process "$pid" "$st" ;;
  esac
}
signal_item() {
  local pid st
  case "$2" in
    g:*) kill "-$1" -- "-${2#g:}" 2>/dev/null ;;
    p:*) IFS=: read -r _ pid st <<< "$2"; same_process "$pid" "$st" && kill "-$1" "$pid" 2>/dev/null ;;
  esac
}
stop_items() {
  local wait=$1; shift
  local it alive mine=()
  for it in "$@"; do
    case "$it" in
      g:*) group_is_ours "${it#g:}" && mine+=("$it") ;;
      p:*) item_alive "$it" && mine+=("$it") ;;
    esac
  done
  [ "${#mine[@]}" -gt 0 ] || return 0
  for it in "${mine[@]}"; do signal_item TERM "$it"; done
  for _ in $(seq $((wait * 2))); do
    alive=0
    for it in "${mine[@]}"; do item_alive "$it" && alive=1; done
    [ "$alive" = 0 ] && return 0
    sleep 0.5
  done
  for it in "${mine[@]}"; do item_alive "$it" && signal_item KILL "$it"; done
  return 1
}

# Items recorded in a run.pids file (all lines but the launcher's)
recorded_items() {
  awk 'NR > 1 {if ($1 == "group") print "g:" $3; else print "p:" $3 ":" $4}' "$pidfile" 2>/dev/null
}

# What a run left behind: stop it if its launcher is gone, refuse if it still runs
handle_previous_run() {
  [ -f "$pidfile" ] || return 0
  local lpid lstart
  read -r _ _ lpid lstart < "$pidfile"
  if same_process "$lpid" "$lstart"; then
    echo "RadarSim is already running (launcher PID $lpid): stop it with Ctrl+C there or: $0 --stop"
    exit 1
  fi
  mapfile -t items < <(recorded_items)
  if [ "${#items[@]}" -gt 0 ]; then
    echo "RadarSim: stopping what the previous run left behind ..."
    stop_items 10 "${items[@]}" || echo "RadarSim: some needed SIGKILL"
  fi
  rm -f "$pidfile"
}

# Process groups holding RadarSim parts that nobody owns any more: the group's leader is gone or was adopted by
# init (PID 1, or WSL's /init), as after a launcher killed with -9 or a run from before run.pids existed. A
# RadarSim started by hand in a terminal is not one of them.
leftover_groups() {
  ps -eo pid=,ppid=,pgid=,args= | PARTS="$PARTS" awk -v mypg="$(ps -o pgid= -p $$ | tr -d ' ')" '
    { par[$1] = $2; a = $0; sub(/^ *[0-9]+ +[0-9]+ +[0-9]+ /, "", a)
      if (a == "/init") init[$1] = 1
      if ($3 != mypg && a ~ ENVIRON["PARTS"]) cand[$3] = 1 }
    END { for (g in cand) if (!(g in par) || par[g] == 1 || (par[g] in init)) print g }'
}
show_groups() { ps -eo pid=,pgid=,args= | awk -v gs=" $* " 'index(gs, " " $2 " ")' | cut -c1-150; }

ports_in_use() {
  local p
  for p in $TCP_PORTS; do ss -H -ltnp "sport = :$p" 2>/dev/null; done
  for p in $UDP_PORTS; do ss -H -lunp "sport = :$p" 2>/dev/null; done
}

# ---------------------------------------------------------------- --stop
if [ "$mode" = stop ]; then
  if [ ! -f "$pidfile" ]; then
    echo "RadarSim: no run recorded in $pidfile"
    [ -n "$(leftover_groups)" ] && echo "RadarSim processes without a launcher are running: $0 --stop-leftovers stops them"
    exit 0
  fi
  read -r _ _ lpid lstart < "$pidfile"
  if same_process "$lpid" "$lstart"; then
    echo "RadarSim: asking the launcher (PID $lpid) to stop everything ..."
    kill -TERM "$lpid"
    for _ in $(seq 60); do same_process "$lpid" "$lstart" || break; sleep 0.5; done
    same_process "$lpid" "$lstart" && { echo "RadarSim: the launcher did not stop"; exit 1; }
    echo "RadarSim: stopped"
  else
    handle_previous_run
    echo "RadarSim: stopped what was left"
  fi
  exit 0
fi

# ---------------------------------------------------------------- checks before starting
handle_previous_run
mapfile -t left < <(leftover_groups)
if [ "${#left[@]}" -gt 0 ]; then
  if [ "$stop_leftovers" = 1 ]; then
    echo "RadarSim: stopping RadarSim processes that no launcher owns:"
    show_groups "${left[@]}"
    stop_items 10 "${left[@]/#/g:}" || echo "RadarSim: some needed SIGKILL"
  else
    echo "RadarSim processes from an earlier run are still running (no launcher owns them):"
    show_groups "${left[@]}"
    echo "Stop them and start with: $0 --stop-leftovers ...  (or stop that simulator yourself)"
    exit 1
  fi
fi
busy=$(ports_in_use)
if [ -n "$busy" ]; then
  echo "RadarSim: ports it needs are in use (another simulator?); nothing was started or stopped:"
  echo "$busy"
  exit 1
fi
# sim_vehicle.py on instance 0 kills every ArduPilot SITL by name, also one on another instance
sitls=$(ps -eo pid=,comm=,args= | awk '$2 ~ /^(arducopter(-heli)?|arduplane|ardurover|ardusub|antennatracker|blimp)$/' \
  | cut -c1-150)
if [ -n "$sitls" ]; then
  echo "RadarSim: another ArduPilot SITL is running; SITL on instance 0 would kill it (sim_vehicle.py stops every"
  echo "SITL by name). Stop it first; nothing was started or stopped:"
  echo "$sitls"
  exit 1
fi
if ps -eo args= | grep -Eq "$GZ"; then
  echo "RadarSim: another Gazebo is running: stop it first."
  exit 1
fi

# Gazebo environment: the RFSonde models and worlds, then the RadarSim plugins first on the plugin path
if [ -z "${GZ_VERSION:-}" ]; then
  env_file=${GZ_ENV:-$HOME/Github/ardupilot_gazebo/env.sh}
  if [ -f "$env_file" ]; then
    set +u; source "$env_file"; set -u
  else
    echo "Gazebo environment $env_file not found (set GZ_ENV)"; exit 1
  fi
fi
gz_build=${RADARSIM_GZ_BUILD:-$HOME/Github/ardupilot_gazebo-RadarSim/build}
if [ ! -f "$gz_build/libRFSondeTruthPlugin.so" ]; then
  echo "RadarSim plugins not built in $gz_build (RADARSIM.md, Setup)"; exit 1
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="$gz_build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# ---------------------------------------------------------------- run
# Each part in its own process group (setsid), so Ctrl+C reaches this script only and it stops the parts in order;
# disowned, so that bash does not report how a part ended (Gazebo sometimes crashes while it shuts down).
# Group leaders: G (Gazebo), S (rfsonde_sitl.sh: twin, sim_vehicle.py, the ArduCopter xterm), R (rfsonde_radarsim.sh:
# radar service, companion, GUI). Watched single processes: the twin, ArduCopter, the radar GUI.
G= S= R=
declare -A watch_pid=() watch_start=()
echo "launcher $$ $$ $(start_of $$)" > "$pidfile"

record_group() { echo "group $1 $2" >> "$pidfile"; }
record_proc() { echo "proc $1 $2 $3" >> "$pidfile"; watch_pid[$1]=$2; watch_start[$1]=$3; }

say() { echo "$@"; echo "$(date +%T) $*" >> "$log/stop.log" 2>/dev/null; }

stopping=0
cleanup() {
  local reason=$1 rc=${2:-0}
  [ "$stopping" = 1 ] && return
  stopping=1
  trap '' INT TERM HUP PIPE
  say ""
  say "RadarSim: stopping everything ($reason); takes up to about 15 s, Ctrl+C is ignored until it is done"
  # radar and companion first: rfsonde_radarsim.sh closes the truth file and stops the companion's SDR
  if [ -n "$R" ]; then
    kill -TERM "$R" 2>/dev/null
    for _ in $(seq 12); do group_alive "$R" || break; sleep 0.5; done
  fi
  local items=() n
  for g in $R $S $G; do items+=("g:$g"); done
  for n in "${!watch_pid[@]}"; do items+=("p:${watch_pid[$n]}:${watch_start[$n]}"); done
  if stop_items 10 "${items[@]}"; then
    say "RadarSim: all stopped"
  else
    say "RadarSim: all stopped (some parts needed SIGKILL)"
  fi
  rm -f "$pidfile"
  exit "$rc"
}
trap 'cleanup "Ctrl+C" 0' INT
trap 'cleanup "terminal closed or stop requested" 0' TERM HUP
trap '[ "$stopping" = 1 ] || cleanup "the launcher ended unexpectedly" 1' EXIT

started=$(date +%s)
gz_args=(-v4 -r "$world")
[ "$headless" = 1 ] && gz_args=(-v4 -s -r "$world")
setsid gz sim "${gz_args[@]}" > "$log/gz.log" 2>&1 < /dev/null &
G=$!; disown "$G"; record_group gazebo "$G"
say "RadarSim: Gazebo starting ($world; log $log/gz.log)"

sitl_opts=(--gps "$gps" "--add-param-file=$SIM/config/radarsim.parm")
[ "$squares" = 0 ] && sitl_opts+=(--no-squares)
setsid "$TOOLS/rfsonde_sitl.sh" "${sitl_opts[@]}" "${sitl_args[@]}" > "$log/sitl.log" 2>&1 < /dev/null &
S=$!; disown "$S"; record_group sitl "$S"
say "RadarSim: SITL, T7 twin ($gps GPS$([ "$squares" = 0 ] && echo ", no squares")) building and starting (log $log/sitl.log)"

# Watched parts: a group leader ending, or a watched process ending, stops everything. (The twin's restart on a
# Gazebo world reset re-executes it in place: same PID, same start time.)
check_parts() {
  local name leader
  for name in gazebo sitl radar; do
    case $name in gazebo) leader=$G ;; sitl) leader=$S ;; radar) leader=$R ;; esac
    [ -n "$leader" ] || continue
    kill -0 "$leader" 2>/dev/null && continue
    case $name in
      gazebo) cleanup "Gazebo was closed or stopped (see $log/gz.log)" 1 ;;
      sitl) cleanup "SITL stopped (see $log/sitl.log)" 1 ;;
      radar) cleanup "the radar launcher stopped (see $log/launcher.log)" 1 ;;
    esac
  done
  local table
  table=$(ps -eo pid=,ppid=,pgid=,args=)
  # the Gazebo window is a child of the server (gz-sim-gui-client)
  [ "$headless" = 0 ] && discover gazebo_window "$G" '^[^ ]*/gz-sim-gui-client ' "$table"
  discover twin "$S" '^[^ ]*/t7_twin --gz' "$table"
  discover arducopter "$S" '^[^ ]*/arducopter .*-I0( |$)' "$table" parent
  [ "$gui" = 1 ] && [ -n "$R" ] && discover radar_gui "$R" '^[^ ]*python[0-9.]* -m rfsonde_radarsim\.gui' "$table"
  for name in "${!watch_pid[@]}"; do
    same_process "${watch_pid[$name]}" "${watch_start[$name]}" && continue
    case $name in
      twin)
        tail -3 "$HOME/.cache/rfsonde/t7_twin_I0.log" 2>/dev/null | sed 's/^/  twin: /'
        cleanup "the T7 twin stopped (see ~/.cache/rfsonde/t7_twin_I0.log)" 1 ;;
      gazebo_window) cleanup "the Gazebo window was closed" 1 ;;
      arducopter) cleanup "ArduCopter stopped or its window was closed" 1 ;;
      radar_gui) cleanup "the radar GUI was closed" 1 ;;
    esac
  done
}
# discover NAME GROUP PATTERN TABLE [parent]: record, once, the first process whose own command line matches PATTERN
# and that is in process group GROUP (with "parent": whose parent is; ArduCopter runs in its own session under the
# xterm that sim_vehicle.py opened)
discover() {
  local name=$1 group=$2 pat=$3 table=$4 by=${5:-self} pid
  [ -n "${watch_pid[$name]:-}" ] && return
  pid=$(PAT="$pat" awk -v g="$group" -v by="$by" '
    {pg[$1] = $3; par[$1] = $2; a = $0; sub(/^ *[0-9]+ +[0-9]+ +[0-9]+ /, "", a); if (a ~ ENVIRON["PAT"]) c[++n] = $1}
    END {for (i = 1; i <= n; i++) if ((by == "parent" ? pg[par[c[i]]] : pg[c[i]]) == g) {print c[i]; exit}}' <<< "$table")
  [ -n "$pid" ] && record_proc "$name" "$pid" "$(start_of "$pid")"
}

# The radar waits for this run's truth ring (a ring left by an earlier Gazebo is older than this start)
say "RadarSim: waiting for Gazebo's truth ring ..."
until [ -e "$ring" ] && [ "$(stat -c %Y "$ring" 2>/dev/null || echo 0)" -ge "$started" ]; do
  check_parts
  sleep 1
done

radar_opts=("${radar_args[@]}")
[ "$gui" = 1 ] && radar_opts+=(--gui)
setsid "$TOOLS/rfsonde_radarsim.sh" "${radar_opts[@]}" < /dev/null > >(tee -i "$log/launcher.log") 2>&1 &
R=$!; disown "$R"; record_group radar "$R"

announced=0
while :; do
  check_parts
  if [ "$announced" = 0 ] && ss -ltn 2>/dev/null | grep -q ":5760 "; then
    say "RadarSim: SITL is waiting: connect Ludifly to TCP 127.0.0.1:5760. Ctrl+C here, closing Gazebo, the radar"
    say "          GUI or the ArduCopter window, or '$(basename "$0") --stop' elsewhere stops everything."
    announced=1
  fi
  sleep 1
done
