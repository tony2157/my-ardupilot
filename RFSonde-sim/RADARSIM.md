# RadarSim: a dual-pol phased-array radar in the RFSonde simulation

RadarSim puts an ATD-like dual-polarized phased-array radar into the norman_radars world. Nothing of it is drawn.
For every radar pulse it computes the field at the drone's horn and passes it to the companion computer, which
receives it through an emulated bladeRF 2.0 micro xA9 and its GPSDO and measures it like a real flight.

| Part | Repository (branch) | What it does |
|---|---|---|
| Truth plugin | ardupilot_gazebo `main-RadarSim`: `RFSondeTruthPlugin` | sim time and the true poses of the horn phase centre, the vehicle and the gimbal, every 1 ms step, in `/dev/shm/rfsonde_truth_I<n>` |
| Radar service | RFSonde_NF2FF `main-RadarSim`: `radarsim/` | the array field at the horn (exact element sum), ground reflection, WR-284 horn reception, per pulse; truth HDF5 |
| Emulated SDR | RFSonde_Computer `main-RadarSim`: `rfsonde_engine/sim/` | AD9361 receive chain, VCTCXO or LBE-1420 GPSDO, sample stream on sim time |
| Vehicle, SITL | my-ardupilot `ArduPilot-4.7-ARRC-RadarSim` | the RFSonde gimbal drop (0.46 m) and the truth plugin in `models/rfsonde_iris_t7`; exact WGS84 geodesy, geoid and RTK model in SITL (`config/gps-rtk.parm`) |

**Time.** Everything runs on Gazebo sim time: SITL (lockstep), the T7 twin, the radar's pulses, the SDR's samples
and the companion's timing chain (`RFSONDE_SIM_CLOCK`). A slow Gazebo slows them together.

## Setup (once)

```bash
# Gazebo plugins: build the RadarSim worktree (your main checkout stays on its branch)
cd ~/Github/ardupilot_gazebo-RadarSim && mkdir -p build && cd build && GZ_VERSION=jetty cmake .. && make -j8
# radar service venv
cd /mnt/c/Users/sega0000/Github/RFSonde_NF2FF/radarsim && python3 -m venv venv && venv/bin/pip install numpy scipy numba h5py pyyaml matplotlib pytest dearpygui
# companion venv (docs/DEVELOPMENT.md of RFSonde_Computer)
cd /mnt/c/Users/sega0000/Github/RFSonde_Computer && python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"
```

## Run

```bash
# Terminal A: Gazebo with the RadarSim plugins first on the plugin path
GZ_SIM_SYSTEM_PLUGIN_PATH=~/Github/ardupilot_gazebo-RadarSim/build:$GZ_SIM_SYSTEM_PLUGIN_PATH gz sim -v4 -r norman_radars.sdf
# Terminal B: SITL as usual, with the RTK profile and the RadarSim gimbal parameters
~/Github/my-ardupilot/RFSonde-sim/tools/rfsonde_sitl.sh --rtk --add-param-file=$HOME/Github/my-ardupilot/RFSonde-sim/config/radarsim.parm
# Terminal C: the radar service and the companion (prints MNT1_ARRC_AZTH/ELEV for the scenario's beam);
# --gui also opens the radar GUI
~/Github/my-ardupilot/RFSonde-sim/tools/rfsonde_radarsim.sh --beam -10,3 --gui
```

The T7 twin may start before or after Gazebo (`rfsonde_sitl.sh` starts it): the gimbal plugin locksteps with it as
soon as it answers. An older RadarSim build ran a late twin's start-up motor test without its torques, so the T7
latched its motors off and ArduPilot reported "Mount: not healthy"; rebuild the RadarSim plugins if you see that.

The companion connects to SITL on TCP 5762 (+10 per instance), so don't put the MAVProxy console there. Ludifly
shows the measurements as from the real CC. Data and logs: `~/.cache/rfsonde/radarsim_I<n>/` (the companion's HDF5
under `cc/data`, the radar truth `truth_*.h5`). Score a flight against the truth:

```bash
cd /mnt/c/Users/sega0000/Github/RFSonde_NF2FF/radarsim
venv/bin/python -m rfsonde_radarsim.scoring --truth ~/.cache/rfsonde/radarsim_I0/truth_<time>.h5 \
  --cc ~/.cache/rfsonde/radarsim_I0/cc/data --out score.json --fig fig
```

## The radar GUI

`rfsonde_radarsim.sh --gui`, or on its own in another terminal while the launcher keeps running (Ctrl+C in the
launcher's terminal stops the radar service and the companion, so Ludifly shows no SDR):

```bash
cd /mnt/c/Users/sega0000/Github/RFSonde_NF2FF/radarsim && venv/bin/python -m rfsonde_radarsim.gui --instance 0
```

One panel: operating frequency, embedded element (isotropic, cos, cos^2, cos^n, dipole over ground, patch, Huygens),
the face's mechanical azimuth and elevation (earth frame), pedestal rotation at a set rate (off by default: the radar
stays at the entered angles), electronic steering, dead elements and per-element power and phase errors (standard
deviations, random from a seed). Every change reaches the running radar at the next physics step, without losing a
pulse, and is logged in the truth file (`config_log`); a new frequency's horn pattern follows about 2.5 s later.
Retune the SDR from Ludifly when you change the radar frequency: the companion receives off-tune pulses out of band.
The plot shows the transmit pattern within 10 deg of the beam (H or V port, co- or cross-pol, EIRP or phase, far field
or at a range) and where the drone is in it. GUI azimuths are clockwise seen from above (the scenario's `beams.az_deg`
is minus the GUI's steering azimuth). The control link is UDP 15975 + 10 x instance.

## The radar

`radarsim/scenarios/atd_stare.yaml` (sources and assumptions in its comments): the ATD's 76 panels x 8 x 8 dual-pol
elements on a 4 m face at the surveyed radome position, the face 1.63 m in front of the pedestal axis, facing the
calibration tower; EIRP 84.8 / 85.3 dBW (H / V); uniform transmit taper; element, panel, phase-shifter and leakage
errors; SHV transmission; the triple-PRT dwell (12 x 3 ms, 20 x 1 ms, 15 x 1.5 ms) with a 38 us pulse-compression
pulse and a 1.8 us fill pulse 8 MHz up. The beam is parked (`beams`); electronic steering is `az_deg`, `el_deg` off
broadside (+az toward the face's +x, which points west when the face looks north). The pedestal is stationary unless
`rotation` is enabled (scenario or GUI).

## Assumptions to replace with measurements

- Horn: an effective aperture fitted to the Pasternack PEWAN284-10NF datasheet (gain within 0.15 dB, HPBW within
  1.5 deg): measure the inner aperture and the flare lengths of yours.
- Gimbal payload mass stays light (0.30 kg) so the Iris hovers; the inertias are the RFSonde horn's.
- ATD panel map, element type, operating frequency (2.8 GHz here), error levels, oscillator: not published.
- SDR noise: the board noise figure measured by the Libre Space Foundation at 2.3 GHz and your R6 post-gain floor.
