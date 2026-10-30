# RFSonde Gazebo worlds

## norman_radars: Max Westheimer Airport radar site, Norman, OK

The world covers a 2 km radius around KCRI. It includes generic WSR-88D models for KCRI and KOUN, the NSSL ATD and its calibration tower, and the real OpenStreetMap layout of buildings, runways, taxiways, roads and trees. All of them sit at their true positions.

### Generate (once after cloning)

The world and its meshes are generated files and are not committed. Build them from the OSM data:

```bash
python3 ~/Github/my-ardupilot/RFSonde-sim/tools/make_norman_world.py
```

### Run

```bash
# Terminal A
gz sim -v4 -r norman_radars.sdf

# Terminal B: SITL, the T7 gimbal twin and GPS accuracy squares (--rtk to start with RTK, --no-squares to skip them)
~/Github/my-ardupilot/RFSonde-sim/tools/rfsonde_sitl.sh

# Terminal C (optional): MAVProxy console for the gimbal and quick commands
mavproxy.py --master tcp:127.0.0.1:5762 --console
```

`rfsonde_sitl.sh` runs `sim_vehicle.py` with the `RFSonde_Norman` location, `config/rfsonde-t7.parm`, `config/gps-standard.parm` (or `gps-rtk.parm` with `--rtk`) and `--no-mavproxy`, and starts `tools/accuracy_squares.py` in the background. Before that it builds and starts the Gremsy T7 digital twin (see [Vehicle and gimbal](#vehicle-and-gimbal)). Other `sim_vehicle.py` options are passed on, and Ctrl+C stops everything. Connect your ground station to TCP `127.0.0.1:5760`: SITL does not start, or link to Gazebo, until that connection is made. SITL also accepts connections on TCP 5762 (the MAVProxy console) and 5765 (the squares helper); 5763 is SERIAL2, the gimbal. Don't give MAVProxy an `--out` to a port your ground station listens on: every packet would arrive twice, and mission uploads fail with `MAV_MISSION_INVALID_SEQUENCE`.

The previous servo gimbal is still available: `gz sim -v4 -r norman_radars_servo.sdf` and `rfsonde_sitl.sh --servo-gimbal` (`config/rfsonde-gimbal.parm`, squares helper on 5763).

`RFSonde_Norman` is defined in `~/.config/ardupilot/locations.txt`. It must match the world origin below, or SITL GPS positions will be wrong.

### Frame and origin

- The Gazebo world frame is ENU: x = East, y = North, z = Up, in metres.
- The origin is also the drone takeoff point: **35.2380802 N, 97.4631246 W, 358.6 m MSL**. It sits in the open field 50 m east and 200 m north of the ATD radome.
- The drone starts facing south-east (heading 135°). Change `TAKEOFF_HEADING_DEG` in `tools/make_norman_world.py` and the last number of `RFSonde_Norman` together.
- Ground level is 358.6 m MSL (NAVD88), taken from the ATD survey of the calibration tower base (1176.42 ft).
- SITL converts positions to lat/lon with ArduPilot's simplified spherical Earth. Its reported GPS position differs from the true geodetic position by about 0.2% of the distance from takeoff, which is under 1 m at the radars. Gazebo geometry is exact WGS84, so use Gazebo truth for RF ranges.

### Radar positions

| Model | E (m) | N (m) | Height | Source |
|---|---|---|---|---|
| `KCRI` | 260.6 | 23.0 | 30 m tower, antenna centre 34.7 m AGL | OSM node 2336571439; OSF-3 site survey (1994): 35°14'18"N 97°27'37"W NAD83, 2950 MHz |
| `KOUN` | 70.7 | -224.6 | 20 m tower, antenna centre 24.7 m AGL (81 ft) | NWS PNS 35°14'09.81"N 97°27'44.46"W; OSF site survey (1987) |
| `ATD` | -51.8 | -199.7 | radome centre 10.07 m AGL (368.668 m ortho) | RFSonde ATD survey (WGS84) |
| `ATD_cal_tower` | -39.8 | 225.3 | probe at 44.63 m AGL (403.23 m ortho) | RFSonde ATD survey |

Each radar model has named SDF frames for RF calculations:

- `KCRI::antenna_center` and `KOUN::antenna_center`
- `ATD::radome_center` and `ATD::array_center`. The array face is 1.63 m forward of the radome centre and points at azimuth 1.62°, toward the calibration tower.
- `ATD_cal_tower::probe`, which points at the ATD

### Vehicle and gimbal

The world flies `models/rfsonde_iris_t7` (named `rfsonde_iris` in the world): the ArduPilot Iris carrying `models/rfsonde_t7`, a 3-axis gimbal in the Gremsy T7 layout (vehicle → pan → roll → tilt, mechanical stops at tilt −150°, roll ±80.5°, pan ±347°).

**Gremsy T7 digital twin.** Nothing in Gazebo or ArduPilot moves this gimbal directly. The `GremsyT7Plugin` (ardupilot_gazebo, branch `main-GremsyT7`) exchanges every 1 ms physics step with `t7_twin`, the real T7 firmware v7.8.7 compiled from `RFSonde_Gremsy/T7_firmware_RE/src/c` and running on simulated hardware. The plugin sends the joint angles and the camera-plate IMU; the twin answers with the motor torques. The autopilot reaches the gimbal over MAVLink on SERIAL2 (`config/rfsonde-t7.parm`: `MNT1_TYPE 6`, `SERIAL2_PROTOCOL 2`, 115200, `MAV3_OPTIONS 2`, `SIM_BAUDLIMIT_EN 1`), the same as the real Cube, and reports `Mount: GREMSY T7V1 fw:7.8.7.0`.

- `rfsonde_sitl.sh` rebuilds the twin at every start, so an edit of the T7 firmware C is in the simulator at the next launch. It finds the twin in `T7_TWIN_DIR` (default `/mnt/c/Users/sega0000/Github/RFSonde_Gremsy/T7_twin`).
- The twin's parameter EEPROM is `~/.cache/rfsonde/t7_eeprom_I0.bin`. The first launch creates it: first-boot encoder and home calibration, Gremsy's recommended profile and a gyro calibration (`t7tool.py factory`, about 3 minutes). Delete it to start from a new board.
- The twin's log is `~/.cache/rfsonde/t7_twin_I0.log`. Its telemetry is `t7_twin_I0.csv`: true joints and torques, the firmware's attitude, and the true antenna boresight azimuth/elevation. `RFSONDE_T7_ARGS=--console` adds the firmware's console messages to the log.
- The gimbal's gTune port is TCP `5800`, for `T7_twin/tools/t7tool.py` (parameters, calibrations, motors, mode).
- The twin and the plugin use UDP 9150; the plugin's `twin_port` in `rfsonde_t7/model.sdf` must match `9150 + 10 × instance`.
- The parameter files are SITL defaults: values saved in SITL's `eeprom.bin` in earlier sessions (for example `MNT1_TYPE 1`) override them. If the mount is not detected, start once with `-w` to wipe them.

Details: `RFSonde_Gremsy/T7_twin/README.md`.

**Servo gimbal** (`norman_radars_servo.sdf`, `rfsonde_sitl.sh --servo-gimbal`). It flies `models/rfsonde_iris`, the ArduPilot Iris carrying `models/rfsonde_gimbal`. Both are copies of the upstream ardupilot_gazebo models, so the upstream repo stays untouched. The gimbal is a 3-axis servo gimbal (vehicle → yaw → roll → pitch). `config/rfsonde-gimbal.parm` holds the upstream gimbal parameters plus `MNT1_DEFLT_MODE 2`, so the gimbal boots at neutral, and a 500-2500 µs servo range (instead of 1100-1900) so a 1 µs PWM step moves the gimbal at most 0.16° (yaw). The `servo_min`/`servo_max` of channels 8-10 in `rfsonde_iris/model.sdf` must match `SERVO9-11_MIN/MAX`.

| Axis | ArduPilot limits | Positive | Servo / RC | Gazebo joint |
|---|---|---|---|---|
| Roll | -30 to +30° | right side down | SERVO9 / RC6 | `roll_joint`, same sign |
| Pitch | -135 to +45° | up | SERVO10 / RC7 | `pitch_joint`, opposite sign |
| Yaw | -160 to +160° | right | SERVO11 / RC8 | `yaw_joint`, opposite sign |

**Frame triad.** It is drawn on the `antenna_rf` frame and moves with the gimbal:

- X (red) is the boresight. It continues as a 300 m laser line and a translucent beam cone (30° beamwidth, 20 m).
- Y (green) points left at neutral.
- Z (blue) points up at neutral; it is the vertical-polarization reference.

None of these have mass or collision, and the gimbal camera image does not show them. Set the `antenna_rf` pose in `rfsonde_gimbal/model.sdf` to your antenna phase centre. To resize the cone, edit `pointer_beam_cone`: radius = length × tan(beamwidth / 2).

**MAVProxy control** (Terminal C). Run `echo "module load gimbal" >> ~/.mavinit.scr` once, so MAVProxy loads its gimbal module at every start. Then:

```
gimbal point 0 -30 40                                      # roll pitch yaw (deg)
gimbal mode RC                                             # then rc 6/7/8 1100-1900
param set MNT1_ARRC_GMODE 0                                # standard ArduPilot ROI
long DO_SET_ROI_LOCATION 0 0 0 0 35.2382871 -97.4602619 34.69   # aim at the KCRI antenna
setyaw 194.5 30 0                                          # face the ATD first (gimbal yaw is limited to ±160°)
```

`MNT1_ARRC_GMODE` sets how the RFSonde firmware aims at an ROI. 0 is standard ArduPilot, 1 keeps the probe plane parallel to the antenna under test, 2 is H-pol aligned and 3 is V-pol aligned. Modes 1–3 use `MNT1_ARRC_AZTH`, `_ELEV` and `_ZOFF`. The SITL defaults set mode 2.

**Actual angles.** Gazebo publishes the achieved joint angles, in radians (both gimbals):

```bash
gz topic -e -t /world/norman_radars/model/rfsonde_iris/model/gimbal/joint_state
```

**Firmware requirement.** The gimbal only holds its angles in flight with the `AP_Mount_Servo` fix (October 2026). `MNT1_ARRC_AZTH` and `MNT1_ARRC_GMODE` reuse the storage of the stock servo lead-filter gains. Without the fix, roll and pitch saturate whenever the vehicle rotates.

**Parameter storage.** The ARRC mount parameters sit at `AP_Mount_Params` indices 12-16 and 19-21, where upstream ArduPilot has `_LEAD_RLL`, `_LEAD_PTCH`, `_SYSID_DFLT`, `_DEVID` and `_OPTIONS`. A board that comes from upstream firmware keeps those values in storage: `MNT1_SYSID_DFLT` then shows the old `MNT1_OPTIONS` value (same index 16 and type), while `MNT1_ARRC_ELEV` and `_ZOFF` (floats at 14 and 15) start at their defaults. After flashing this firmware over an upstream build, check `MNT1_SYSID_DFLT` and set `MNT1_OPTIONS` again. The indices are stable within this branch; renumbering them would reset every ARRC vehicle's gimbal parameters.

### GPS accuracy and RTK

SITL simulates the GPS receiver; Gazebo only supplies the true position. The RFSonde GNSS error model (`libraries/SITL/SIM_GPS.h`, `GNSSErrorModel`) adds two terms to each axis:

- a slowly varying error, a first-order Gauss-Markov process with exact discretisation, so the statistics don't depend on `SIM_GPS1_HZ`;
- white noise.

It also adds white noise to the velocity. The simulated u-blox reports the configured 1-sigma accuracy (`hAcc`, `vAcc`, `sAcc`), and reports RTK Fixed when `hAcc` is below 0.1 m. With all model parameters at 0, the stock SITL GPS applies (`SIM_GPS1_ACC`, `SIM_GPS1_NOISE`).

| Parameter | Meaning |
|---|---|
| `SIM_GPS_HBIAS` / `SIM_GPS_VBIAS` | slowly varying error, 1σ per horizontal axis / vertical (m) |
| `SIM_GPS_BTAU` | its correlation time (s) |
| `SIM_GPS_HWN` / `SIM_GPS_VWN` | white noise per sample (m) |
| `SIM_GPS_VELNSE` | velocity noise per axis (m/s) |
| `SIM_GPS_SEED` | random seed (0 = new each start) |

The profiles (u-blox ZED-F9P class) include matching EKF3 settings:

| Profile | Horizontal | Vertical 1σ | Fix | EKF3 |
|---|---|---|---|---|
| `config/gps-standard.parm` | 1.5 m CEP | 1.9 m | 3D | defaults (`EK3_POSNE_M_NSE 0.5`) |
| `config/gps-sbas.parm` | 1.0 m CEP | 1.3 m | 3D | defaults |
| `config/gps-rtk.parm` | 1.1 cm CEP | 1.4 cm | RTK Fixed | `EK3_POSNE_M_NSE 0.05`, `EK3_VELNE_M_NSE 0.1` |

All profiles set `EK3_SRC1_POSZ 3` (GPS height). EKF3 weighs GPS position by `max(hAcc, EK3_POSNE_M_NSE)`, so the RTK profile lowers that floor.

Start with `tools/rfsonde_sitl.sh --gps sbas` or `--rtk`. To switch at any time, run in Terminal C:

```
param load ~/Github/my-ardupilot/RFSonde-sim/config/gps-rtk.parm        # or gps-sbas.parm / gps-standard.parm
```

In Mission Planner, use Config → Full Parameter List → Load from file → Write Params. The error changes size immediately, with no RTK float stage. Home altitude is set at arming, so arming in standard mode and then enabling RTK leaves a constant height offset until the next arming.

### Evaluate GNSS and EKF accuracy

```bash
python3 tools/gnss_flight_test.py --profile rtk              # takeoff, hover, box, land, then evaluate (TCP 5762)
python3 tools/gnss_flight_test.py --profile standard --hover 300 --box 30
python3 tools/gnss_eval.py --latest                          # evaluate the newest log in ~/Github/my-ardupilot/logs
```

`gnss_eval.py` compares the raw GPS (delayed by `SIM_GPS1_LAG_MS`) and the EKF3 estimate with the true position. It reports, for the whole flight, hovering and moving:

- error per axis, RMS, CEP50/95 and vertical RMS;
- the fast-noise RMS;
- how closely EKF3 follows the GPS error;
- reported vs actual accuracy;
- the error's correlation time;
- timing: the measured GPS delay, and how far the EKF3 output trails the truth (about 20–30 ms in SITL, which adds about speed × lag while moving);
- EKF3 innovation test ratios (`XKF4`).

The report is saved next to the log as `*_gnss_report.txt`, and `--csv` exports the time series. With a correlation time of a minute, one short flight can read ±20% from the configured accuracy; set `SIM_GPS_SEED` to compare configurations on the same error sequence. To check the model's own statistics, record a long stationary run (`LOG_DISARMED 1`, `SIM_SPEEDUP 10`, several times `SIM_GPS_BTAU`), then run `gnss_eval.py --all`.

### Accuracy squares

`tools/accuracy_squares.py` draws a red 1 m square and an orange 60 cm square, each with a vertical counterpart across the drone. They are centred on ArduPilot's estimated position (MAVLink on TCP 5763), level with the ground and turned with the drone's heading. The drone model is at its true position, so it moves inside the squares by the actual position error. They are Gazebo markers: visible in the Gazebo window only, with no physics, and not in camera images.

The squares are on by default with `rfsonde_sitl.sh`. These commands work from any terminal and act on the running helper:

| To | Do this |
|---|---|
| Hide the squares | `python3 tools/accuracy_squares.py --off` |
| Show them again | `python3 tools/accuracy_squares.py --on` |
| Show one square | `--on --show large` (1 m) or `--on --show small` (60 cm) |
| Start SITL without squares | `tools/rfsonde_sitl.sh --no-squares` |
| Squares for a SITL started with `sim_vehicle.py` directly | `python3 tools/accuracy_squares.py` (Ctrl+C removes them) |

The helper waits for SITL, reconnects if SITL restarts, and logs to `~/.cache/rfsonde/accuracy_squares.log`.

### Assumptions

- **Radomes:** all are 11.89 m (39 ft) WSR-88D radomes. The ATD radome size is not published, so it is assumed equal to the WSR-88D radome.
- **Radar models:** towers, shelters and radomes are generic shapes, not the real structures.
- **Buildings:** heights come from OSM `height` or `building:levels` when present. Otherwise a generic height per building type is used: houses 5 m, commercial 6 m, hangars 9 m, university buildings 8 m.
- **Trees:** OSM trees are used where they exist. 49 extra trees in 7 clusters around the radar compound are generic placements, seeded so the world is repeatable.
- **Other structures:** water towers, communication masts and the airport control tower are placed at their OSM positions with generic heights.
- **RF properties:** none. Meshes only provide geometry, for line-of-sight and blockage tests.

### Regenerate

Edit `tools/make_norman_world.py` (positions, heights, takeoff point), then run it again:

```bash
python3 ~/Github/my-ardupilot/RFSonde-sim/tools/make_norman_world.py
```

This rewrites `models/norman_*`, `worlds/norman_radars.sdf` (T7 twin) and `worlds/norman_radars_servo.sdf` (servo gimbal), all ignored by git. If you change the takeoff point, update `RFSonde_Norman` in `~/.config/ardupilot/locations.txt` too; the script prints the new values.

The OpenStreetMap data is in `osm/norman_osm.json` (© OpenStreetMap contributors, ODbL).
