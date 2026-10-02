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

# Terminal B
cd ~/Github/my-ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -L RFSonde_Norman \
  --add-param-file=$HOME/Github/my-ardupilot/RFSonde-sim/config/rfsonde-gimbal.parm \
  --no-mavproxy

# Terminal C (optional): MAVProxy console for the gimbal and quick commands
mavproxy.py --master tcp:127.0.0.1:5762 --console
```

Connect your ground station to TCP `127.0.0.1:5760`. With `--no-mavproxy`, SITL does not start (and does not link to Gazebo) until that connection is made. SITL also accepts a second connection on TCP 5762, which the optional MAVProxy console uses. Don't give MAVProxy an `--out` to a port your ground station listens on: every packet would arrive twice, and mission uploads fail with `MAV_MISSION_INVALID_SEQUENCE`.

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

The world flies `models/rfsonde_iris`, the ArduPilot Iris carrying `models/rfsonde_gimbal`. Both are copies of the upstream ardupilot_gazebo models, so the upstream repo stays untouched. The gimbal is a 3-axis servo gimbal (vehicle → yaw → roll → pitch). The launch command under Run loads `config/rfsonde-gimbal.parm`: the upstream gimbal parameters plus `MNT1_DEFLT_MODE 2`, so the gimbal boots at neutral.

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

**Actual angles.** Gazebo publishes the achieved joint angles, in radians:

```bash
gz topic -e -t /world/norman_radars/model/rfsonde_iris/model/gimbal/joint_state
```

**Firmware requirement.** The gimbal only holds its angles in flight with the `AP_Mount_Servo` fix (October 2026). `MNT1_ARRC_AZTH` and `MNT1_ARRC_GMODE` reuse the storage of the stock servo lead-filter gains. Without the fix, roll and pitch saturate whenever the vehicle rotates.

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

This rewrites `models/norman_*` and `worlds/norman_radars.sdf` (both ignored by git). If you change the takeoff point, update `RFSonde_Norman` in `~/.config/ardupilot/locations.txt` too; the script prints the new values.

The OpenStreetMap data is in `osm/norman_osm.json` (© OpenStreetMap contributors, ODbL).
