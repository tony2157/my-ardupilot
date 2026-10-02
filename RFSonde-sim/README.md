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
WIN_IP=$(ip route show default | awk '{print $3}')
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  -L RFSonde_Norman --console --map --out=$WIN_IP:14550
```

`RFSonde_Norman` is defined in `~/.config/ardupilot/locations.txt`. It must match the world origin below, or SITL GPS positions will be wrong.

### Frame and origin

- The Gazebo world frame is ENU: x = East, y = North, z = Up, in metres.
- The origin is also the drone takeoff point: **35.2380802 N, 97.4631246 W, 358.6 m MSL**. It sits in the open field 50 m east and 200 m north of the ATD radome.
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
