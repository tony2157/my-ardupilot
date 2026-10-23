# ArduPilot-4.7-ARRC-revamp and Ludifly-SDR-revamp: operator notes and hardware test plan

2026-10-23. This covers the fixes for the 2026-10-22 safety review
(`REVIEW_2026-10-22_ARRC-4.7.md`) and the separate RF stream from
`PROPOSAL_2026-10-23_ARRC_RF_STREAM.md`. Everything was verified in SITL only. Nothing has run on
the Cube, the Gremsy T7, the LB5900, the Herelink or the LattePanda yet. Section 6 is the bench
plan for that.

AI-assisted (Claude Code): the code, the SITL harnesses and this document. Review every change
before flying it.

## 1. Branches and builds

| Repository | Branch | Base | Commits |
|---|---|---|---|
| my-ardupilot | `ArduPilot-4.7-ARRC-revamp` | `ArduPilot-4.7-ARRC` 9afcd1ca49 | 19 |
| RFSonde_GUI | `Ludifly-SDR-revamp` | `Ludifly-SDR` 538ce1d | 5 |

These were not changed:

- the MAVLink dialect (hash 0x7EFB3788, same pins everywhere);
- RFSonde_Computer;
- `Tools/autotest/default_params/copter.parm`;
- the dead FLTMODE2 slot (H2: PosHold is compiled out).

Nothing is pushed.

- **Firmware to flash:** `build/CubeOrangePlus-ODID/bin/arducopter.apj`.
  - 1,721,092 B of flash used, 244,980 B free.
  - That is 6,480 B more than ArduPilot-4.7-ARRC.
  - 0 compiler warnings.
- **Variant builds** (each needs a `./waf configure` with the option): all compile.
  - CubeOrangePlus-ODID: default, without the SDR, without the LB5900, without the mount.
  - SITL: without the SDR.
- **Ludifly:** run it from source, or rebuild the exe from `Ludifly-SDR-revamp`. The installed exe
  is the old Ludifly. It works with the new firmware (section 4) but does not have the leaner link.

## 2. Configuration to check on the Cube

**New parameters** (no index reused):

| Parameter | Default | What it is |
|---|---|---|
| `MAVn_ARRC` (index 19) | 0 | Rate of the RF data stream group on port n. Leave it at 0: Ludifly asks for the RF data per message. Set it only for a GCS that cannot ask, e.g. `MAV2_ARRC 10` for Mission Planner on the Herelink. |
| `MNT1_ARRC_SLEW` (index 23) | 60 deg/s | In MNT1_ARRC_GMODE 1-3, each gimbal angle command moves at most this fast. 0 disables. |
| `USR_SDR_OPT` bit 3 | off | Refuse arming while the LB5900 is an RF source and has no fresh reading. |

**Changed meaning:**

- **`MNT1_ARRC_GMODE`.** 0 now points exactly like stock ArduPilot (nadir allowed, no 10 m hold,
  no ARRC limits). Any value other than 0-3 points like 0, warns "Mount: MNT1_ARRC_GMODE x
  invalid, using 0" and fails the pre-arm check.

**Recommended settings** (configuration only, not done by the firmware):

- `MNT1_DEFLT_MODE 1` (Neutral), not 0 (Retract). Finding H1: Retract turns the T7 motors off at
  boot and after every ROI-none.
- `SERIALn_PROTOCOL 2` on the companion, gimbal and Herelink ports. A MAVLink1 port that carries
  the companion, the gimbal or an RF stream now fails pre-arm with "SERIALn is MAVLink1: set
  SERIALn_PROTOCOL 2".
  - The ADS-B receiver on SERIAL5 (MAVLink1) does not trigger this unless `MAV6_ARRC` is above 0.
  - Checked in SITL with the real parameter set: pre-arm passes.
- `USR_RF_SRC 3` (Both) when the LB5900 is fitted, so the LB5900 is not switched off once the SDR
  is detected.

**Operator notes carried over from the review:**

- **L16.** A switch between stock ArduPilot and ARRC firmware reinterprets the MNT1 parameters at
  indices 12-18. Reset `MNT1_*` after such a switch.
- **L17.** A mission ROI persists through RTL, Land and disarm (intentional). Clear it with ROI
  none, or from Ludifly's Gimbal window. With `MNT1_DEFLT_MODE 1` that gives Neutral, not
  Retract.

## 3. What behaves differently

### RF data link (autopilot to Ludifly)

- The RF data have their own stream group (`STREAM_ARRC`): ARRC_SENSOR_RAW (228, SDR) and
  CASS_SENSOR_RAW (227, LB5900).
  - Requests for all streams, or for EXTRA3, no longer touch the RF rate.
  - This holds whether they come from Ludifly, Mission Planner or the Herelink's QGC.
- Each RF measurement is sent once, as it arrives, capped at the requested rate.
- The companion port gets no RF data back.
- **LB5900 downlink:**
  - Each reading is sent once, with a reading counter in `values[4]`.
  - `values[1..3]` are NaN.
  - While the sensor is missing or failing, NaN power is sent once a second.
- **Ludifly:**
  - It asks for each message it reads at its own rate. ATTITUDE 10 Hz; GLOBAL_POSITION_INT and
    the gimbal attitude 5 Hz; VFR_HUD and ADSB_VEHICLE 4 Hz; SYS_STATUS and GPS_RAW_INT 2 Hz;
    the rest 1 Hz.
  - It asks again for any of these that falls below half its rate (checked every 10 s).
  - The RF Data Readout shows how the RF rate was set:
    - "30 Hz by message interval": normal.
    - "message interval (no answer yet, asking again)": no ACK yet; Ludifly asks again every 5 s.
    - "EXTRA3 stream": the firmware refused the interval for 228 (very old firmware only).
  - A NaN LB5900 value is no reading: a gap, and the LB5900 is not shown as a live source.

### Gimbal (AP_Mount, GMODE 1-3)

- Of the two orientations that align the polarization, the one closest to the last command is
  used.
- Each angle is slew-limited by `MNT1_ARRC_SLEW`. The 80-180 deg steps of the review (M3) are
  gone.
- Within 10 m of the AUT the last solution is held.
- In singular geometry (probe on the AUT's y axis) the gimbal points like GMODE 0 instead of
  freezing.
- A NaN or non-unit attitude from the gimbal is rejected. The old firmware raised
  INTERNAL_ERROR constraining_nan, which blocks arming.
- The MNT log pairs desired and actual angles in the same convention.
- ARGM changes:
  - a new `MM` field (mount mode);
  - flag bit 3 = slew-limited.

  ARRC_GIMBAL_POINTING carries only the documented flag bits.

### Polarization alignment (aux switch, RCx_OPTION 47)

| Switch | Effect |
|---|---|
| HIGH | Starts the scan with the RF source in use: "Executing Gimbal alignment (SDR)" or "(LB5900)". |
| LOW | Stops a running scan and clears the offset. |
| MIDDLE | Nothing. |

Results:

- **Accepted** only when the fitted offset is known to ±2 deg: "Gimbal alignment correction: x
  deg", then "Gimbal alignment success: +-s deg, R = r".
- **Rejected:** "Gimbal alignment: x deg +-s, R r", then "Gimbal alignment failed: offset too
  uncertain".
- **Other failures** ("no power samples", "too few power samples", "no power variation", "gimbal
  mode changed") and a LOW stop put back the offset the scan started from.

### Companion computer

- The companion is bound to the MAVLink channel of its heartbeat.
  - A second "companion" on another link is ignored.
  - After the real companion's heartbeat is lost, another channel is taken: "SDR: companion now
    on MAVLink channel n".
- TX stop on disarm or failsafe:
  - It goes to the vehicle's own sysid, component 191, on that channel.
  - It is repeated every 300 ms until COMMAND_ACK, 10 times at most.
  - Texts: "SDR: TX stop acknowledged (disarm)", "SDR: TX stop NOT acknowledged (...)", or
    "SDR: TX stop refused (...)".
- New texts:
  - "SDR: SDR lost (device absent | no status)" and "SDR: SDR restored";
  - "SDR: companion link lost";
  - "SDR: dialect XXXXXXXX, autopilot 7EFB3788" on a dialect mismatch. The LB5900 is then not
    switched off.
- Log caps against a flooding companion:
  - SDRM at most 100 Hz;
  - SDRS and SDCC at most 2 Hz;
  - SDRC and SDRQ at most 5 Hz.

  SDCC gains the companion's dialect hash (DH). RFSR, SDRC and SDRQ are written again at log
  start and at arming, so every flight log has them.

### LB5900

- The sensor is configured from the I2C thread with a non-blocking state machine. Boot no longer
  waits for it.
- Configuration is retried every second. The sensor is configured again after 2 s without a
  reading.
- Texts:
  - "LB5900: OK" (or "found" when it appears late);
  - "LB5900: no readings, reconnecting", then "LB5900: reconnected";
  - "LB5900: not found (I2C bus b, address a)";
  - "LB5900: no I2C bus b".
- Readings outside −120..+40 dBm are rejected.
- LB59 log:
  - written once per new reading, with a counter `Cnt`;
  - a `Hth 0` row once a second while the sensor is unhealthy.
- Pre-arm "LB5900: no readings" with `USR_SDR_OPT` bit 3.

### GPS

- NAV-HPPOSECEF no longer counts toward "GPS configured". A u-blox that refuses it (e.g. an M8)
  can still arm; the ECEF log and the companion's ECEF stream are then empty.

## 4. Link budget and compatibility (SITL, 57600 baud emulated)

Setup:

- Real vehicle parameter set (MAV1-4 stream rates all 0), sysid 2, disarmed.
- Scripted companion at 30 Hz on SERIAL0, fake T7 on SERIAL2.
- Ludifly's real `RFSonde` class (old or new source tree) on SERIAL1, behind a 5760 B/s proxy.
- RF rate 30 Hz. Windows of 60 s, then 30 s after a second GCS on the same link sends
  REQUEST_DATA_STREAM ALL at 4 Hz (as the Herelink's QGC can).

| Firmware | Ludifly | Link load | RF received / sent | After ALL=4: RF, ATTITUDE |
|---|---|---|---|---|
| old | old | 100 % | 25.4 / 28.0 Hz | 4.0 Hz, 4.0 Hz |
| old | new | 53 % | 28.0 / 28.2 Hz | 4.0 Hz, 10.0 Hz |
| new | old | 100 % | 27.4 / 28.5 Hz | 28.6 Hz, 4.0 Hz |
| new | new | 53 % | 28.7 / 28.7 Hz | 27.2 Hz, 9.6 Hz |

- **Old Ludifly.** Stream groups bring messages it never reads (AHRS2, SIMSTATE, RC_CHANNELS,
  ESC_TELEMETRY, LOCAL_POSITION_NED at 4-10 Hz). That filled the link: about 91 % on a real Cube,
  where SIMSTATE does not exist.
- **New Ludifly.** The link carries 24 message types instead of 38.
- **Old firmware.** Any GCS that requests all streams drops the RF data to that GCS's rate.
- **Compatibility:** every combination works.
  - The RF rate is always set "by message interval".
  - The new firmware keeps the RF rate when Mission Planner or QGC asks for streams.
  - Old Ludifly does not ask again for what a second GCS changed. New Ludifly does, for its own
    telemetry.
- **Companion end-to-end** (real RFSonde_Computer `main` from source, simulated SDR, SITL -I1,
  sysid 2): all 11 phases pass.
  - Phases: boot and dialect hash, ARRC_SENSOR_RAW at 10 Hz, config set and echo, catalogues,
    flight with recording, HDF5, engine kill, power command, scripts, SITL restart, .BIN contents.
  - The companion logged the autopilot's "SDR: TX stop acknowledged (disarm)". Finding M8 is
    fixed against the real companion code, with no change to RFSonde_Computer.

## 5. Verification done (SITL)

Firmware harness `/var/tmp/claude-revamp/tests`, results in `/var/tmp/claude-revamp/runs/final`.
All passed on the final build:

- **t01 stream group:**
  - ALL and EXTRA3 do not touch the RF data;
  - stream id 200 sets both RF messages;
  - `MAV2_ARRC` persists across a reboot;
  - no RF data on the companion port.
- **t02 rate caps:** the RF data are capped at the requested rate, with no duplicates. The
  companion's nav streams keep their rates during a parameter download.
- **t03a, t03b LB5900:**
  - recovery from a reset and a replug;
  - out-of-range readings rejected;
  - NaN at 1 Hz while unhealthy;
  - LB59 counter; FAST mode capped at 30 Hz.
- **t04 gimbal (GMODE 0, 2, 3, invalid 4, slew 0):**
  - GMODE 0 is equal to stock;
  - the invalid mode falls back to 0 and fails pre-arm;
  - no steps above the slew limit;
  - NaN attitude rejected.
- **t05 MNT log:** desired and actual angles in the same convention, for the servo and MAVLink
  gimbals.
- **t06 alignment from the LB5900 and from the SDR:**
  - the true offset is found;
  - LOW clears;
  - a lost source restores the saved offset;
  - MIDDLE does nothing.

  Noisy-sensor runs (t06b, true offset 12 deg, saved offset 5 deg):
  - 6 dB noise: fit 83.5 ± 63.4 deg, R 0.085, rejected as too uncertain; the offset stays at 5.
  - 1 dB noise: accepted at ±0.87 deg, 0.46 deg from the truth.
- **t07 companion link:**
  - dialect hash, binding, TX stop acknowledged;
  - 10 tries when not acknowledged;
  - SDR lost and restored;
  - spoof ignored, then rebinding;
  - log caps;
  - configs present in the logs started at arming.
- **t08 MAVLink1:** a gimbal on a MAVLink1 port gives the warning and the pre-arm failure. Nothing
  happens with MAVLink2.
- **t09 Ludifly link:** section 4.
- **Regression against stock ArduPilot 4.7.2** (mission, failsafes, GPS loss, fence, mount, task
  timing, boot). Results in `/var/tmp/claude-revamp/runs/regr`.
  - Matches stock: arming times, modes, failsafe reactions, landing error, attitude errors,
    scheduler load (PM).
  - Boot to ready: 2.44 s, as stock. The old firmware took 2.69 s (blocking LB5900 init).
  - Mount scenario (GMODE 0): servo outputs for an ROI below and near the vehicle now equal stock.
    Scheduler MaxT is 4998 µs as stock (the old firmware: 8330 µs).
  - Remaining differences, all known:
    - SmartRTL, PosHold, Brake and Sport are compiled out (H2, so FENCE_ACTION 4 lands instead of
      braking);
    - the ROI is kept after Auto (L17).
- **Ludifly:** 625 tests pass.
- **param_parse:** passes for Copter, Plane, Rover and Sub.

## 6. Hardware test plan

Bench first, props off. Use a fresh log for each item, and keep Ludifly's tlog. The tlogs and
`.BIN` files are enough to check every item.

1. **Flash and boot.** Flash `arducopter.apj`.
   - Check `MAV1..4_ARRC` = 0, `MNT1_ARRC_SLEW` = 60, `MNT1_DEFLT_MODE` = 1.
   - Messages to expect: "LB5900: OK" (or "not found" if none is fitted). With the companion up:
     "RF source: SDR (LB5900 disabled)".
   - There must be no "SERIALn is MAVLink1" pre-arm.
2. **Herelink link, Ludifly revamp.**
   - The RF Data Readout says "30 Hz by message interval".
   - The LINK chip is not FULL, and the gimbal chips are not STALE.
   - Then open the Herelink's own QGC screen. The RF rate in Ludifly must stay at 30 Hz.
   - Repeat with the old Ludifly exe once to compare.
3. **Companion over USB.**
   - Arm and disarm: "SDR: TX stop acknowledged (disarm)".
   - Unplug the companion USB: "SDR: companion link lost". Replug: the link comes back without
     a reboot.
   - Stop the SDR (or unplug the bladeRF): "SDR: SDR lost (...)", then "SDR restored".
4. **LB5900, if fitted.**
   - Readings in Ludifly.
   - Unplug the sensor: "no readings, reconnecting", and a gap in the plot. Replug:
     "reconnected".
   - With `USR_SDR_OPT` bit 3 and the sensor unplugged, arming is refused ("LB5900: no
     readings").
5. **Gremsy T7.**
   - Boot with `MNT1_DEFLT_MODE 1`: the motors stay on.
   - GMODE 0, ROI at an AUT: points like stock.
   - GMODE 2 and 3: walk the vehicle around the AUT. The motion is smooth at the HPOL/VPOL
     boundaries.
   - ARGM has `MM` 4 (GPS point) and flag bit 3 while slewing.
   - Pre-arm checks pass. In the T7 logs, look for a NaN attitude.
6. **Alignment with the SDR as source.** Set an aux switch to option 47 and point at a
   transmitter. Expect the texts of section 3.
   - Accepted: `MNT1_ARRC_POFF` changes.
   - Rejected or stopped: it comes back to its old value.
7. **MAVLink1 check.** Set `SERIAL2_PROTOCOL 1` (the T7 port) and reboot: pre-arm "SERIAL2 is
   MAVLink1: set SERIAL2_PROTOCOL 2". Set it back to 2.
8. **Task timing.** With `SCHED_OPTIONS 1` and a 5-minute log with the companion streaming and
   the LB5900 polling:
   - PM.NLon and PM.MaxT are no worse than with ArduPilot-4.7-ARRC;
   - `user_LB5900_logger` and `user_ARRC_SDR_logger` do not overrun.
9. **Logs.** In a flight log started at arming:
   - RFSR, SDRC and SDRQ are present;
   - LB59 `Cnt` increases by 1 at the default rate;
   - no SDRM gaps beyond the companion's own.
10. **GPS.** The GPS reaches 3D and the ECEF log is filled. On a u-blox that refuses
    HPPOSECEF, arming still works.
11. **Short hover, then a survey flight** with the RF source, the gimbal in GMODE 2 or 3 and
    Ludifly on the Herelink. Compare the RF sample rate in the HDF5 and in Ludifly with the
    companion's measurement rate.

## 7. Still open (not in this work)

- **H2:** FLTMODE2 = PosHold, which is compiled out (you asked to leave it).
- **Configuration items C1-C5** from the review: BATT_FS_CRT_ACT, MAV3_OPTIONS saved, LGR/RC7.
  These are for you to apply on the Cube.
- **Doc fixes inside the dialect XML** (L23: SDR_CONTROL param2 wording, phase reference). They
  wait for the next dialect change.
- **L24:** the ARRC `copter.parm` (left as it is).
- **O1:** the ODID embedded defaults.
- **The companion side of M8** (pinning its sysid). The firmware side is done: TX stop goes to the
  vehicle's sysid and is acknowledged.
