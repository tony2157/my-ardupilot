# ArduPilot-4.7-ARRC safety review (2026-10-22/23)

Review of the custom code on branch `ArduPilot-4.7-ARRC` (HEAD `9afcd1ca49`, 63 commits on top of upstream
`Copter-4.7.2` `fb77356789`) for the CubePilot Orange+ (`CubeOrangePlus-ODID`). Scope, as requested: Gremsy/ARRC
gimbal, user code, companion-computer connection, custom MAVLink dialect, SDR/Ludifly data storage and streaming.
No code was changed. This file is untracked.

Time box: 3 h (23:34 to 02:34 CDT). Everything below was run or read inside that window. Hardware was not
touched (the LattePanda was off the network); SITL timing does not represent the Cube.

## 1. Bottom line

- **No critical defect was found in the custom code.**
- With stock parameters, the ARRC build flies the same as stock 4.7.2 in SITL. Takeoff, Loiter, an Auto
  mission with ROI, RTL, landing, and the RC-loss, GCS-loss, battery and GPS/EKF failsafes all give the same
  modes and timings.
- A dead, flooding or rebooting companion never changes the flight mode. It only prints warnings. It can block
  arming only through the optional SDR pre-arm check (USR_SDR_OPT bit 2), which is off on your Cube.
- The Cube firmware builds with 0 warnings and 251 KB of flash free (stock: 244 KB).
- **Two items should be fixed before the next flight. Both can be fixed with a parameter change today:**
  - **H1:** MNT1_DEFLT_MODE 0 (Retract) makes the autopilot send RETRACT to the Gremsy T7 at 50 Hz at boot and
    after every ROI-none. On the T7 that means motors off. This is stock code made dangerous by the setting.
  - **H2:** FLTMODE2 = 16 (PosHold) on the real Cube, but this firmware compiles PosHold out (custom
    `APM_Config.h`), so switch slot 2 does nothing.
  - Also check **C1: BATT_FS_CRT_ACT 0**. A critical battery only sends a message (stock behaviour, your
    setting).
- **Most medium findings are about data integrity, not flight control:**
  - LB5900 power streamed as 0 dBm with no health flag;
  - silent LB5900 init failure;
  - HPOL/VPOL gimbal command jumps at geometric boundaries;
  - an invalid MNT1_ARRC_GMODE silently freezes the gimbal;
  - polarization alignment unusable when the SDR is the source;
  - the Herelink link budget and dropped routed companion statuses;
  - MAVLink1 ports drop every ARRC message;
  - the autopilot's failsafe TX stop can be addressed to sysid 1 and dropped by the real companion (M8). The
    companion's own interlocks still stop TX on disarm or link loss.

## 2. Scope and method

| Area | Custom code reviewed |
|---|---|
| Gimbal | `libraries/AP_Mount/*` (Backend +343, Params, MAVLink, Servo, AP_Mount), `SRV_Channel` float angles, `ArduCopter/mode_auto.cpp` |
| User code | `ArduCopter/UserCode.cpp`, `UserParameters.*`, `arrc_polfit.h`, `APM_Config.h`, `Copter.cpp/.h`, `sensors.cpp`, `system.cpp`, `AP_Arming_Copter.cpp`, `GCS_MAVLink_Copter.cpp`, `libraries/AP_ARRC_LB5900/*` |
| Companion link | `libraries/AP_ARRC_SDR/*`, `GCS_Common.cpp`, `ap_message.h`, `GCS_MAVLink_Parameters.cpp`, routing |
| Streams / storage | `AP_ARRC_SDR_Nav.cpp`, AP_GPS HPPOSECEF/ECEF (u-blox + DroneCAN), DSDL 20008, `AP_Logger` ids, `Tools/AP_Periph/gps.cpp`, companion `ap_record.py` contract |
| Dialect | `modules/mavlink` `b2f9a70f` (fork branch `ArduPilot-4.7-ARRC-SDR`): `arrc.xml`, `ardupilotmega.xml` (+CASS 227) |

What was done:

- **Isolated copies.** Builds ran in copies under `/var/tmp/claude-ap-review/`: `arrc`, `upstream` (stock
  4.7.2 + stock mavlink), `arrc-tests` and `upstream-tests`. Your working tree was not touched.
- **Builds.** SITL, CubeOrangePlus-ODID and CubeOrangePlus for ARRC; SITL and CubeOrangePlus-ODID for stock.
  Two extra variants: `AP_ARRC_SDR_ENABLED=0` and `HAL_MOUNT_ENABLED=0`.
- **Unit tests and parameter docs.** ArduPilot unit tests (`waf tests`) on both trees, and `param_parse.py`.
- **Three review rounds** (36 + 12 + 2 agents, no agent errors). Round 3 ran the real companion with the real
  parameters and fact-checked this report. Its corrections are applied.
  - five static reviewers, one per area;
  - four SITL test agents on separate instances (never instance 0): gimbal with a fake MAVLink2 T7, companion
    end to end with the real `RFSonde_Computer` and its simulated SDR, user code / modes / failsafes, and an
    ARRC-vs-stock regression;
  - every medium or worse finding re-checked by one or two independent verifiers that tried to refute it
    against stock 4.7.2;
  - a completeness critic;
  - in round 2, an integrated run of the real flight configuration and a cross-check of the real parameters.
- **Real parameters.** They were taken from the companion's HDF5 recordings
  (`Desktop/LudiSDR/rfsonde-pc/20261016/1792122086/rfs_1792122086_0000.h5`, 1150 parameters; copy in
  `/var/tmp/claude-ap-review/work/main-params/real_params_20261016.parm`). Changes made since 10-16 (from
  bench notes): SERIAL2_PROTOCOL 2 / 115200, MNT1_DEFLT_MODE 0, MAV3_OPTIONS 2.
- **Evidence.** Every test result, harness and log is in `/var/tmp/claude-ap-review/work/<agent>/`. The
  machine-readable results of both rounds (every finding with its verifier reasoning) are in
  `/var/tmp/claude-ap-review/results/` (`wf_result.json`, `wf2_result.json`, `r1_detail.txt`, `r2_detail.txt`).

## 3. Findings

Severity legend:

- **High:** wrong flight-mode or gimbal behaviour in flight.
- **Medium:** wrong or lost primary data, feature failure, degraded link.
- **Low:** robustness, docs, maintainability.

"Custom" = introduced by the ARRC code or build configuration; "Stock" = upstream 4.7.2 behaviour that the ARRC
configuration exposes. "Verified" = a separate verifier confirmed it.

| ID | Sev | Area | Finding | Origin | Status |
|---|---|---|---|---|---|
| H1 | High | Gimbal | MNT1_DEFLT_MODE 0 makes the autopilot send RETRACT to the T7 at 50 Hz at boot and after every ROI-none (T7 motors off) | Stock code + config | SITL, both builds; 4 verifiers |
| H2 | High | Modes | FLTMODE2 = 16 (PosHold) but PosHold is compiled out: switch slot 2 is dead; from Auto the vehicle keeps flying the mission | Custom build (APM_Config.h) + real param | SITL with real params, ARRC vs stock; 2 verifiers |
| M1 | Medium | User code | LB5900 power streamed and logged with no health or freshness: 0 dBm when absent, stale when lost | Custom | SITL; 3 verifiers |
| M2 | Medium | User code | LB5900 init failure is silent and permanent (no retry, no text), and init blocks boot | Custom | Code + SITL boot timing; verified |
| M3 | Medium | Gimbal | HPOL/VPOL commands jump 80-180 deg at the AUT plane, over the AUT, and at the slope switch; no slew limit | Custom | Replica + SITL; verified |
| M4 | Medium | Gimbal | Invalid MNT1_ARRC_GMODE silently freezes the gimbal; the 10 m hold keeps stale or placeholder angles | Custom | SITL; 2 verifiers |
| M5 | Medium | User code | Polarization alignment needs the LB5900, so it is unusable once the SDR is the source; abort saves POFF = 0 | Custom | Code + SITL; verified |
| M6 | Medium | Link | RF messages 227/228 sit in STREAM_EXTRA3. Ludifly's fallback (REQUEST_DATA_STREAM EXTRA3 at the RF rate) raises 8-9 stock messages to the RF rate (6x bytes) and starves routed companion status. Routed status is dropped whenever the 57600 link is full (stock). | Custom (firmware + Ludifly) + stock routing | SITL with a 57600 proxy; verified |
| M7 | Medium | Link | A MAVLink1 port silently drops every ARRC/gimbal-v2 message the autopilot sends; ARRC_SENSOR_RAW is truncated | Stock mechanism; custom design depends on MAVLink2 | C harness; verified |
| M8 | Medium | Companion | The autopilot's TX stop targets the first 191 route of any sysid. The real companion heartbeats as sysid 1 before adopting sysid 2, so a failsafe TX stop went to 1/191, was dropped, and was still reported as "sent". | Custom (firmware + companion) | Reproduced with the real companion in SITL (round 3) |
| C1 | Medium | Config | BATT_FS_CRT_ACT 0: a critical battery (20.4 V) only sends a message; after the low-battery RTL nothing forces a landing | Stock + config | SITL with real params; verified |
| C2-C5 | Low | Config | Saved MAV3_OPTIONS 2 needs a reboot; RC7 landing gear retracts while disarmed; IMU nav stream only 25 Hz; no RC switch assigned to the alignment | Mostly stock + config | SITL / code |
| L1-L25 | Low | various | See the table at the end of section 4 | Custom | Mixed: tested, verified or static |

## 4. Finding details

### H1. Gremsy T7 is commanded RETRACT (motors off) at boot and after every ROI-none

**Classification:** stock code, made dangerous by the configuration. Confirmed in SITL on both builds by 4
verifiers.

- **Mechanism.** The AP_Mount_MAVLink backend supports RETRACTED natively
  (`libraries/AP_Mount/AP_Mount_MAVLink.h:44-48`). In MNT1 mode RETRACT it sends GIMBAL_DEVICE_SET_ATTITUDE with
  a NaN quaternion and `GIMBAL_DEVICE_FLAGS_RETRACT` at 50 Hz (`AP_Mount_MAVLink.cpp:229-246`). Gremsy firmware
  7.7.1 and later turns its motors off on that flag.
- **When it happens.** The mount enters MNT1_DEFLT_MODE in each of these cases:
  - at boot (`AP_Mount.cpp:204-206`);
  - on `DO_SET_ROI_NONE` or a DO_SET_ROI with a zero location, and on a mission ROI-none item, when the mount
    is in GPS_POINT (ROI) mode. That is the case after Ludifly "Point here" or a mission ROI
    (`AP_Mount_Backend.cpp:409-413`, via `autoyaw.cpp:174-180`).
- **What ARRC changed.** ARRC removed only the Auto-exit path (`mode_auto.cpp:77-78`).
- **SITL evidence** (`work/sitl-gimbal/analysis_s12_arrc.txt`, `analysis_s12_stock.txt`):
  - `first_set_attitude rate=49.9Hz flags={1: 150} nan_q=150`
  - after ROI_NONE: `flags={28: 1, 1: 283}`
  - The stock build sends the same.
- **Scenario.** In flight, while tracking, Ludifly or a mission clears the ROI. The T7 goes limp until the next
  ROI or mount-mode command: the probe swings, the CG shifts and the RF data is lost. After every boot the T7 is limp until
  the first ROI.
- **Fix (configuration, now):** set **MNT1_DEFLT_MODE 1 (Neutral)**. That was the value on 10-16. The MAVLink
  backend has no native Neutral, so it sends the MNT1_NEUTRAL angles as a normal attitude with the motors on.
  Mode 3 (RC targeting) also works.
- **Optional code fixes:**
  - in the ARRC build, send the MNT1_RETRACT angles instead of the RETRACT flag (drop RETRACTED from
    `natively_supported_mount_target_types()`), or refuse Retract while armed;
  - a pre-arm warning for MNT1_TYPE 6 with DEFLT_MODE 0.

**Round-2 integrated SITL run with the real parameters** (`work/r2-integrated/analysis_flight.txt`):

- Boot: RETRACT with a NaN quaternion at 50 Hz from 11 s after boot (1391 packets, identical on both builds).
- After "Point here": flags 28, finite, through Guided/Auto/AltHold/Loiter/RTL/landing.
- After `DO_SET_ROI_NONE`: RETRACT at 50 Hz again.
- Stock 4.7.2 with the same parameters also retracts on Auto exit and during the failsafe RTL. ARRC avoids
  only that one path.

**Round-3 in-flight evidence** (`work/r3-realcc/analysis_r3.txt`):

- An Auto mission flown at 20 m: DO_SET_ROI_LOCATION, waypoints, then a DO_SET_ROI_NONE item.
- With MNT1_DEFLT_MODE 0, the ROI-none item made the autopilot send RETRACT with a NaN quaternion at 50 Hz
  (1697 of 1697 messages) until landing, including the GCS-failsafe RTL. Stock 4.7.2 is identical.
- With **MNT1_DEFLT_MODE 1**, the same mission gives flags 0 with finite neutral angles (motors on) and no
  RETRACT at any point. The fix is confirmed in SITL.

### H2. FLTMODE2 = 16 (PosHold) on the real Cube, but this firmware compiles PosHold out: switch slot 2 is dead

**Classification:** custom build configuration plus a real parameter. Confirmed in SITL (ARRC vs stock); 2
verifiers rated it high.

- **Configuration.** `ArduCopter/APM_Config.h:8-21` compiles out ACRO, BRAKE, DRIFT, FLIP, FOLLOW,
  GUIDED_NOGPS, **POSHOLD**, SMARTRTL, SPORT and THROW (commit 6d6ca1b994). The commit gives no reason
  ("Remove unnessesary features"). Flash is not a constraint: this build has 251 KB free, and stock 4.7.2 with
  all these modes has 244 KB free. The real parameters (all four HDF5 snapshots) have FLTMODE_CH 5 and FLTMODE1-6 =
  6 (RTL), **16 (PosHold)**, 2, 3, 5, 2.
- **What happens on slot 2.** RC5 at PWM 1231-1360 calls `Copter::set_mode(16)`. `mode_from_mode_num()` returns
  null and `notify_no_such_mode()` runs (`AP_Vehicle.cpp:940-944`). That only sends the WARNING "No such mode 16"
  and an ERR log record, with no failure tone. The mode is unchanged and the switch is not retried
  (`RC_Channel_Copter.cpp:24-26`).
- **SITL** (`work/r2-integrated/run_arrc_flight.log`, `analysis_latency.txt`):
  - In Auto, slot 2 leaves the vehicle in AUTO, still flying the mission.
  - In Loiter, it stays in Loiter.
  - Stock 4.7.2 with the same parameters enters PosHold in 0.65-0.73 s.
  - The warning reached the GCS 0.4 s after the first slot-2 selection and 1.8-3.1 s after the second, in
    three runs over the 57600 proxy. The longer delays come from SITL-only link load (see M6) and are not
    expected on the vehicle.
- **Scenario.** The pilot uses slot 2 to take over from Auto or an RTL near an obstacle and gets no control
  back. If the switch already sits at slot 2 when RC comes back after a failsafe, moving the sticks does nothing
  either.
- **Reachability.** Check whether the Herelink/transmitter can reach slot 2 (RC5 range 1100-1900, trim 1420).
- **Failsafes and fence (verified safe).** Every failsafe and fence path that asks for a compiled-out mode
  falls back safely:
  - FS_THR_ENABLE 4/5: RTL/Land;
  - BATT_FS 3/4: RTL/Land;
  - FS_GCS_ENABLE 7: Land;
  - FENCE_ACTION 4: Land at the fence instead of Brake.

  GCS requests and RCx_OPTION 33/42 are refused ("No such mode N", "Invalid channel option"). No pre-arm
  check flags a FLTMODEn or action that names a missing mode.
- **Fix:**
  - **Now (configuration):** set FLTMODE2 to a compiled-in mode (5 Loiter or 2 AltHold).
  - **Code:** re-enable MODE_POSHOLD (and optionally BRAKE and SMARTRTL, so that a future failsafe, fence or
    RCx_OPTION setting that names them behaves as documented; none of the current settings does), or add a
    Copter pre-arm check that every FLTMODEn, mode RCx_OPTION, FENCE_ACTION and FS action resolves to a
    compiled-in mode.

### M1. LB5900 power is streamed and logged with no health or freshness (0 dBm when absent, stale when lost)

**Classification:** custom. Verified by 3 agents.

- **What the code does.** `Copter::send_arrc_gcs_message` (`ArduCopter/GCS_MAVLink_Copter.cpp:254-273`) sends
  `ARRC_LB5900.power_measure()` in CASS_SENSOR_RAW (datatype 4), stamped with the send time. It never checks
  `healthy()` or whether there is a new sample. The gate `arrc_lb5900_streaming()` (`UserCode.cpp:173-176`)
  checks only `enabled()` and that USR_RF_SRC is not SDR only; it never checks health or a new sample.
- **Absent sensor.** A failed init leaves `_enabled = true` and `_power = 0`, so the vehicle streams 0.0 dBm.
- **Logging.** `user_LB5900_logger` writes LB59 at 50 Hz with Pwr 0.0 / Hth 0, and LB59 has no sample counter.
- **SITL evidence:**
  - CASS at 20 Hz, all values `[0,0,0,0,0]`, with USR_RF_SRC 0, 1 and 3 (`work/sitl-usercode/run_s1_*`);
  - 678 zero CASS messages in a stock-parameter mission (`work/sitl-regression/arrc_mission/result.json`).
- **Ludifly** queues every CASS value as a real LB5900 point (`RFSonde.py` `__read_cass_sensor`).
- **Fix:**
  - send CASS only when healthy and `sample_count()` has changed (the same pattern as `send_sensor_raw`), or
    send NaN;
  - put health and sample count in `values[1..4]`;
  - log LB59 only on a new sample, with a counter.

### M2. LB5900 init failure is silent and permanent; init blocks boot

**Classification:** custom. Verified.

- **Silent and permanent.** `init_ARRC_lb5900()` (`ArduCopter/sensors.cpp:3-16`) ignores the result of
  `ARRC_LB5900.init()`. After 100 failed `configSensor()` tries, `init()` returns before
  `register_periodic_callback` (`AP_ARRC_LB5900.cpp:49-72`), so the sensor is never polled again until reboot.
  There is no GCS text and no pre-arm check. A sensor that powers up after the Cube, or that resets in flight,
  is never reconfigured.
- **Blocks boot.** Init runs on the main thread in `init_ardupilot` (`system.cpp:73-74`), with `delay(50)`, a
  blocking bus semaphore, 5 retries per transfer and 3 ms delays.
  - In SITL it adds 0.25 s to boot ("Ready" at 2.69 s vs 2.44 s).
  - On the Cube a stuck SDA could stretch this to seconds.
  - It cannot cause a watchdog reset, because the watchdog starts after `setup()`.
- **Fix:** do the configuration as a non-blocking state machine in the I2C periodic callback, retried after
  failures. Add a GCS text when the sensor is missing while it is the active source, and an optional pre-arm
  check under the Camera bit.

### M3. HPOL/VPOL gimbal commands jump 80-180 deg at geometric boundaries (no hysteresis, no slew limit)

**Classification:** custom. Verified (medium / low).

- **Mechanism.** `AP_Mount_Backend.cpp:1248` (`if (c.z < 0) { b = -b; c = -c; }`) chooses between the two
  polarization solutions by sign. Combined with the ±35 deg roll clamp (`:1121`) and the elevation/slope mode
  switch (`:1187`), the command can step in a single update.
- **Measured steps:**

  | Case | Step | Source |
  |---|---|---|
  | HPOL, crossing the AUT aperture plane (x = 0) | 79.8 deg within 1.7 cm (roll −35 → +35, yaw ~41 deg) | Python replica, `work/static-gimbal/jump_refine.out` |
  | HPOL, crossing the plane in SITL with the real binary | roll flips −35/+35 or yaw 0.5 → 179.5 | `work/sitl-gimbal/analysis_s2c_arrc.txt` |
  | Directly over the AUT | ~180 deg | replica |
  | VPOL slope switch | 37.6 deg | replica |

- **Normal flight and noise:**
  - In normal SITL flight the largest step was 0.09 deg (the same as stock).
  - A 15 s chatter test on the boundary with GPS noise did not reproduce back-and-forth flipping.
- **Impact.** The T7 slews at full rate (reaction torque on the airframe), the data during the slew is invalid,
  and the probe cable can twist.
- **Fix:**
  - pick the solution closest to the previous command;
  - add hysteresis on the c.z sign and on the slope switch;
  - rate-limit the target (for example 60 deg/s) before `send_target_angles`;
  - flag big steps in ARGM.

### M4. An invalid MNT1_ARRC_GMODE silently freezes the gimbal; the 10 m "too close" hold keeps a stale target

**Classification:** custom. Verified twice (medium).

- **Invalid GMODE.** `get_ARRC_gimbal_mode()` returns UNKNOWN unless the float parameter is exactly 0-3. The
  `default:` branch (`AP_Mount_Backend.cpp:1271`) then returns true without new angles. The same happens when the
  probe is within 10 m of the target (`:1177`).
- **Result.** The caller re-sends the previous `mnt_target.angle_rad`. After a Retract that is the identity
  "body forward, unlocked" placeholder.
- **SITL evidence:** GMODE 4 or 1.5 gives `flags [0] (0,0,0)` for every target, with no STATUSTEXT
  (`work/sitl-gimbal/analysis_s2b_arrc.txt`). The servo gimbal stays at the previous target's angles
  (`analysis_s4_arrc.txt`).
- **Also affected.** The 10 m hold applies to GMODE 0 and to HOME/SYSID targets too.
- **Possible trigger.** The parameter shares slot 13 with stock `MNT1_LEAD_PTCH`, so it can arrive with a
  value such as 0.05.
- **Fix:**
  - treat UNKNOWN as GMODE 0 and warn (rate-limited);
  - add a pre-arm check that GMODE is an integer 0-3;
  - apply the 10 m hold only in GMODE 1-3, and only once a valid solution exists.

### M5. Polarization alignment cannot run with the SDR as the source, and an abort overwrites the saved offset

**Classification:** custom. Verified.

- **LB5900 only.** The aux-switch alignment (`UserCode.cpp:57-154, 281-318`) needs `ARRC_LB5900.healthy()`. In
  USR_RF_SRC 0 the first SDR detection calls `set_enabled(false)`, which forces healthy false
  (`UserCode.cpp:213-215`, `AP_ARRC_LB5900.h:137`). So with the real setup the alignment is always refused:
  "Gimbal alignment needs a healthy power sensor" (seen in SITL).
- **Abort wipes the offset.** Any abort or failed fit calls `set_ARRC_pol_offset(0, true)` (`set_and_save`),
  which wipes a previously calibrated MNT1_ARRC_POFF. An example is the SDR being detected mid-scan.
- **Other details:**
  - the handler ignores the switch position (any change toggles start/clear);
  - USER2 and USER3 are silent no-ops;
  - R >= 0.7 does not bound the offset error: p95 3-7 deg at 0.25 dB noise
    (`work/static-usercode/polfit/results_noise.txt`).
- **Fix:**
  - feed the fit from the active RF source (SDR `pwr_c`);
  - on abort, restore the value saved at the start of the scan;
  - act on HIGH/LOW only;
  - accept the fit on the parameter uncertainty, not on R.
- **Current exposure.** RC7_OPTION is 29, so no switch is assigned to the alignment today.

### M6. Herelink (57600) link budget: the RF stream sits in STREAM_EXTRA3, and routed companion status is dropped when the link is full

**Classification:** custom (firmware + Ludifly) on top of stock routing. Verified.

- **Firmware side.** MSG_ARRC_SDR (CASS 227) has been in `STREAM_EXTRA3_msgs` since c88ae3613f (2023), and
  715dad5079 added MSG_ARRC_SENSOR (228) next to it (`GCS_MAVLink_Parameters.cpp:374-377`).
- **Normal path is fine.** Ludifly normally sets the RF rate with SET_MESSAGE_INTERVAL on 228/227
  (`RFSonde.py:2956-2992`, ACK ACCEPTED). The stock EXTRA3 members then stay at 1 Hz (205-269 B/s).
- **Fallback path.** After 3 un-ACKed attempts, Ludifly falls back to REQUEST_DATA_STREAM EXTRA3 at the RF rate
  (`mavlink_rate_manager.py:413-419`). That raises AHRS, WIND, SYSTEM_TIME, TERRAIN_REPORT, EKF_STATUS_REPORT,
  VIBRATION, GIMBAL_DEVICE_ATTITUDE_STATUS, DISTANCE_SENSOR and BATTERY_STATUS to the RF rate:
  - 1.41-1.44 kB/s, 6x the normal load;
  - ARRC_SENSOR_RAW dropped to 7.6 Hz;
  - routed CC/SDR status fell to 0 Hz in the RF-30 windows (`work/r2-integrated/analysis_links.txt`).

  Nothing lowers EXTRA3 again until a reconnect. Conversely, any later REQUEST_DATA_STREAM ALL/EXTRA3 resets
  227/228 to the group rate.
- **Stock routing** forwards a companion message only if the TX buffer has room at that instant
  (`MAVLink_routing.cpp:231`), with no queue.
- **Real-hardware numbers.** The 10-22 bench measurement had MAV3_OPTIONS 2 and RF 30: 87.8 % link use,
  statuses at 1.0 Hz, 228 at 27 Hz. The integrated SITL run showed 2-3 s heartbeat latency, lost PARAM_SET
  echoes and failed mission uploads at RF 30. The verifier traced those to SITL-only SIMSTATE/ESC_TELEMETRY
  filling the link, so they are not claimed for the vehicle. Even so, 88 % leaves little headroom for
  STATUSTEXT bursts, parameter writes and mission uploads.
- **Gimbal forwarding.** Booted with MAV3_OPTIONS 0 (the 10-16 value), the fake T7's forwarded chatter took
  about 12 % of the link in SITL (about 700 B/s) and routed SDR status fell to 0.08-0.16 Hz. The real T7
  measured on the bench on 10-19 forwarded 1153 B/s, about 20 %. MAV3_OPTIONS only takes effect after a reboot.
- **Mission uploads.** The round-2 mission-upload failures came from the 57600 emulation. The same mission
  uploads cleanly over a direct link (round 3).
- **Fix:**
  - remove 227/228 from STREAM_EXTRA3 (the interval map stays);
  - in Ludifly, retry the interval method instead of falling back to EXTRA3, and reset EXTRA3 to 1 Hz if the
    fallback was used;
  - budget RF on the Herelink at 10-15 Hz (the full-rate data is recorded on the companion);
  - optionally echo the key companion state (SDR detected/lost, TX armed, recording) in the autopilot's own
    downlink instead of relying on forwarding.

### M7. A MAVLink1 serial port silently drops every ARRC message the autopilot sends (ids > 255)

**Classification:** stock mechanism. Known. Verified.

- **Mechanism.** With SERIALn_PROTOCOL 1 the channel is flagged OUT_MAVLINK1 for good
  (`GCS_Common.cpp:228-236`). The generated helper then drops every originated message with an id above 255:
  - the ARRC_NAV_* streams and ARRC_GIMBAL_POINTING;
  - GIMBAL_DEVICE_SET_ATTITUDE (284) to the T7.
- **Partial delivery.** ARRC_SENSOR_RAW (228) is cut to 28 bytes, losing snr, flags and n_avg, so Ludifly shows
  NaN. Routed companion MAVLink2 frames still pass, so the GCS sees CC status while every RF sample is invalid.
- **Evidence:** `work/static-dialect/ctest/arrc_check.log`.
- **Real Cube.** SERIAL2_PROTOCOL was 1 until 10-18 and SERIAL5 (ADSB) is still 1.
- **Fix:** a pre-arm or boot STATUSTEXT when a port that carries the companion, the GCS RF downlink or a MAVLink
  gimbal is MAVLink1. Document "SERIALn_PROTOCOL must be 2" in the parameter docs.

### M8. The autopilot's TX stop can be addressed to the wrong sysid and silently dropped by the real companion

**Classification:** custom (firmware + companion). Found in round 3 with the real RFSonde_Computer; reproduced
in SITL. Depends on start order.

- **Mechanism.** `send_tx_stop()` (`AP_ARRC_SDR.cpp:371-390`) targets the first routing entry with type 18 and
  compid 191, of any sysid (`find_by_mavtype_and_compid`). Before the companion adopts the autopilot's sysid it
  heartbeats as **sysid 1** (`rfsonde_cc/mediator.py:116`, `int(link.get('sysid', 0) or 1)`). If the companion
  starts first, the autopilot learns a 1/191 route, which never expires, and keeps using it.
- **Observed.** The failsafe stop went out as `COMMAND_LONG src=2/1 tgt=1/191 cmd=47000 p1=3 p2=2`. The
  companion (now 2/191) dropped it in `_for_us()`. No ACK was sent, and the autopilot does not check for one.
  The autopilot still printed "SDR: TX stop sent (failsafe)". The companion's TX arm stayed set (flags 1079,
  arm timer still counting down).
- **Comparison.** The same command sent by the GCS to 2/191 disarmed TX immediately. With the autopilot up
  first, the problem did not reproduce. Evidence: `work/r3-realcc/` (item 4 in
  `/var/tmp/claude-ap-review/results/wf3_result.json`).
- **What still protects you.** The companion's own interlocks:
  - TX stops about 1 s after an autopilot disarm (heartbeat armed flag);
  - after 10 s of GCS-link loss (production `gcs_timeout_s 10`);
  - autopilot-link timeout;
  - protected bands.

  So the gap is a **failsafe while still armed** (e.g. RC-loss RTL): TX can stay armed until landing or the
  companion's own timeout. TX is not licensed or used yet (RX only), so today there is no exposure.
- **Other weaknesses** (formerly L2): the stop is one-shot, has no ACK check or retry, consumes the edge even
  when not sent, and is skipped when `detected()` is false.
- **Fix:**
  - **Companion:** pin `link.sysid` to the vehicle's sysid, or never heartbeat before adopting it.
  - **Firmware:** look up the route with `sysid == mavlink_system.sysid` (or use the channel/sysid of the last
    companion heartbeat). Keep a pending-stop flag, resend until the COMMAND_ACK arrives, and print
    "acknowledged" or "NOT acknowledged".
  - **Bench:** check the stop's target sysid after a cold start of both.

### Configuration items on the real Cube (10-16 snapshot plus later bench changes)

| # | Item | Evidence | Recommendation |
|---|---|---|---|
| C0 | MNT1_DEFLT_MODE 0 (H1) and FLTMODE2 16 (H2) | Above | MNT1_DEFLT_MODE 1; FLTMODE2 → 5 or 2 |
| C1 | **BATT_FS_CRT_ACT 0** with BATT_CRT_VOLT 20.4 (6S). After the low-battery RTL (BATT_FS_LOW_ACT 2) the critical level only sends "Battery 1 is critical". Once the pilot leaves the RTL, nothing forces a landing. Stock behaviour, medium. | SITL with real params (RTL at 20.9 V, text only at 20.2 V); verified | BATT_FS_CRT_ACT 1 (Land); optionally BATT_CRT_MAH |
| C2 | MAV3_OPTIONS 2 (gimbal private) only takes effect after a reboot. The 10-16 snapshot still had 0, and with 0 the forwarded T7 chatter takes about 20 % of the Herelink (1153 B/s with the real T7 on 10-19; 12 % with the SITL fake T7). | `GCS_Common.cpp:196-198`; SITL | Keep it in the saved parameter file; reboot after setting it |
| C3 | RC7_OPTION 29 with LGR_ENABLE 1 / SERVO11 29: the landing gear retracts while disarmed on the ground. Stock behaviour. | SITL, both builds | If no retractable gear is fitted: LGR_ENABLE 0, RC7_OPTION 0 |
| C4 | LOG_BITMASK 180222 has no IMU_FAST, so ARRC_NAV_IMU reaches the companion at 25 Hz as point samples of a 23 Hz-filtered gyro (aliasing). LB59 repeats each LB5900 value about 12 times. LOG_DISARMED 0 loses the RFSR/SDRC/SDRQ records (L18). | Code | Decide what the companion needs from IMU data (IMU_FAST, or averaged samples) |
| C5 | No RCx_OPTION is 47, so the polarization alignment (M5) cannot be started today. SERIAL5_PROTOCOL 1 (ADSB) is MAVLink1, which is fine for ADSB but drops anything with an id > 255 (M7). | Params | Assign it only once M5 is fixed |
| C6 | Expected bench pre-arm blockers: "Hardware safety switch" (BRD_SAFETY_DEFLT 1), and ODID "UA_TYPE required in BasicID" when the CubeID module does not supply BasicID. | SITL with real params | None (informational) |

### Low findings (custom unless marked)

| # | Finding | Where | Fix in short |
|---|---|---|---|
| L1 | SDR loss with the companion alive is silent: no autopilot text when SDR_STATUS stops or DEVICE_PRESENT clears. In Auto the LB5900 stays latched off, so RF data ends silently (by design, but unannounced). Tested. | `AP_ARRC_SDR.cpp:319-341`, `UserCode.cpp:213` | Warn on the falling edge of `_detected`. Repeat while armed. Recommend USR_RF_SRC 3 when the LB5900 is fitted. |
| L2 | Promoted to **M8** after round 3 (see above). | — | — |
| L3 | Companion identity (sysid = MAV_SYSID, compid 191) is accepted from any channel. A device on the Herelink can trigger detection, the LB5900 latch, the pre-arm pass and the TX-stop route. Tested. | `AP_ARRC_SDR.cpp:33-36` | Bind the companion to its channel (first heartbeat, or a USR_SDR_PORT parameter). |
| L4 | Nav-stream cap uses the measured bucket spacing. Stock's 4x stream slowdown during a parameter download halves IMU and drops MAG/BARO/gimbal samples at companion reconnect (HDF5 only; .BIN unaffected). A REQUEST_MESSAGE or an interval >= 2 s gives 2 s bursts at full log rate. | `AP_ARRC_SDR_Nav.cpp:51-53, 62-66, 77` | Use `get_ap_message_interval()` for both the cap and the subscription. |
| L5 | An `AP_ARRC_SDR_ENABLED=0` build keeps MSG_ARRC_SENSOR in EXTRA3 with no handler. From the code (not run), it reaches the default case of `try_send_message`: AP_HAL::panic in SITL and "Sending unknown message" DEBUG text on hardware. Not the flown build (it compiles: 1,709,840 B). | `GCS_MAVLink_Parameters.cpp:374-377`, `GCS_Common.cpp:6891-6894` | Guard it with `AP_ARRC_SDR_ENABLED`. |
| L6 | The firmware does not build with `HAL_MOUNT_ENABLED=0`: `UserCode.cpp:65` uses `camera_mount` unguarded (build run here). | `ArduCopter/UserCode.cpp:65` | Add `#if HAL_MOUNT_ENABLED` guards (AGENTS.md rule). |
| L7 | The autopilot never checks ARRC_CC_STATUS.dialect_hash. A future one-sided dialect bump would drop SDR_MEAS on CRC yet still detect the SDR and latch the LB5900. Today all three pins match (b2f9a70f, 0x7EFB3788). | `AP_ARRC_SDR.cpp:125-129` | Compare a build-time hash; warn and do not latch on a mismatch. |
| L8 | LB5900 failure counter is uint8 and wraps: a dead sensor reads healthy for ~0.75 s every 62.7 s (replica). | `AP_ARRC_LB5900.cpp:269`, `.h:209` | Saturate the counter or add a last-success time. |
| L9 | The LB5900 parser accepts any finite number (SCPI sentinels like 9.91E+37, trailing junk). There is no evidence the sensor emits them. | `AP_ARRC_LB5900.cpp:243` | Range-check (e.g. −100..+30 dBm). |
| L10 | `_enabled`/`_healthy` race between the I2C thread and the main thread. `_sem` is never taken by readers. | `AP_ARRC_LB5900.h:137` | `healthy()` returns `_enabled && _healthy`; copy the power and count under the semaphore. |
| L11 | ARRC pointing hard-codes ±35 deg roll and ±75 deg pitch and ignores MNT1_*_MIN/MAX. GMODE 0 is not stock: no nadir (−75 deg), 10 m hold, repeated "pitch limit" text. | `AP_Mount_Backend.cpp:1120-1121, 1275-1289` | Use min(hard limit, MNT1 limits). Keep GMODE 0 stock-equivalent. Nothing downstream clamps the ARRC angles: the T7 receives them as computed. With the T7's reported limits (roll ±45, pitch −99..+45, which override the unsaved MNT1 defaults at runtime), the ±35 roll always fits. Pitch above +45 saturates in the gimbal while ARGM still reports "aligned"; below −75 the ARRC clamp stops a T7 that could reach −90. |
| L12 | A NaN quaternion from the gimbal reaches `constrain_float()` in `get_ARRC_pointing()`, causing INTERNAL_ERROR constraining_nan, which blocks arming until reboot. Static; needs a T7 check of what it sends. | `AP_Mount_Backend.cpp:778` | Reject NaN or non-unit quaternions in `get_measured_attitude_ef()`. |
| L13 | HPOL/VPOL singular geometry (probe on the AUT y axis) returns false, so no command is sent and a Retract can persist. | `AP_Mount_Backend.cpp:1218, 1228` | Fall back to GMODE 0 pointing. |
| L14 | 312/321 mismatches: the servo POI-at-home-alt line of sight is wrong with roll (3-30 deg); the Gremsy MNT log pairs 312 desired with 321 actual angles. | `AP_Mount_Backend.cpp:976, 247` | Use the quaternion directly; log in one convention. |
| L15 | ARGM / ARRC_GIMBAL_POINTING keep the old ARRC geometry for 1 s after the mount leaves ARRC pointing (RC nudge, manual attitude). | `AP_Mount_Backend.cpp:759-764` | Require the mount mode, or clear `_arrc.update_ms` on a mode change. Add the mode to ARGM. |
| L16 | Parameter layout vs stock: MNT1 indices 12/13 (ARRC_AZTH/GMODE) reuse stock LEAD_RLL/PTCH, 14/15 (ARRC_ELEV/ZOFF) reuse stock SYSID_DFLT/DEVID, and SYSID_DFLT/DEVID/OPTIONS moved to 16-18 (stock OPTIONS at 16 becomes SYSID_DFLT). USR index 1 (now LB_FREQ) was SDR_NPULSE in an older ARRC layout. A stock<->ARRC reflash reinterprets values. | `AP_Mount_Params.cpp:145-194`, `UserParameters.cpp:31` | Document "reset MNT1_* when switching firmware"; never reuse indices. |
| L17 | Auto exit no longer resets the mount: a mission ROI persists through RTL, Land and disarm. This is intentional and avoids H1 on Auto exit. Tested. | `mode_auto.cpp:77-78` | Document it in the operator checklist. |
| L18 | RFSR, SDRC and SDRQ are one-shot records, usually written before logging starts (LOG_DISARMED 0), so flight logs miss the RF source and SDR config. Tested. | `UserCode.cpp:224`, `AP_ARRC_SDR.cpp:213-317` | Re-write them at log start and at arming. |
| L19 | No rate cap on companion-driven logging. A 1 kHz flood took the log from 74 to 175 KB/s; harmless in SITL, untested on the Cube SD card. | `AP_ARRC_SDR.cpp:101-317` | Cap SDRM at about 100 Hz and the status/config logs at 1-5 Hz. |
| L20 | GPS/ECEF are polled at the requested interval, not sent per fix: fixes alias away if the requested rate is <= the fix rate (the CC requests 50 Hz, so fine). The constructor relies on static zero-init. | `AP_ARRC_SDR_Nav.cpp:55`, `AP_ARRC_SDR.cpp:28-30` | Send on a new fix; zero all arrays. |
| L21 | The NAV-HPPOSECEF rate bit counts toward "GPS configured" and a refused CFG-MSG is never given up. Matters for the custom Here4 firmware (GNSSPeriph) if an F9 ever refuses it, because the GPS would then never become ARMABLE. | `AP_GPS_UBLOX.cpp:416, 474-477, 532-535` | Treat HPPOSECEF as optional for `is_configured()`. |
| L22 | Custom ids inside upstream numbering: LOG_ECEF shifts later static log ids, ap_message 102-109, DSDL 20008, msgs 227/228 in ArduPilot's own block. All correct today; a future merge risk. | `AP_GPS/LogStructure.h:14`, `ap_message.h` | Append, reserve ranges, keep a single DSDL source. |
| L23 | Docs: USR_SDR_OPT and the arming comment still say "Camera bit of ARMING_CHECK cleared" (4.7: ARMING_SKIPCHK bit 16 set; the code is right). TIMING_CONFIGURATION ranges are wrong. CASS app_datatype 4 is undocumented. MAV_CMD_ARRC_SDR_CONTROL param2 says "0 = all" but means RX only. ARRC_SENSOR_RAW has no phase reference. | `UserParameters.cpp:67`, `AP_Arming_Copter.cpp:58`, `arrc.xml:273, 373`, `ardupilotmega.xml:2103` | Fix the texts (doc-only, hash unchanged); add a `phase_ref` extension. |
| L24 | `Tools/autotest/default_params/copter.parm` was replaced by a 4.6-era ARRC dump with stale names (SYSID_THISMAV, ARMING_CHECK, USR_SDR_*). It changes every SITL/autotest default and is not the real configuration. | `Tools/autotest/default_params/copter.parm` | Restore the stock file; keep a current 4.7 vehicle file under `RFSonde-sim/config/`. |
| L25 | Structure: the LB5900 is compiled into Copter with no feature guard; the `build_options.py` entry has no `extract_features` pattern and sits in the Gimbal block; core `GCS_Common.cpp` includes the custom config header; dead `set_fixed_yaw_angle()` would `set_and_save` MNT1_ARRC_AZTH. | various | Clean-up commits. |
| S1 | Stock: BATT_FS_CRT_ACT 6 (DO_LAND_START or RTL) and 7 (Brake or Land) never run once the low-battery failsafe has fired: both are missing from `_failsafe_priorities`, so they rank below every listed action, including 0. Tested for 7 after a low failsafe. | `Copter.h:668`, `AP_BattMonitor.cpp:942-958` | Use 1 (Land). |
| S2 | Stock: any source on any link can send ARRC commands to the companion (unauthenticated MAVLink). Commands to 2/1 are rejected. | `MAVLink_routing.cpp` | MAVLink2 signing, or a sysid filter in the companion. |
| O1 | Out of scope: the CubeOrangePlus-ODID embedded defaults drop the `@READONLY` locks (DID_OPTIONS, DID_CANDRIVER, AHRS_EKF_TYPE) and default DID_MAVPORT to 1 (Herelink). The ODID MAVLink static-message rotation now advances without sending when SERIAL1 is full. The real Cube has DID_MAVPORT −1. | `hwdef/CubeOrangePlus-ODID/defaults.parm`, `AP_OpenDroneID.cpp` | Decide whether intended; restore the locks if Remote ID compliance requires them. |

## 5. Verified OK

### Build and tests

- Builds:
  - CubeOrangePlus-ODID, CubeOrangePlus and SITL build with 0 warnings.
  - ODID flash use is 1,714,612 B with 251,460 B free; stock 4.7.2 uses 1,722,076 B with 243,996 B free.
  - BSS is +32 B over stock, and the ARRC code does no dynamic allocation.
  - The `AP_ARRC_SDR_ENABLED=0` Cube build compiles.
- Unit tests: 51 of 55 test binaries pass. The 4 that fail (bitmask, math, math_double, rotations; they test
  crashes/aborts, which only works under the waf test runner) fail identically on stock 4.7.2.
- `param_parse.py` runs clean. All USR_* and MNTx_ARRC_* parameters are documented.

### Flight behaviour vs stock (SITL, stock parameters on both builds)

- Takeoff, Loiter, an Auto mission with DO_SET_ROI, RTL and landing match stock (landing error 0.03 vs
  0.04 m), as do attitude error and the EKF flag sequence.
- RC-loss, GCS-loss, battery and GPS/EKF failsafes give the same mode and timing as stock: RTL 0.99 s, RTL
  5.12 s, RTL 10.2 s, Land about 8 s.
- PM NLon/MaxT are identical to stock. SITL timing is only indicative.
- Every failsafe or fence path that asks for a compiled-out mode falls back safely:
  - SmartRTL → RTL → Land;
  - Brake → Land (FENCE_ACTION 4 lands at the fence);
  - a GCS request for a missing mode is refused with "No such mode N".
- With MNT1_TYPE 0, the ARRC mount code is inert: no extra messages, no ARGM, no texts.

### Scheduler and threads

- The ARRC tasks reuse the stock userhook slots (10 Hz/100 µs, 10 Hz/75 µs, 50 Hz/175 µs). Nothing custom is
  in FAST_TASK.
- No custom task calls delay, I2C or printf on the main thread. The only exception is the one-time LB5900 init
  at boot (M2).
- With LOG_BITMASK 180222 the IMU nav stream runs at 25 Hz from the 25 Hz logging task.

### Companion link

- The companion end-to-end harness (real RFSonde_Computer plus simulated SDR) passes 11/11 phases at sysid 2,
  with RF source 0 and with RF source 3.
- The HDF5 nav groups match the .BIN log rates: IMU 24.1, MAG/BARO 9.6, GPS/ECEF 4.87, gimbal 9.57 Hz.
- ECEF numerics and units match across the UBX driver, DroneCAN, MAVLink, the log and the companion.
- Losing the companion in flight only prints a warning: altitude held to ±1 mm in Guided, no failsafe.
- A malformed 1 kHz flood (NaN/Inf/±3.4e38, unterminated strings) caused no crash and no loop-rate change in
  SITL.
- The TX stop reaches the companion on disarm and on RC failsafe. `any_failsafe_triggered()` also covers GCS,
  battery and EKF failsafes.
- The SDR pre-arm (USR_SDR_OPT bit 2) works and can be bypassed with ARMING_SKIPCHK bit 16. It is off with the
  real USR_SDR_OPT 3.
- ARRC MAV_CMDs sent to the autopilot are rejected UNSUPPORTED. The companion's runtime code sends the
  autopilot only REQUEST_MESSAGE, SET_MESSAGE_INTERVAL and read-only parameter/mission requests, so it does not
  change flight state. The link is unsigned, though, so the autopilot would accept any standard command from it
  (S2).
- **Round 3, real companion with the real parameters** (`work/r3-realcc`):
  - detection and the LB5900 latch 11.2 s after boot;
  - recording starts 0.66 s after arming and stops on landing;
  - HDF5 rates: feed 30 Hz, IMU 25, MAG/BARO 10, GPS/ECEF 5, gimbal 10 Hz while ARRC pointing is active;
  - the companion's own disarm interlock stops TX about 1 s after disarm.
- **GCS-loss failsafe with the real FS_GCS_ENABLE 1 / FS_OPTIONS 16, in Auto:** RTL at 5.0 s on both ARRC and
  stock. The companion's heartbeats (same sysid, compid 191) do not mask the GCS failsafe.

### MAVLink dialect

- There is no message, command or enum collision with any local XML, with ArduPilot/mavlink master
  (e03dad40), or with any `ap_upstream` branch.
- mavgen with validation (C and Python) passes. All payloads are ≤ 236 B. The C and Python crc_extra values
  match.
- The firmware headers are byte-identical to a fresh generation, apart from XML_HASH.
- The firmware, companion and Ludifly all pin `b2f9a70f` with dialect hash 0x7EFB3788.
- An anonymous clone of the fork gets the right submodule commit.

### Gimbal

- With no limit hit, the ARRC pointing and polarization maths is exact (1e-6 deg in a replica; ≤ 0.03 deg
  decoded from the SITL quaternions).
- No NaN is ever sent in a locked command.
- In-flight pointing matches stock (< 1 deg). The 50 Hz rate is unchanged.
- The servo PWM stays within the limits. Motor outputs are identical to stock.
- ROI before home sends nothing, the same as stock.

### Fixes from 2026-10-15

- All seven F1-F7 fixes of 2026-10-15 are in HEAD.

## 6. Not covered (needs hardware or more time)

- **On-Cube timing.** SITL reports 0 µs for every task, so the ARRC tasks were not measured on the Cube.
  Run with SCHED_OPTIONS 1, read `@SYS/tasks.txt` over MAVFTP, and check PM NLon/MaxT. Measure these:
  - AP_Mount::update with the double-precision solve;
  - user_ARRC_gimbal, user_ARRC_SDR_logger and user_LB5900_logger;
  - the nav sends;
  - SDRM logging under a companion flood;
  - LB5900 I2C load at USR_LB_RATE 2/3.
- **Real Gremsy T7.**
  - How it applies 312 quaternions with roll and earth-frame locks.
  - Whether its GIMBAL_DEVICE_ATTITUDE_STATUS can carry NaN (L12).
  - The behaviour on Neutral vs Retract.
- **Real LB5900.**
  - Init timing when absent or with a stuck bus.
  - Whether it emits SCPI sentinels.
  - An end-to-end alignment and the POFF save.
- **DroneCAN HpposEcef on the Cube with the custom Here4 firmware.** GNSSPeriph-release was not reviewed (a
  separate repo); only the DSDL equality was checked. Also: does the Here4 still report ARMABLE (L21)?
- **Companion USB unplug and re-enumeration** on the Cube, and a USB CDC stall. Only heartbeat silence was
  tested.
- **AP_ARRC_SDR_ENABLED=0 runtime (L5).** Only the build was checked.
- **Parameter upgrade from a real 4.6-ARRC eeprom.** The vehicle already runs 4.7-ARRC, so the risk is low.
- **Mission Planner/QGC.** How they handle THISFIRMWARE "ARRC-ArduCopter V4.7.2" was not checked. Ludifly and
  the companion use the numeric version.
- **Current Cube parameters.** The newest snapshot is from 10-16. Read the live Cube before acting on the
  configuration items.
- **Integrated SITL run limits (rounds 2-3):**
  - Round 3 ran the real companion from source with its simulated SDR, the real parameters and a direct GCS
    link. Ludifly itself was replaced by a scripted GCS that mimics its stream requests.
  - The real airframe tuning and motor order could not fly the SITL octa, so SITL defaults were used.
  - Not run: RC7 in flight, and the RC-loss, battery and EKF failsafes with the real companion under the stale
    route (M8).
  - Absolute downlink latencies over the 57600 proxy are SITL-specific.
  - The real T7's reaction to Retract vs Neutral is inferred from Gremsy's documented behaviour, not measured.
- **SITL crash, simulator only.** The SITL Benewake TF02 simulator (`--serial4=sim:benewake_tf02`) crashes with
  SIGFPE after arming, on both builds. This is in `SITL::RF_Benewake::packet_for_alt`, not the firmware.

## 7. Prompt for a follow-up Claude Code session

Paste this into a new session started in `/home/tonito/Github/my-ardupilot`:

```text
Work through the findings of the ArduPilot-4.7-ARRC safety review in
RFSonde-docs/REVIEW_2026-10-22_ARRC-4.7.md (read it fully first, then sections 3-6 again as you go).
Evidence and harnesses from that review are in /var/tmp/claude-ap-review/work/<agent>/ (fake Gremsy T7,
companion failure injection fi.py, mode/failsafe harnesses, regression runner); reuse them.

Rules:
- Branch ArduPilot-4.7-ARRC in my-ardupilot. Fetch and compare with origin before any amend or rebase,
  because I push between turns. Follow AGENTS.md: one logical change per commit, "Subsystem: description"
  messages of 50 words or fewer, no AI trailer, minimal diffs in the files' own style, never change existing
  AP_GROUPINFO indices, keep compile guards.
- Do not touch the dialect (modules/mavlink) unless a fix needs it; doc-only XML fixes must keep the dialect
  hash 0x7EFB3788. If the dialect changes, update the RFSonde_Computer and Ludifly pins together.
- SITL only on instances >= 1 (never -I0, never pkill by name); my own sim may be running. Build in an
  isolated copy, or ask before reconfiguring my tree. Commit only after the relevant tests pass, judged by the
  test's own exit code.

Do, in this order, and ask me before anything marked (ASK):
1. Configuration (ASK; I apply it on the Cube): MNT1_DEFLT_MODE 1 (H1). Choose a replacement for FLTMODE2
   (PosHold is compiled out), or re-enable PosHold (H2). BATT_FS_CRT_ACT 1 (C1). MAV3_OPTIONS 2 in the saved
   file, with a reboot (C2). LGR_ENABLE/RC7_OPTION if no gear is fitted (C3). Confirm SERIAL1/SERIAL2_PROTOCOL 2.
   Read the live parameters first (Ludifly param editor or an HDF5 recording) and report the differences
   from the review's 10-16 snapshot.
2. H2 / compiled-out modes: propose re-enabling MODE_POSHOLD, MODE_BRAKE and MODE_SMARTRTL (251 KB of flash is
   free), or a Copter pre-arm check that fails when FLTMODE1-6, an RCx_OPTION mode switch, FENCE_ACTION or
   FS_*_ACT name a compiled-out mode (ASK which one). Test in SITL against stock 4.7.2.
3. M1 + M2 + L8-L10 (LB5900):
   - Send CASS_SENSOR_RAW only for healthy, new samples (NaN otherwise), with health and sample count in
     values[1..4].
   - Log LB59 only on new samples, with a counter.
   - Move the sensor configuration into the I2C callback as a non-blocking, retrying state machine.
   - Send a GCS text when the LB5900 is missing while it is the active source.
   - Saturate the failure counter, range-check replies, and fix the enable/health race.
   - Check the Ludifly side (RFSonde.py __read_cass_sensor) handles NaN/unhealthy; ASK before changing Ludifly.
4. M3 + M4 + L11-L15 (AP_Mount ARRC):
   - Continuity: hysteresis on the c.z orientation choice and on the slope switch, plus a rate limit on the
     angle target.
   - Treat an UNKNOWN GMODE as GMODE 0 with a warning and a pre-arm check.
   - Apply the 10 m hold only in GMODE 1-3, and only once a valid solution exists.
   - Combine the hard-coded limits with MNT1_*_MIN/MAX. Keep GMODE 0 stock-equivalent.
   - Reject NaN gimbal quaternions.
   - Fall back on the singular geometry.
   - Make ARGM report the mount mode.
   - Verify with the fake-T7 harness in /var/tmp/claude-ap-review/work/sitl-gimbal (jump check, boundary
     sweeps) and with stock comparisons.
5. M5 (alignment): use the active RF source (SDR pwr_c when the SDR is the source). Restore the
   pre-scan POFF on abort. Act on switch HIGH/LOW only. Base acceptance on the parameter uncertainty (ASK
   about the scan span).
6. M6 + M7 (link):
   - Apply the link-budget fix decided in section 4 (e.g. take ARRC_SENSOR_RAW/CASS out of STREAM_EXTRA3 and
     let Ludifly use SET_MESSAGE_INTERVAL; ASK before changing Ludifly).
   - Add a STATUSTEXT/pre-arm warning when a MAVLink port in use is MAVLink1.
   - Echo the key companion state in the autopilot's own downlink.
   - Re-measure with the 57600 proxy harness in /var/tmp/claude-ap-review/work/r2-integrated.
7. M8 (TX stop): in the firmware, look up the companion route by our own sysid (or by the channel of the last
   companion heartbeat), and resend until a COMMAND_ACK arrives. In RFSonde_Computer (ASK), pin link.sysid or
   do not heartbeat before adopting the sysid. Reproduce first with the round-3 harness
   (/var/tmp/claude-ap-review/work/r3-realcc/r3.py, companion started before SITL).
8. Lows L1, L3-L7 and L16-L25, as separate small commits:
   - SDR-loss warning;
   - bind the companion to its channel;
   - nav cap from get_ap_message_interval;
   - EXTRA3 guard for AP_ARRC_SDR_ENABLED=0 (and run that SITL variant);
   - HAL_MOUNT_ENABLED guards;
   - re-write RFSR/SDRC/SDRQ at log start;
   - cap companion-driven logging;
   - doc fixes (ARMING_SKIPCHK wording, CASS datatype 4, SDR_CONTROL param2);
   - restore the stock copter.parm and move the vehicle dump under RFSonde-sim/config.
9. After each group: build SITL and CubeOrangePlus-ODID (0 warnings, flash margin), run the
   ARRC-vs-stock regression runner (/var/tmp/claude-ap-review/work/sitl-regression) and the companion
   e2e (RFSonde_Computer/tools/sitl_e2e.py --sysid 2) and report the results. Then write the bench test plan for
   the items in section 6 that need hardware (on-Cube task timing with SCHED_OPTIONS 1, T7 Neutral/Retract,
   LB5900 absent/late, companion USB unplug).
```
