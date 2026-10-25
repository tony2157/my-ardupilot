# Here4 (GNSSPeriph-release, branch release-ARRC) firmware review

Date: 2026-10-23. Reviewer: Claude (AI-assisted review, code reading only).
Trigger: the Here4 feels warm and reports about 10 satellites where it used to report about 30.
Hypothesis to test: a loop in the ARRC firmware hogs the H757 CPU and starves the GPS.

## 1. Verdict

1. **No CPU-hogging loop was found.** The code the ARRC fork adds on top of CubePilot's
   v1.15.9 is 29 lines in the AP_Periph publisher and about 100 lines in the u-blox
   driver. It adds one 36-byte DroneCAN message at 5 Hz and one CFG-MSG poll every
   config cycle. It cannot load the CPU. The main loop sleeps in the CAN driver and runs
   at about 1 kHz, exactly like stock AP_Periph.
2. **MCU load cannot reduce the satellite count.** The number reported to the autopilot
   is numSV from UBX-NAV-PVT, computed inside the NEO-F9P receiver. An overloaded H757
   would drop UART bytes and produce gaps or an unhealthy GPS, not a stable 10-satellite
   fix. A persistent drop from 30 to 10 is a receiver-side configuration, RF, or
   receiver-restart problem.
3. **Two things really changed on this unit** between the June firmware and the
   2026-10-14 firmware, both from CubePilot upstream v1.15.8/v1.15.9, not from the fork:
   the "boot config prime" (a deliberate CFG-MSG write at power-on) and the CubeID
   Remote ID firmware pushed from v1.8.0 to v1.12.0 (the FEM build, that is with the
   2.4 GHz front-end amplifier enabled). Both are legitimate upstream fixes, but they
   are the only new variables on the hardware.
4. The known CubePilot issue "Here4 loses satellites about 8:20 after power-up" is a
   receiver quirk (first configuration write resets antenna status) and is the reason
   v1.15.9 exists. It gives a transient, not a persistent 10-satellite state, but it
   shows that this receiver reacts to configuration writes. The firmware writes
   several configuration items at every boot (section 5) and one of them enables
   NavIC on a receiver that supports at most four GNSS at once.
5. The plan (section 8) is measurement first: node telemetry and receiver-side UBX
   capture, then an A/B flash against the official v1.15.9 image. Nothing in the fork
   needs changing until those measurements point at it.

## 2. Scope and method

- Repos: `~/Github/GNSSPeriph-release` (branch `release-ARRC`, HEAD ee9ec3b717) and
  its `ardupilot` submodule (branch `here4-hpposecef`, HEAD dbc16da7e0).
- Compared against CubePilot upstream: `upstream/release` b84ff44cb2 (= v1.15.9) and
  `cubepilot/Here-v1.15` 79b94643fb. The fork is fully up to date with both; the
  nested submodules (ChibiOS, libcanard, mavlink...) match upstream exactly.
- Read: AP_Periph main loop, CAN loop, GPS publisher, the whole u-blox config state
  machine and parser, PPS/HSI trim, CubeID updater, serial tunnel, I2C slave,
  LED update, hwdef and defaults.
- Not done: no hardware was touched, no firmware was built, no logs were available
  (the Cube SD card is not mounted on this PC today).
- Build identity check: the bundled CubeID blob is v1.12.0, 164902 bytes,
  crc32 0x257cd554, and CubePilot's CI downloads `fem_fw_update.bin` v1.12 for every
  Here4 build, so the fork ships the same blob as the official firmware.

## 3. What the fork adds (and why it is harmless)

| Where | What | Cost |
|---|---|---|
| `AP_Periph/sensor_gps.cpp:269-295` | `send_hpposecef_msg()`: copies the last HPPOSECEF once per new message and broadcasts `ardupilot.gnss.HpposEcef` at LOW priority | one CAN frame per epoch (5 Hz) |
| `AP_Periph/can.cpp:683-684` | publisher priority/timeout | none |
| `AP_GPS_UBLOX.cpp:560-574` | `STEP_HPPOSECEF`: one CFG-MSG rate poll per config cycle, only when the receiver is an F9 | one 11-byte UBX poll every 0.75 s (2 s once configured) |
| `AP_GPS_UBLOX.cpp:617-622` | `_verify_rate` case: writes rate 1 only if the receiver reports a different rate; clears the bit once it matches | converges in one cycle |
| `AP_GPS_UBLOX.cpp:1971-2031` | parser: 28-byte payload copy, read-once flag, no `_check_new_itow()`, no nav state touched | microseconds |
| `AP_GPS_UBLOX.cpp:924-962` | `log_ecef()` is a no-op on the node (no logger); `get_hpposecef()` returns true once per message | none |
| `AP_GPS.cpp:1084-1093`, `AP_GPS_DroneCAN.*`, DSDL, LogStructure | plumbing; the DroneCAN subscriber is vehicle-side only | none |
| `waf`, `scripts/py3_compat`, `BUILD_HERE4.txt` | build tooling | none |

The `CONFIG_RATE_HPPOSECEF` bit is kept out of `CONFIG_ALL`, so it cannot hold the node
in the "unconfigured" state. The HPPOSECEF week-rollover fix (dbc16da7e0) is a
comparison and an increment.

## 4. Main loop and timing (no busy loop)

- `AP_Periph_FW::update()` (`AP_Periph/AP_Periph.cpp:231-328`) runs LED, MAVLink and
  housekeeping at 1 Hz / 50 Hz gates, then `can_update()`.
- `can_update()` (`AP_Periph/can.cpp:704-745`) calls `can_gps_update()`, mag, baro,
  flushes the debug log buffer, then `canard_iface.process(1)`.
- `CanardInterface::process()` (`libraries/AP_DroneCAN/AP_Canard_iface.cpp:397-420`)
  processes RX/TX and then **waits on a semaphore until the 1 ms deadline**. That is the
  idle point. The loop therefore runs at about 1 kHz, identical to upstream.
- `AP_GPS_UBLOX::read()` (`AP_GPS_UBLOX.cpp:704-722`) sends one config step every
  300 ms before the first PVT, 750 ms while anything is unconfigured, 2000 ms once
  configured. It reads at most 8192 bytes per call. GPS traffic at 230400 baud is a few
  hundred bytes per 200 ms epoch.
- `pps_interrupt()` (`AP_GPS_UBLOX.cpp:2104-2138`) runs 1000 times a second because the
  Here4 configures the receiver timepulse to 1 kHz for HSI trimming. That is a few
  microseconds per interrupt; upstream behaviour since v1.15.4.
- Threads: no IMU thread (`IMU_SAMPLE_RATE` defaults to 0), no monitor thread
  (`HAL_NO_MONITOR_THREAD`), no scripting. The I2C slave peripheral is interrupt driven
  and idle on a CAN installation.
- CubeID updater (`AP_Periph/MAVLink.cpp:168-212`): sends an update-start on every
  CubeID heartbeat until the module answers "already current". The 10-14 bench check
  saw `CubeID running fw v1.12.0`, so it had converged.

## 5. Everything the node writes into the receiver at each boot

These are the real candidates for a receiver-side change in satellite count. All but
the HPPOSECEF line are CubePilot upstream behaviour. Layers: RAM is lost at receiver
power-off, BBR survives with backup power, Flash is permanent.

| Order | Write | Layer | Condition | Reference |
|---|---|---|---|---|
| 0 | CFG-PRT + `$PUBX,41` to 230400 baud (detection blob) | RAM | always | `AP_GPS.cpp:75-87` |
| 1 | **Boot config prime**: CFG-MSG NAV-CLOCK rate toggled then restored, verified by poll | RAM | always, even with `GPS_AUTO_CONFIG=0` | `AP_GPS_UBLOX.cpp:308-316, 2242-2305` |
| 2 | CFG-MSG rates: PVT 1, TIMEGPS 5, DOP 1, MON-HW 5, MON-HW2 5, POSLLH/STATUS/SOL/VELNED 0, **HPPOSECEF 1 (fork)** | RAM | only when the polled rate differs | `_verify_rate`, :583-681 |
| 3 | CFG-NAV5 dynModel 8 (airborne 4 g), minElev untouched (`GPS_MIN_ELEV` -100) | RAM | if different | :1259-1285 |
| 4 | CFG-GNSS constellation set | RAM | only if node param `GPS_GNSS_MODE` is non-zero | :1288-1340 |
| 5 | **CFG-TP5: timepulse 1000 Hz, flags 0x3f** (for HSI trim) | RAM | if different | :1432-1465 |
| 6 | CFG-RATE 200 ms, nav rate 1 | RAM | if different | :1419-1428 |
| 7 | CFG-VALSET TMODE_MODE 0 | RAM | if time mode was on | :1487-1500 |
| 8 | CFG-VALSET F9 debug streams off: NAV-SAT, NAV-CLOCK, NAV-SIG, MON-RF, MON-HW3, MON-COMMS, **MON-SPAN** = 0 | RAM+BBR | `GPS_DRV_OPTIONS` bit 10 clear (it is) | :294-302, :537-558 |
| 9 | **CFG-VALSET L5 override: L5_HEALTH_OVRD 1, GPS_L5_ENA 1, NAVIC_ENA 1, NAVIC_L5_ENA 1** | RAM+BBR | receiver reports "L1L5" in MON-VER and `GPS_DRV_OPTIONS` bit 5 set (default 288 = bits 5 and 8) | :268-273, :519-534, :1603-1609 |
| 10 | CFG-CFG save all (RAM to BBR+Flash) | Flash | when everything is configured, `GPS_SAVE_CFG`=2 and something was changed; up to 5 tries | :723-731, :2390-2405 |
| any | Auto-disable of any UBX message the parser does not know: CFG-MSG rate 0 every 256th occurrence | RAM | always, even with `GPS_AUTO_CONFIG=0` | :1111-1121, :2071-2076 |

Points worth attention:

- **Line 9 enables NavIC.** The Here4 manual lists the NEO-F9P as L1/L5 with GPS,
  GLONASS, Galileo, BeiDou plus SBAS/QZSS and "max 4 GNSS systems"; NavIC is not among
  them. If the receiver accepts the write, a fifth system competes for channels and the
  NavIC satellites are never visible from Oklahoma. If it rejects the write (VALSET
  NACK), the driver does not handle VALSET NACKs: the `CONFIG_L5` bit never clears, the
  node stays "unconfigured" (750 ms config cycle forever, no CFG-CFG save, no
  GlobalTimeSync, `ARMABLE` false in `ardupilot.gnss.Status`). Either way this line is
  the first thing to read back from the receiver. This is CubePilot's code, not the
  fork's, but it is also **not** upstream ArduPilot behaviour: ArduPilot 4.7's list
  (`AP_GPS_UBLOX_CFG_List.h:189-192`) writes only `L5_HEALTH_OVRD 1` and
  `GPS_L5_ENA 1`; the two NavIC keys exist only in CubePilot's Here-v1.15 driver.
- **The disable list is asymmetric.** Clearing bit 5 of `GPS_DRV_OPTIONS` only writes
  `L5_HEALTH_OVRD 0` (`config_L5_ovrd_dis`, :275-277). NavIC stays enabled in BBR
  once it has been turned on. Reverting needs a manual VALSET or a receiver config
  reset.
- **The fork added HPPOSECEF to line 2 in June**, so if the 30-satellite baseline is
  from before June, that write and the CFG-CFG save that follows it are part of the
  difference too. The write itself is an output-message enable and cannot change
  tracking; but it is a configuration write, and this receiver resets its antenna
  status on the first write after power-up (section 6).
- The node masks `CONFIG_TP5` out of the armable check (`sensor_gps.cpp:258`), which
  means CubePilot expects the 1 kHz timepulse write to fail on some units.

## 6. What changed on this unit, and when

| | June firmware (flashed around 2026-06-04) | Current firmware (flashed 2026-10-14) |
|---|---|---|
| GNSSPeriph commit | 4ed168b057 on v1.15.7-10-g5e6cc3bcbe | ee9ec3b717 on v1.15.9 (b84ff44cb2) |
| ardupilot submodule | e4120b4bad (base b315a73a86, 2026-05-06) | dbc16da7e0 (base 79b94643fb, 2026-08-28) |
| u-blox driver delta | HPPOSECEF (fork) | HPPOSECEF + week fix (fork), **boot config prime (upstream)** |
| CubeID blob | v1.8.0, 198414 B, crc 0x659fc531 | **v1.12.0 FEM build**, 164902 B, crc 0x257cd554 |
| defaults.parm, hwdef | identical | identical (`GPS_DRV_OPTIONS 288`, `GPS1_TYPE 2`) |

So relative to June the unit gained exactly two behaviours: the prime and the new
Remote ID firmware with the front-end amplifier enabled. The HSI trim at 1 kHz PPS, the
L5/NavIC override, the F9-debug disable list and MON-* message consumption were already
in the June firmware (they date from v1.15.4 and 2026-05-06).

If the 30-satellite baseline predates June (stock CubePilot firmware), add the
HPPOSECEF enable and the fork's first CFG-CFG save to the delta.

## 7. Known receiver-side issue in the field

- CubePilot v1.15.9 release note: "GPS satellites and 3D fix drop once, several minutes
  after boot (about 8.5 minutes in). Cause: a u-blox NEO-F9P firmware quirk where the
  first configuration write after boot resets antenna status. Affected units had an
  unexpected MON-SPAN message enabled, likely left from factory testing, so the first
  configuration write landed at about 8 minutes 32 seconds." The firmware now performs a
  harmless write at power-on (the prime). The 8:32 figure is the 256-message
  auto-disable counter in `unexpected_message()` at a 0.5 Hz MON-SPAN rate.
- Forum thread 17408 (March to August 2026): drops from 28-30 to 10-13 satellites,
  once per power cycle, reproducible on the bench, on Here4 Black and Blue, HW 4.19,
  all v1.15 builds up to v1.15.7. A user found MON-SPAN on UART1 as the trigger by A-B-A
  testing; a receiver config revert to defaults plus save plus cold start removed the
  dropouts but left the unit at 24-26 satellites instead of 28-30. CubePilot said it was
  working with u-blox; no official statement about which units are affected.
- Forum thread 17893: a Here4 on 1.15.309D03B7 whose node driver never left
  configuration (error_codes 24575, re-probing every 13.6 s) while u-center through the
  passthrough showed 7 satellites used of 26 tracked. Unresolved. It shows that "used"
  and "tracked" can differ a lot on these units and that the node's config state
  machine can get stuck on a receiver that answers MON-VER but NACKs or ignores writes.

What this means here: the unit now runs the version that contains CubePilot's fix for
the transient, so a transient at 8 minutes is not expected any more. A *persistent*
10-satellite state is a different thing and the receiver has to be asked directly
(section 8, step 2). "Satellites" everywhere above is numSV used, not tracked.

## 8. Code nits found (none are a cause)

- `GPS_DRV_OPTIONS` bit 8 (`HSITrimUsingPPS`) is defined (`AP_GPS.h:673`) but never
  read. HSI trim is always on when `HAL_HSI_TRIM_USING_PPS` is compiled in, which the
  Here4 hwdef does. The bit is a no-op. Upstream CubePilot.
- `hal.gpio->attach_interrupt(HAL_GPIO_PPS, ...)` is called on every CFG-TP5 poll
  response (`AP_GPS_UBLOX.cpp:1440`), that is every config cycle for ever. The HAL
  rejects the second attach because the line already has a callback
  (`AP_HAL_ChibiOS/GPIO.cpp:368-371`), so it is harmless. Same in upstream ArduPilot.
- Unknown-message auto-disable and the prime both run with `GPS_AUTO_CONFIG=0`. The
  parameter name promises more than it does; it matters for the capture in step 2.
- VALSET NACKs are not handled (only VALGET and CFG-MSG NACKs are). A rejected key
  blocks the config state machine permanently. Upstream.
- The prime's first CFG-MSG poll arrives before the port is known, which costs one
  extra 300 ms round (the handler asks for CFG-PRT first). Harmless.
- Fork: `RATE_HPPOSECEF 1` means 5 Hz at the default `GPS_RATE_MS`. Fine. Nothing to
  change.

## 9. Plan

Measurement first. Steps 1 to 3 are read-only and safe on the bench. Keep the
antenna position, sky view and powered neighbours (gimbal, bladeRF, LattePanda) the
same between runs; log the start time of each power-up.

**Step 0. Existing evidence (10 minutes).** Open the most recent Cube dataflash logs
(SD card, `APM/LOGS`) in Mission Planner or copy the `.BIN` into this session folder.
Plot `GPS.NSats` and `GPS.Status` over time and read the `MSG` lines starting with
`CAN[` (node text is always logged). Look for: a flat 10-satellite line from boot or a
step; the receiver version string `u-blox 1 HW: 00190000 SW: EXT CORE 1.00 (...)` and
whether it contains `L1L5`; `u-blox supports L5 Band`; `u-blox config primed`;
`u-blox saving config`; any `configuration` failure reason text such as
`L5 Enable Disable`; node uptime resets in `CAND`. Note the IDs of the `GPS[0]` versus
other GPS instances.

**Step 1. Node telemetry (one cold boot, 10 minutes).**

- Vehicle: `CAN_LOGLEVEL` = 3 (INFO and below reach the GCS as `CAN[n] ...`; 4 adds
  DEBUG text).
- Node (via Mission Planner DroneCAN parameters): `DEBUG` = 13 (bit 0 stack per thread,
  bit 2 CAN stats, bit 3 per-UART stats every 5 s), `GPS_DRV_OPTIONS` = 800 (288 plus
  bit 9, HSI trim stats: PPS interrupts/s, trim value, error and **MCU temperature**
  every 5 s). Reboot the node.
- Read: `SERIAL3` line `TX= RX= TXBD= RXBD= RXO=`; RXO (RX overruns) must stay 0 and
  RX bytes/s should be a few hundred. Non-zero RXO is the only signature an overloaded
  MCU could leave. PPS rate must read about 1000/s and the trim value stable. MCU
  temperature gives the first number for "warm". Stack lines should not shrink.
- Also note whether `u-blox saving config` ever appears; if it never does, something
  stays unconfigured (section 5, line 9 is the first suspect).
- Afterwards set `DEBUG` back to 4 and `GPS_DRV_OPTIONS` back to 288.

**Step 2. Receiver-side capture (the decisive step).** Use CubePilot's gps_debug tool
(`scripts/gps_debug` in GNSSPeriph-release, run on Windows against the Cube's MAVLink)
or Mission Planner's u-center passthrough (Here4 manual section 6, 230400 baud). Enable
**lock port** in the tunnel so the node's driver stops reading and cannot auto-disable
the messages you turn on. Collect, in order:

1. MON-VER (receiver firmware and the L1L5 marker).
2. CFG-VALGET of the RAM, BBR and Flash layers for: CFG-SIGNAL-* (all constellations
   and signals, including NAVIC_ENA and L5_HEALTH_OVRD), CFG-NAVSPG-INFIL_MINELEV,
   CFG-NAVSPG-DYNMODEL, CFG-RATE-MEAS, CFG-TP-* (timepulse 1000 Hz expected),
   CFG-MSGOUT-UBX_MON_SPAN_UART1 and the other MSGOUT keys. Save the dump; it is the
   baseline for every later comparison.
3. Poll MON-SYS (receiver CPU load, memory and **receiver temperature**), MON-RF
   (jamming indicator, noise, AGC per RF block) and MON-HW (antenna status and power,
   noisePerMS, jamInd).
4. Enable NAV-SAT for two minutes: count tracked versus used, per-signal C/N0 and the
   health and ephemeris flags. Thirty tracked and ten used means configuration or
   health flags; ten tracked means RF or antenna.
5. Leave MON-RF and NAV-SAT running across the first 10 minutes after a cold power-up
   to see whether anything steps (the old 8:32 transient would show as an AGC step).

**Step 3. Firmware A/B (30 minutes each, same conditions).** Flash the official
`Here4` v1.15.9 `.apj` from the GNSSPeriph-release GitHub releases page. Same satellite
count and temperature means the fork is cleared and the issue is in the upstream
firmware, the receiver or the environment. Then flash v1.15.7: if 30 satellites come
back, the difference is the prime or the CubeID v1.12 FEM firmware (section 6). Then
re-flash the fork and confirm the result is reproducible, not a sky-view artefact.
The fork's `.apj` is `build/Here4/bin/AP_Periph.apj`. Flashing does not touch the
receiver's stored configuration. Do not flash u-blox receiver firmware (the manual
warns it removes moving-baseline support).

**Step 4. Isolate the Remote ID module if step 3 points at it.** The June blob
(v1.8.0) is extractable from the submodule commit e4120b4bad and a fork build bundling
it would push it back to the module; only do that if the A/B says the CubeID change
matters and CubePilot has no statement about v1.12 on Here4 Blue. The bladeRF on the
bench can also check the RF side directly: compare the noise floor at 1575.42 MHz and
1176.45 MHz with the Here4 powered and unpowered, and look for 2.4 GHz activity.

**Step 5. If the receiver configuration is the cause.** Prefer a targeted fix over the
forum's full "revert to defaults": write the specific keys back (for example
NAVIC_ENA 0, NAVIC_L5_ENA 0) to RAM+BBR+Flash with u-center, cold start, and re-check
step 2. If NavIC is the problem, the clean firmware fix is a `GPS_DRV_OPTIONS` default
without bit 5 plus a disable list that also clears NavIC; that would be a change to
propose upstream to CubePilot, not something to carry quietly in the fork. If a full
reset is unavoidable, save the step 2 dump first and expect to lose nothing but the
items in section 5, which the node rewrites at the next boot.

**Step 6. Only if steps 1 to 3 still leave CPU load in doubt.** Build a bench-only
debug firmware with `define HAL_ENABLE_THREAD_STATISTICS TRUE` in the Here4 hwdef
(turns on `CH_DBG_STATISTICS`) and extend `show_stack_free()` to print each thread's
cumulative cycles, which gives the idle-thread share directly. That is a two-line
change and is the only way to get a real CPU percentage out of this board.

**Step 7. Decide and record.** Write the numbers (satellites used/tracked, MCU and
receiver temperature, RXO, PPS rate, config dump diff) into this folder next to this
file and update the Here4 memory note.

## 10. Live investigation, 2026-10-23 evening (vehicle ArduPilot-4.7-ARRC-revamp)

Question: does the vehicle firmware configure the Here4 correctly, and is the GNSS loss
software or hardware? Method: read-only MAVLink client on the Herelink port after the
user closed Ludifly (system id 253, so no GCS failsafe is triggered when it leaves),
plus the GCS telemetry logs of the day, plus the Cube's CAN frame forwarding
(`MAV_CMD_CAN_FORWARD`, type filter 341/16383/1063/20003) to read the Here4's own
DroneCAN traffic. Scripts and raw captures: session scratchpad `live/`.

**Vehicle side: cleared.**

- Live parameters: `GPS1_TYPE` 9, `GPS_AUTO_CONFIG` 1, `GPS1_GNSS_MODE` 0,
  `GPS1_RATE_MS` 200, `GPS_DRV_OPTIONS` 32, `GPS_INJECT_TO` 127 with no RTCM source,
  `CAN_D1_UC_OPTION` 0, `CAN_LOGLEVEL` 0, `DID_ENABLE` 1, `DID_CANDRIVER` 1,
  `GPS1_CAN_NODEID` 119. With `GPS_AUTO_CONFIG` 1 the Cube never issues a parameter
  get/set to the node (`AP_GPS_DroneCAN::read`, `do_config` runs only at value 2), so
  the vehicle configures nothing on the Here4.
- Vehicle code delta vs Copter-4.7.2 in the GPS/CAN/ODID area: the HpposEcef
  subscriber (passive), OpenDroneID send-path changes (always flag DroneCAN, Location
  at MEDIUM priority with 50 ms timeout) and board defaults. None can affect the
  receiver.
- Vehicle quirk observed (upstream behaviour, cosmetic): "GPS 1: detected DroneCAN1-119"
  printed 154 times this morning at the Fix2 rate while the receiver had no fix after
  having had one. `update_instance` re-announces after the timeout branch clears
  `state[]`; the timeout keeps firing because the corrected timestamp derived from the
  node's epoch time is stale when the receiver has no fix. Not a node reboot.

**Here4 node: healthy and correctly configured.**

- NodeStatus uptime counted continuously through 4- and 10-minute captures (health OK,
  mode OPERATIONAL, free memory constant 62008 B). Only reset was the user's power
  cycle at 20:29:27.
- Boot texts captured: `GPS 1: probing for u-blox at 230400 baud`,
  `GPS 1: detected u-blox`, `GPS 1: u-blox config primed` (one second after detection),
  `CubeID running fw v1.12.0` (19 s after boot). One longer text (the MON-VER line) was
  lost on the 57600 link.
- `ardupilot.gnss.Status.error_codes` (the driver's unconfigured bitmask) went from
  0x315fff at boot to 0 in 23 s, with the node flagged ARMABLE from 20:29:50 on. Bits
  20 (`CONFIG_L5`) and 21 (`CONFIG_F9_DEBUG`) were set and cleared, so the receiver
  reports L1/L5 and the L5/NavIC override list was applied and verified. No
  "saving config" text followed, so nothing in the receiver had to be changed: the
  stored configuration already matched. No config churn.

**Receiver: the GNSS loss is in the receiver's RF path, and it is getting worse.**

- 11:07 power-up (Mission Planner log): first fix after 33 s with 7 satellites, 24
  satellites and a DGPS fix with HDOP 0.52 after 3.5 minutes.
- Same morning: five Cube reboots (11:07, 11:15, 11:24, 11:36, 12:00). After every one
  the receiver dropped to 0 satellites 10 to 20 s later, for 20 s to 2 min, and came
  back with fewer (24, 16, 9, 10). During those episodes the Here4's Remote ID relay and
  compass also went quiet for seconds ("ODID: lost transmitter" 19 times, "Compass 1 not
  healthy").
- Evening (Ludifly logs and live): 4 to 10 satellites, HDOP 1.1 to 2.5, horizontal
  accuracy estimate 10 to 90 m; brief 0-satellite dips at 18:25 to 18:29 and 19:46.
- 20:29:27 full power cycle: **no fix and 0 satellites for 4 min 46 s**, then a 3D fix
  with 4 satellites and a 141 m accuracy estimate, settling at 5 satellites, HDOP 1.8,
  11.6 m after 9 minutes. A NEO-F9P with any sky view acquires in well under a minute.

**Reading.** The node firmware and the vehicle firmware do what they should; the
receiver sees a degraded signal. Two explanations remain and only a bench test
separates them:

1. Hardware: antenna LNA or antenna supply failing (the unit is warm; MON-HW
   `aStatus`/`aPower` would show it).
2. The Remote ID transmitter inside the Here4 Blue, whose firmware went from v1.8.0 to
   the v1.12.0 FEM build on 2026-10-14. It starts broadcasting a few seconds after each
   Cube boot, which matches the morning timing, and a front-end amplifier in the same
   housing explains both heat and desensitisation.

**A/B material prepared** (local builds, nothing flashed), on the Windows desktop in
`C:\Users\sega0000\Desktop\Here4_fw\`:

- `Here4_AP_Periph_2026-06-04_4ed168b0_CubeIDv1.8.apj`: the June firmware the unit ran
  until 2026-10-14 (same HPPOSECEF code, CubeID v1.8.0 blob). Flashing it makes the
  Here4 push v1.8.0 back into the CubeID on first boot.
- `Here4_AP_Periph_2026-10-13_ee9ec3b7_CubeIDv1.12.apj`: the current firmware, to go
  back afterwards.
The GNSSPeriph checkout was restored to release-ARRC afterwards and the build folder
rebuilt; the rebuilt image matches the saved current image.

**Next steps, in order.**

1. Mission Planner over USB, DroneCAN tab, u-center passthrough on the Here4: read
   MON-HW (antenna status and power, noise, AGC, jamming indicator), MON-RF and NAV-SAT
   (tracked vs used, C/N0). This decides hardware vs interference in minutes.
2. Flash the June image, power-cycle, wait for `CubeID running fw v1.8.0`, and compare
   time to first fix and satellite count at the same spot for 10 minutes. Then flash
   the current image back.
3. If the June image fixes it, the problem is the v1.12 FEM Remote ID firmware on this
   unit; report to CubePilot with both captures and keep v1.8.0 until they answer. If
   it does not, the antenna path is failing and the unit needs replacement or warranty.
4. Housekeeping: my capture stopped the TELEM1 streams; Ludifly or Mission Planner
   re-request them on connect. The Cube shows "PreArm: GCS failsafe on" until a GCS
   with system id 255 connects again.

## 11. A/B result, receiver baseline and fix plan (2026-10-23, 21:00 to 21:45)

**Result.** The user flashed `release-ARRC-June2026` (June code + ECEF week fix, CubeID
v1.8.0 blob). Over USB, 60 s after settling: DGPS fix, 29 to 32 satellites, HDOP 0.52,
horizontal accuracy 3.5 m, node configured (error bitmask 0, armable). The same unit,
same spot, 70 minutes earlier on the current image: no fix for 4 min 46 s after power-up,
then 5 satellites and 12 m. **The current firmware causes the GNSS degradation; the ECEF
code is not involved** (June2026 carries the identical publisher, parser and DSDL).

**Receiver baseline with June2026** (UART tunnel through the Cube, read-only polls; raw data
in `/var/tmp/claude-rfsonde-tests/here4/baseline_june2026/`):

| Item | Value |
|---|---|
| MON-VER | NEO-F9P, FWVER HPGL1L5 1.41, PROTVER 27.40, EXT CORE 1.00 (49f616), GPS;GLO;GAL;BDS, SBAS;QZSS, NAVIC |
| MON-HW | antenna status OK, antenna power ON, noisePerMS 73, agcCnt 3510, jamInd 3, jammingState 0 |
| MON-RF | L1 block: antStatus OK, antPower ON, noise 73, AGC 3510, jamInd 8; L5 block: noise 41, AGC 4212, jamInd 3 |
| MON-SYS | CPU 61 % (max 62), temperature 43 C, runtime 3271 s, error counter 744 |
| NAV-SAT | 39 tracked, 32 used; used C/N0 16 to 39 dBHz; GPS 12/9, SBAS 3/3, Galileo 11/7, BeiDou 7/7, GLONASS 6/6 |
| CFG (RAM = Flash) | NavIC on, NavIC L5 on, GPS L5 on, L5 health override on, dynmodel 8, min elevation 10, 200 ms, UART1 230400, TP1 1000 Hz, MSGOUT PVT 1, HPPOSECEF 1, MON-HW 5, TIMEGPS 5, DOP 1, debug streams 0, TMODE 0 |

The configuration is exactly what the node driver asks for and is saved in Flash, so the
receiver configuration is not the variable either.

**What differs between the two images** (everything else is byte-identical or cosmetic):

| | current `release-ARRC` (ee9ec3b7 / dbc16da7e0) | `release-ARRC-June2026` (75c4909b / 63f4f27fe2) |
|---|---|---|
| CubeID Remote ID firmware blob pushed into the module | **v1.12.0 FEM build** (164902 B, from CubePilot v1.15.8) | **v1.8.0** (198414 B) |
| u-blox boot config prime (CubePilot v1.15.9) | present | absent |
| CubeID version text at boot, `MAVLink.h` fields | present | absent (cosmetic) |
| HPPOSECEF week fix, publisher, parser, DSDL | same | same |

Which of the two is the cause is not yet separated, but the evidence points at the
Remote ID firmware: this morning every fix loss followed a **Cube** reboot by 10 to 20 s
while the Here4 node stayed up (so no prime ran), and the Cube's reboot is exactly when
the OpenDroneID stream to the CubeID restarts ("ODID: transmitter OK" 1 to 2 s after
each boot). An RF transmitter with a front-end amplifier inside the same housing explains
the desensitisation (long time to first fix, 5 to 10 satellites, 10 to 90 m accuracy
estimates) and the warm case. The prime is a one-time 1 s event at node boot and cannot
explain fix losses that follow Cube reboots.

**Cross images to separate the two** (built, on the desktop in `Here4_fw\`, not flashed):

- `VARIANT-B_June2026_75c4909b_with_CubeIDv1.12.apj`: June2026 code, v1.12 FEM blob. If
  GNSS degrades again, the Remote ID firmware is the cause.
- `VARIANT-A_current_ee9ec3b7_with_CubeIDv1.8.apj`: current code incl. prime, v1.8.0 blob.
  Expected healthy if the blob is the cause; if it degrades, the prime is implicated.
After each flash: power-cycle, wait for the CubeID update text, 10 minutes, then run
`tunnel_raw.py COM14 <name>.json` and `com_probe.py COM14 60 <name>.json` and compare
with the baseline (noise, AGC, jamInd, antenna status, tracked vs used, C/N0).

**Fix plan.**

1. Pin the CubeID blob to v1.8.0 on `release-ARRC` (revert fb115f3f4b, keep everything
   else of v1.15.9) once Variant B confirms, and document the reason in `BUILD_HERE4.txt`.
   Cost: v1.8.0 has the cold-boot UART wedge that v1.12 fixed (sticky "invalid location"
   prearm on some boots; power-cycle clears it). Flying with v1.8.0 is what this unit did
   from June to October.
2. If instead Variant A degrades, gate the boot config prime behind a `GPS_DRV_OPTIONS`
   bit (default off) in the fork's u-blox driver and keep MON-SPAN disabled by the F9-debug
   list, which already protects this unit from the 8:32 transient.
3. Report to CubePilot (GNSSPeriph-release issue): unit HW 4.19, NEO-F9P 49f616, both
   captures (sat count, time to first fix, MON-HW/MON-RF/NAV-SAT before and after), the
   fix-loss-after-Cube-reboot timeline, and the request for a Here4 Blue CubeID build
   that does not desensitise the receiver (or a non-FEM v1.12). Ask whether other Here4
   Blue units show the same on v1.15.8+.
4. Keep `release-ARRC-June2026` as the flight-ready fallback until the pinned branch is
   tested for a full session (first fix under 60 s, 25+ satellites, no 0-satellite dips).
5. Vehicle side: no change needed. Optional: set `CAN_LOGLEVEL` 3 on the Cube so the
   node's texts (receiver version, "config primed", "saving config", CubeID version) show
   in the GCS and in the dataflash during future tests.

## 12. A/B outcome: the boot config prime is the cause (2026-10-23, 22:00)

Four flashes of the same unit at the same spot, same evening:

| Image | CubeID blob | Boot config prime | Result |
|---|---|---|---|
| `release-ARRC` ee9ec3b7 (current) | v1.12 FEM | yes | no fix for 4 min 46 s, then 5 satellites, 12 m |
| `release-ARRC-June2026` 75c4909b | v1.8.0 | no | fix in under a minute, 29 to 32 satellites, 3.5 m |
| `release-ARRC-cubeid-v1.8` 7ab70ab478 | v1.8.0 | yes | **not above 10 satellites** |
| `release-ARRC-June2026` again | v1.8.0 | no | over 20 satellites again |

The two failing images share only the prime; the two working images lack only the
prime. The CubeID firmware is cleared (section 11's lean toward it was wrong). The
ECEF code is in all four.

**Mechanism.** CubePilot's v1.15.9 note states the receiver quirk: the first
configuration write after power-up resets the antenna status. Their prime deliberately
provokes that write about one second after every Here4 boot, assuming the effect is a
harmless blip before the first fix. On this unit (NEO-F9P, HPGL1L5 1.41, build 49f616,
HW 4.19) the effect is not a blip: the receiver stays at 5 to 10 satellites with poor
C/N0 until it is power-cycled without a write. The June2026 firmware never writes
anything on a normal boot because the driver's configuration is already saved in the
receiver's flash (section 11 shows RAM equal to Flash), so the quirk never fires.

This also explains the morning (section 10): each of the fix losses followed a 1 to 3 s
gap in the Here4's Remote ID relay, i.e. a node restart, and the prime runs at every
node restart. Why the node restarted (Cube reboots at 11:15 and 12:00, two more at
11:24 and 11:36) is a separate, lesser question; tonight the node ran 45 minutes
without a restart.

**Fix candidate built**: `release-ARRC-noprime` (e3aaad06b0), submodule
`here4-hpposecef-noprime` (4dce3f797e = dbc16da7e0 plus a clean revert of CubePilot's
79b94643fb). Everything else of v1.15.9 stays, including the CubeID v1.12.0 blob and
its version report. Image on the desktop:
`Here4_AP_Periph_1.15.9-ARRC_e3aaad06b0_NOPRIME_CubeIDv1.12.apj`. If it is healthy
it becomes the flight firmware and confirms the blob is innocent in one step.

**Operating rule that follows.** Any configuration write to the receiver is a risk on
this unit, not only the prime: after changing a GPS parameter on the node
(`GPS_RATE_MS`, `GPS_GNSS_MODE`, `GPS_DRV_OPTIONS`, ...), or after a first flash that
changes the message set, power-cycle the drone once and check the satellite count.
MON-SPAN is off in this receiver's flash, so the 8:32 transient the prime was meant to
pre-empt does not apply here.

**Report to CubePilot.** The prime should be optional (a `GPS_DRV_OPTIONS` bit, default
off, or skipped when the receiver's stored configuration already matches), and the
antenna-status reset deserves a look on HPGL1L5 1.41 / 49f616 units: on this one it is
a persistent degradation, not a transient. Evidence to attach: the four-flash table,
the healthy baseline (`/var/tmp/claude-rfsonde-tests/here4/baseline_june2026/`), and,
when convenient, one MON-HW/MON-RF/NAV-SAT capture under the failing image
(`tunnel_raw.py COM14 bad.json`) to show the antenna status and C/N0 difference.

## 13. Second factor: the companion computer (2026-10-23, 22:30)

User report: the degradation returned on `release-ARRC-June2026` and appears mainly
while the companion computer (LattePanda + bladeRF) is up, running and connected.
Ludifly logs 21:59 to 22:27 (same spot):

| Session | Firmware | SDR stream | Satellites |
|---|---|---|---|
| 21:59 to 22:01 | cubeid-v1.8 (prime) | started 22:00:10 | 0 for 2 min |
| 22:10 to 22:12 | cubeid-v1.8 (prime) | started 22:11:43 | 0 for 2 min |
| 22:17 | cubeid-v1.8 (prime) | started 22:17:44 | 0 |
| 22:18 to 22:20 | June2026 (node restarted 22:18:18) | none | 0 to 6 to 12 in 2 min, 3.6 m |
| 22:23, 22:27 | June2026 | none | 18 to 23, DGPS, 3.5 m |

**Can the autopilot's MAVLink rate to the CC change what it asks the Here4 for? No.**

- The Here4 broadcasts Fix2 at its own `GPS1_RATE_MS` (200 ms, read from the node
  tonight); DroneCAN GPS is publish-only, the Cube never requests a position.
- The CC asks the Cube for MAVLink streams with `MAV_CMD_SET_MESSAGE_INTERVAL`
  (`stream_rates_hz` in `rfsonde_cc/config.py`: MISSION_CURRENT 1 Hz,
  EXTENDED_SYS_STATE 2 Hz, ARRC_NAV_IMU 50 Hz, ARRC_NAV_MAG 10 Hz, ...). These only
  change what the Cube sends on its serial link; nothing propagates to the CAN bus.
- The Cube has `GPS_AUTO_CONFIG` 1, so it never writes a parameter to the node, and the
  CC code sends no RTCM or GPS injection (`GPS_INJECT_TO` 127 has no source).
- The vehicle-side ARRC libraries (`AP_ARRC_SDR`, `AP_ARRC_LB5900`) only read GPS state.
- The Here4's CPU is not the bottleneck: node health OK, configuration bitmask 0 and
  stable memory in every capture, including the bad ones.

So the coupling between the CC and the Here4 can only be physical: **RF** or **power**.
Both are classic with a companion computer plus USB 3 SDR next to a GNSS antenna:
USB 3 SuperSpeed cabling and connectors radiate broadband noise into L-band; the
bladeRF's 38.4 MHz VCTCXO has its 41st harmonic at 1574.4 MHz, 1 MHz from the GPS
L1/Galileo E1 centre (to be confirmed by the receiver's spectrum, see below); the
LattePanda's DDR/USB clocks add more. Power: a shared 5 V rail sagging or rippling
when the N5105 and the bladeRF load up would starve the Here4's LNA/receiver.

Note the two disruptors are independent: the 20:29 power cycle on the prime firmware was
bad with the CC idle (no SDR stream in that capture), and June2026 is good with the CC
idle (21:23, 21:40, 22:23). Both must be removed.

**Controlled test (receiver's own instruments, same firmware June2026-weekfix):**
Cube USB on the PC (COM14) for the probe; Ludifly on the Herelink to drive the CC.
For each state run `tunnel_raw.py COM14 <state>.json` (now also polls MON-SPAN, the
receiver's 256-bin spectrum per band) and `com_probe.py COM14 60 <state>.json`:

| State | How |
|---|---|
| A: CC off | LattePanda unpowered |
| B: CC on, idle | CC booted, Ludifly closed (SDR stream stopped) |
| C: CC streaming | Ludifly connected, RF feed on ("SDR streaming started") |
| D: CC on, bladeRF unplugged | Ludifly connected |

Compare: satellites used, C/N0 distribution (NAV-SAT), noisePerMS, agcCnt, jamInd per
band (MON-RF), MON-SPAN peaks (a CW line near 1574.4 MHz points at the bladeRF
reference; a raised floor across the band points at USB 3/digital noise), antenna
status and power (MON-HW), and time to first fix after a power-up in state A vs C.
Also measure the 5 V at the Here4 connector in A, B, C.

**Direct confirmation (user, 22:35).** Drone booted with the CC and SDR unpowered:
20+ satellites and "ready to arm". CC and SDR powered: a few seconds into their
initialisation the GPS glitched and lost satellites. The CC starts the bladeRF stream
by itself at boot (`sdr.auto_start: True`), so this is the disruptor switching on.

Ludifly log 22:26:56 (June2026 firmware, CC powered at about 22:27:00): 22 to 23
satellites with a DGPS fix at 22:26:57; 14 to 17 at 22:27:07; "GPS Glitch or Compass
error" and "PreArm: GPS glitching" at 22:27:19; 13 satellites, 3D only, 7 m at 22:27:27.
**No companion heartbeat and no SDR stream existed yet in that window** (the LattePanda
had not finished booting), and the battery voltage was flat at 23.6 V. So the disruptor is
the CC hardware powering up, not the SDR software or the stream: USB 3 SuperSpeed link
enumeration and the bladeRF's reference oscillator start within seconds of power. Note
that the bladeRF moved from a USB 2 link to a USB 3.0 SuperSpeed link on 2026-10-16
(cable change), a few days before the GNSS complaints began; USB 3 SuperSpeed cabling is
a well-known GNSS L1 jammer.

**Power-up sequence experiments (Ludifly logs 22:41, 22:47, 22:51; June2026 firmware).**

| Sequence | Before CC/SDR | After SDR streaming started |
|---|---|---|
| 22:41 autopilot up and locked; CC on 22:44:05 (SDR not found yet); SDR found and streaming 22:45:18 | 13 to 18 satellites, DGPS, 2.9 m (still improving while the CC alone ran) | satellites flat at 17 to 18, accuracy 2.9 to 4.2 m |
| 22:47 everything powered together; SDR streaming 22:49:25 | 0 satellites for 40 s, then 4 at 52 m | 6 to 8 satellites, 15 m after 2.7 min, DGPS never reached |
| 22:51 autopilot first; locked (15 satellites, DGPS at 22:54:14); CC 22:55:07, SDR 22:55:25 | 15 satellites, 5.0 m | 15 to 13 to 12 satellites, 5.0 to 7.7 m, DGPS lost at 22:56:14 |

Reading: the interference raises the receiver's noise floor continuously. A receiver that is
already tracking keeps its locks (tracking tolerates a few dB less C/N0 than acquisition),
so a locked constellation survives with reduced margin; a cold or warm start under the
interference fails or crawls, because acquisition needs the higher C/N0. The margin loss is
visible even in the "good" sequence: fewer satellites, worse accuracy, SBAS lost. Over a
mission the constellation erodes, because every rising satellite must be acquired under
the interference. In the 22:41 log the LattePanda alone did not hurt (satellites kept
rising for 70 s with the CC up and the bladeRF absent); the degradation began when the
bladeRF and its USB 3 link came up. Powering the autopilot first is therefore a valid
operating procedure for now, but it is a workaround, not a fix.

**Consequence for the firmware A/B (section 12).** The CC was powered during every
"bad" test tonight and Ludifly was closed during the "good" June2026 readings, so the
prime verdict is confounded and must be re-run with the CC off: flash the current
`release-ARRC` (prime, v1.12 blob), power-cycle with the CC off, check for 20+
satellites within two minutes. If that passes, the prime is innocent and the current
firmware stays; the only fix is on the CC side.

**Fixes depending on the outcome.**

- RF: shielded double-screened USB 3 cable with ferrites, shortest route, bladeRF case
  grounded to the frame, a metal plate or shielded bay between the CC/SDR and the GNSS
  antenna, Here4 mast higher or farther. Software mitigations: have the CC not start
  the bladeRF stream until the Cube reports a 3D fix with 20+ satellites (the CC already
  sees GPS_RAW_INT), and stop streaming on a GNSS-degraded signal; both are small
  changes in `rfsonde_cc/mediator.py`.
- Power: separate BEC for the LattePanda/bladeRF, or verify the carrier's 5 V rail is
  not shared.
- Prime: keep `release-ARRC-noprime` as the firmware candidate, but retest it in state
  A and state C so its result is not confounded.

## 14. Root cause and close-out (2026-10-24)

**Root cause (user-confirmed):** EMI from the USB 3 SuperSpeed cable between the LattePanda
and the bladeRF. With a USB 2 cable the Here4 behaves normally; wrapping the USB 3 cable
in conductive tape did not help. Neither Here4 firmware was at fault: the current
`release-ARRC` (CubePilot v1.15.9 + ARRC ECEF code, CubeID v1.12.0) and
`release-ARRC-June2026` (CubePilot v1.15.7-era + ECEF code + week fix, CubeID v1.8.0)
are both good. Sections 11 and 12's firmware verdicts were confounded by the CC being
powered during the "bad" tests.

**Repository state:** GNSSPeriph-release on `release-ARRC` (ee9ec3b717, submodule
`here4-hpposecef` dbc16da7e0) with the current image rebuilt; `release-ARRC-June2026`
(75c4909be2, submodule `here4-hpposecef-June2026` 63f4f27fe2) kept as the fallback. The
test branches `release-ARRC-cubeid-v1.8`, `release-ARRC-noprime` and their submodule
branches were deleted. Desktop `Here4_fw\` holds only the two kept images plus a
README. Nothing pushed.

**Why tape on the cable was not enough.** The SuperSpeed link's scrambled 5 Gbps data is
broadband; the noise leaves through three paths: the cable, the host receptacle and the
device connector. Intel's 2012 white paper names all three and recommends shielding the
peripheral and the host receptacle, not only the cable. A tape wrap over the jacket is
not bonded to the connector shells, so the shield gap at each end, where most of the
leakage is, stays open, and an unbonded wrap can itself re-radiate.

**Mitigation plan, in order of expected effect, keeping SuperSpeed:**

1. **Shielded bay.** Put the LattePanda and the bladeRF in one grounded aluminium
   enclosure with the USB cable entirely inside; only the RF coax (shielded, SMA) and a
   filtered DC feed leave the box; bond the box to the frame ground and the bladeRF case
   to the box. This attacks all three leakage paths at once and also protects the SDR's
   own receiver from the computer. Measured precedent: a full grounded shield over all
   USB 3 parts restored an unusable GPS to near normal (McElhenny), foil shielding gave
   about 50 % SNR gain (Emlid forum).
2. **Cable and connectors.** A certified USB 3.2 Gen 1 cable with braid plus foil and
   metal overmoulded shells (360-degree shield termination), as short as possible,
   ideally with screw-lock connectors; or a rigid right-angle adapter so the bladeRF sits
   on the LattePanda port with no cable at all. Clamp-on ferrites rated for the GHz
   range (NiZn material) at both ends help the common-mode part; they are a supplement,
   not the fix.
3. **Distance and ground plane.** Inverse-square: each doubling of the distance from the
   USB 3 parts to the Here4 buys 6 dB. A taller mast and a metal plate under the Here4,
   between it and the CC bay, are cheap and additive (ETH MAV team and McElhenny both
   relied on placement).
4. **Fibre-optic USB 3 cable (hybrid AOC with copper power).** Removes the cable as a
   radiator; connectors still leak, so combine with item 1. Check the AOC's power
   budget against the bus-powered bladeRF.
5. **Fallback to USB 2 with 8-bit samples.** The CC/Ludifly protocol already carries
   `sample_fmt` (0 = SC16_Q11, 1 = SC8_Q7) but `rfsonde_engine/bladerf_device.py`
   always configures SC16_Q11. Implementing SC8_Q7 halves the USB payload, so the USB 2
   ceiling of about 8 to 10 MSPS becomes roughly 16 to 20 MSPS at 6 dB less dynamic
   range. A software-only option if the hardware fixes fall short.
6. **Operational safeguards regardless:** power the autopilot first and let the Here4 lock
   before the CC and SDR come up; have the CC hold the bladeRF's USB port unpowered
   (`uhubctl`) or the stream stopped until the autopilot reports a 3D fix with 20 or more
   satellites; let Ludifly flag a satellite-count drop when the SDR starts.

**Verification after each step:** `tunnel_raw.py COM14 <step>.json` with the CC on and the
receiver locked; compare jamInd, noisePerMS, AGC, satellites used and C/N0 with the
healthy baseline (`baseline_june2026/`: jamInd 3 to 8, noise 73, 32 used, C/N0 up to
39 dBHz). A cold start under the interference (time to first fix) is the hardest test and
should be the acceptance criterion.

## 15. Sources

- CubePilot GNSSPeriph-release releases page (v1.15.9 and v1.15.8 notes):
  <https://github.com/CubePilot/GNSSPeriph-release/releases>
- Forum: Here4 loses many satellites about 8:20 after power-up:
  <https://discuss.cubepilot.org/t/here4-loses-many-satellites-and-positional-accuracy-8-20min-after-being-switched-on/17408>
- Forum: Here4 + Cube Orange no GPS fix (node driver stuck, 7 used of 26 tracked):
  <https://discuss.cubepilot.org/t/here4-cubepilot-orange-no-gps-fix/17893>
- Here 4 manual (NEO-F9P, L1/L5, max 4 GNSS, passthrough, firmware warnings):
  <https://docs.cubepilot.org/user-guides/here-4/here-4-manual>
- Forum: Here 4 sat count and firmware (batch to batch variation):
  <https://discuss.cubepilot.org/t/here-4-sat-count-and-firmware/15256>
