# Proposal: a separate RF data stream from the autopilot to Ludifly (2026-10-23)

Question: what would it take to stream the ARRC-SDR messages to Ludifly at 30 Hz in their own stream, separate
from EXTRA3, while everything else stays at default rates, to save bandwidth on the 57600 Herelink?

Nothing has been changed yet. This note covers the investigation and the proposed changes.

## 1. Short answer

- **Taking the RF messages out of EXTRA3 is a 2-line firmware change.** ArduPilot 4.7 already lets one message
  have its own rate (SET_MESSAGE_INTERVAL), and 227/228 are already mapped for it.
  - A message that is in no stream group cannot be changed by any REQUEST_DATA_STREAM, including Ludifly's
    "ALL = 1" or another GCS. Only a per-message interval controls it.
  - A boot-time default with no GCS request at all needs no code: a one-line text file on the SD card
    (`message-intervals-chan1.txt`). That feature is compiled into the CubeOrangePlus-ODID firmware.
  - A real new stream group with its own `MAVn_ARRC` parameter is possible, but it changes core GCS code
    (section 4, option B).
- **Bandwidth.** The saving does **not** come from EXTRA3 in normal operation.
  - In the real 10-22 Herelink capture, Ludifly already set the RF rate per message, and the other EXTRA3
    members ran at 1 Hz (235 B/s).
  - The link was 88 % full because Ludifly asks for whole groups at 4-10 Hz. That drags in **1120 B/s of
    messages Ludifly never reads** (AHRS2, SCALED_IMU2/3, SCALED_PRESSURE2, NAV_CONTROLLER_OUTPUT, MCU_STATUS,
    MEMINFO, POWER_STATUS, TERRAIN_REPORT, DISTANCE_SENSOR, AHRS).
  - Requesting only what Ludifly uses, at sensible rates, takes the link from **90 % to about 55 % with RF at a
    full 30 Hz**. That saves about 2.0 kB/s (section 5).
- **Where EXTRA3 does hurt is a failure mode, and it has already happened.** Your Ludifly log on 2026-10-16
  20:18 over the Herelink shows three SET_MESSAGE_INTERVAL ACKs lost:
  ```
  SET_MESSAGE_INTERVAL(228) not accepted (No COMMAND_ACK ... after 3 attempts); using the EXTRA3 stream
  RF stream rate 30 Hz (extra3)
  ```
  That fallback asks for about 9 stock messages at 30 Hz, around 8.9 kB/s, more than the whole link. Taking
  227/228 out of EXTRA3 and removing the fallback closes this for good.

## 2. How ArduPilot 4.7 schedules telemetry (the parts that matter)

- **Stream groups.** Each group (RAW_SENS, EXT_STAT, RC_CHAN, RAW_CTRL, POSITION, EXTRA1, EXTRA2, EXTRA3,
  PARAMS, ADSB) is a fixed list of messages with one rate parameter per port, `MAVn_<group>`
  (`libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp`).
  - On Copter every default is **0**, and your Cube has all `MAV1-4_*` at 0. So the "defaults" on the
    Herelink are whatever Ludifly asks for.
- **REQUEST_DATA_STREAM (group, rate)** sets the interval of every member of that group
  (`GCS_Common.cpp:6914-6928`), overwriting any per-message interval set earlier.
  - `MAV_DATA_STREAM_ALL` does this for every group except PARAMS (`GCS_Param.cpp:147-160`).
  - Copter does not save these rates (`persist_streamrates()` is false).
- **SET_MESSAGE_INTERVAL (msg id, interval)** sets one message on one port. It works for any message in
  `mavlink_id_to_ap_message_id()`, and 227 and 228 are there (`GCS_Common.cpp:1208-1211`).
  - Rates reset on an autopilot reboot.
  - Rates are overwritten by a later group request only if the message belongs to that group.
- **Default intervals from files.** At boot each port reads `@ROMFS/message-intervals-chanN.txt`, then
  `message-intervals-chanN.txt` from the SD card root (`GCS_Common.cpp:7031-7104`).
  - Each line is `<mavlink msg id> <interval ms>`.
  - The file also defines what "reset to default" (interval 0) means.
  - It is compiled into CubeOrangePlus-ODID (`DefaultIntervalsFromFiles` is in the binary).
  - N is the MAVLink channel: on your Cube, chan0 = SERIAL0 USB (companion), **chan1 = SERIAL1 Herelink**,
    chan2 = SERIAL2 T7, chan3 = SERIAL5 ADSB.
- **Scheduler.** Each port has 10 rate buckets (`GCS.h:900`).
  - Every bucket is slowed 4x while parameters, mission items or FTP are transferring
    (`GCS_Common.cpp:1262-1290`). This applies to the RF message too, whatever group it is in.
- **228 data handling.** ARRC_SENSOR_RAW sends each companion measurement at most once per port, and only while
  it is less than 1 s old (`AP_ARRC_SDR.cpp:348-369`). The bytes are therefore capped by the companion's feed
  rate, not by the request rate.

## 3. What happens today

| Path | What Ludifly sends | Effect on the Herelink |
|---|---|---|
| Connect | REQUEST_DATA_STREAM ALL = 1 Hz, then RAW_SENS 4, EXT_STAT 4, RC_CHAN 4, RAW_CTRL 4, POSITION 10, EXTRA1 10, EXTRA2 4 (`mavlink_rate_manager.py:222-372`); ADSB_VEHICLE 4 Hz by interval | EXTRA3 at 1 Hz; every member of the other groups at 4-10 Hz |
| RF rate, normal | SET_MESSAGE_INTERVAL 228 and 227 at the RF rate (`RFSonde.py:2956-2992`) | Only 228/227 change (27 Hz delivered at 30 requested) |
| RF rate, fallback after 3 lost ACKs | REQUEST_DATA_STREAM EXTRA3 at the RF rate (`mavlink_rate_manager.py:389-427`) | ~9 stock messages at 30 Hz. Nothing resets them until a reconnect. Seen on 10-16 |
| Gimbal window | SET_MESSAGE_INTERVAL GIMBAL_DEVICE_ATTITUDE_STATUS 5 Hz, GIMBAL_MANAGER_STATUS 1 Hz | GDAS is an EXTRA3 member, so a later ALL = 1 drops it to 1 Hz and the fallback raises it to 30 Hz |

Real Cube + Herelink measurement, 10-22, RF 30, MAV3_OPTIONS 2
(`/var/tmp/claude-rfsonde-tests/linkstale/nofwd_rf30.json`):

- total 5060 B/s = 87.8 % of 5760 B/s;
- ARRC_SENSOR_RAW 27.0 Hz = 1270 B/s (47 B each on the wire);
- GLOBAL_POSITION_INT and ATTITUDE 400 B/s each;
- GIMBAL_DEVICE_ATTITUDE_STATUS 305;
- AHRS2 279;
- RC_CHANNELS 216;
- the 4 Hz EXT_STAT/RAW_SENS members about 1.3 kB/s;
- routed companion traffic 203 B/s.

## 4. Proposed changes

### Firmware (my-ardupilot, ArduPilot-4.7-ARRC)

**F1. Take the RF messages out of EXTRA3 (recommended, minimal).** Delete these lines in
`libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp:374-377`:

```c
#if APM_BUILD_COPTER_OR_HELI
    MSG_ARRC_SDR,
    MSG_ARRC_SENSOR,
#endif
```

- The interval map entries stay, so SET_MESSAGE_INTERVAL keeps working.
- No REQUEST_DATA_STREAM can raise, lower or reset the RF stream any more, from Ludifly, Mission Planner or
  anything else.
- This also closes review finding L5: the `AP_ARRC_SDR_ENABLED=0` build would no longer have an unhandled
  EXTRA3 member.
- Update the `MAVn_EXTRA3` @Description (it does not list them today anyway) and the companion/Ludifly docs.

**F2. A default RF rate that needs no GCS request. Pick one:**

- **(a) SD-card file (no code).** Put `message-intervals-chan1.txt` in the SD card root:
  ```
  228 25
  ```
  - 25 ms means "poll at 40 Hz". See F3 for why this does not cost more than 30 Hz.
  - It applies at every boot, to the Herelink port only. The USB companion port (chan0) gets no 228.
  - Ludifly's SET_MESSAGE_INTERVAL still overrides it, and interval 0 returns to it.
  - Add `227 250` only if the LB5900 is used.
  - Risk: a replaced or missing SD card drops the default silently (Ludifly still requests it). Check on the
    bench that the root path is the one ChibiOS FATFS uses.
- **(b) A real stream group: `STREAM_ARRC` with a per-port `MAVn_ARRC` parameter** (what you described). About
  40 lines in core GCS:
  - `GCS.h`: add `STREAM_ARRC` before `NUM_STREAMS` (Copter only).
  - `GCS_MAVLink_Parameters.cpp`:
    - add a `default_rates` entry (0);
    - add `AP_GROUPINFO("_ARRC", 19, ...)` with docs. Use index 19, the last slot before `_OPTIONS` (20), to
      stay clear of the next stream upstream adds at 11;
    - add `STREAM_ARRC_msgs[] = { MSG_ARRC_SENSOR, MSG_ARRC_SDR }`;
    - add `MAV_STREAM_ENTRY(STREAM_ARRC)`.
  - `GCS_Param.cpp`:
    - skip `STREAM_ARRC` in the `MAV_DATA_STREAM_ALL` loop, as PARAMS is skipped, so a GCS "ALL" never touches
      it;
    - add `case MAV_DATA_STREAM_ARRC`.
  - `arrc.xml`: add `<enum name="MAV_DATA_STREAM"><entry value="200" name="MAV_DATA_STREAM_ARRC"/></enum>`.
    This is an enum addition only, so the message CRCs and the dialect hash 0x7EFB3788 do not change.
  - Benefit over (a): a persistent knob visible in Mission Planner (`MAV2_ARRC 30`, `MAV1_ARRC 0`), with no SD
    card dependency.
  - Costs:
    - it touches core library tables, which is more to carry on every upstream merge;
    - one new parameter per port;
    - `@RebootRequired` like the other stream parameters, unless also applied live.

  Recommendation: F1 + F2(a) now. Do (b) only if you want the parameter knob.

**F3. RF data handling in the firmware:**

- Request 228 faster than the companion's feed, for example a 25 ms interval against a 30 Hz feed.
  `send_sensor_raw` never repeats a sample, so the bytes stay capped at the feed rate. Faster polling recovers
  the samples the 33 ms poll misses (27.0 of 30 Hz delivered on 10-22). Check on the bench that the
  remaining gap is aliasing and not tx space.
- CASS_SENSOR_RAW (227): send only on a new, healthy LB5900 sample (review M1). Today, in LB5900/BOTH mode,
  it repeats a ~4 Hz value at the RF rate: 38 B x 30 = 1.1 kB/s, where ~150 B/s would do. In Auto with the SDR
  detected it is already off.
- Optional (review M6): keep the latest ARRC_SDR_STATUS/ARRC_CC_STATUS from the companion and re-send it at
  1 Hz from the scheduler when there is room, instead of relying only on routing, which drops when the link is
  full. Only worth it if statuses still go stale after L2.

### Ludifly (RFSonde_GUI, Ludifly-SDR)

**L1. Remove the EXTRA3 fallback for the RF rate:**

- After a missing ACK, keep retrying SET_MESSAGE_INTERVAL in the background (for example every 5 s).
- Judge success by the **measured 228 rate**, not by the ACK.
- Fall back only on an explicit DENIED/UNSUPPORTED from firmware without the 228 mapping, and say so in the UI.
- If a fallback was ever used, send EXTRA3 back to 1 Hz.

**L2. Request only what Ludifly reads, per message ("Herelink profile").** Replace the group requests,
including the ALL = 1 wake-up (unnecessary on ArduPilot: a per-message interval starts the message by
itself), with SET_MESSAGE_INTERVAL for exactly the messages its handlers use (`RFSonde.py:366-394`):

| Message | Proposed Hz | Use in Ludifly |
|---|---|---|
| ARRC_SENSOR_RAW | 30 (poll 40) | RF plots |
| ATTITUDE | 10 | HUD |
| GLOBAL_POSITION_INT | 5 | map, altitude |
| GIMBAL_DEVICE_ATTITUDE_STATUS | 5 | gimbal window, HUD needle |
| VFR_HUD | 4 | HUD |
| SYS_STATUS, GPS_RAW_INT | 2 | battery, GPS fix |
| MISSION_CURRENT, BATTERY_STATUS, EKF_STATUS_REPORT, VIBRATION, SYSTEM_TIME, RAW_IMU (temperature), SCALED_PRESSURE, SERVO_OUTPUT_RAW, RC_CHANNELS, GIMBAL_MANAGER_STATUS | 1 | status panels, logger |
| ADSB_VEHICLE | 4 (as now) | traffic |
| LOCAL_POSITION_NED | 0 or 1 | not sent on the real link today |

- Keep the table in one place and give each rate a "Herelink" and a "USB/Wi-Fi" value, so a fast link can
  still get 10 Hz everywhere.
- Request messages fire-and-forget, then re-request the missing ones from measured rates.
- Re-send everything after an autopilot reboot (already detected) and on reconnect.
- ~20 COMMAND_LONGs at connect is about 1 kB uplink, negligible.

### Optional phase 2: pack the RF samples (dialect change)

- **New message** `ARRC_SENSOR_BATCH` (next free id, 47015) carrying up to 4 samples with scaled int16 fields:
  - the base sample time (uint64);
  - a per-sample time offset;
  - pwr_c/pwr_x in 0.01 dB;
  - phi_c/phi_x in 0.01 deg;
  - snr in 0.01 dB;
  - flags, n_avg.
  - INT16_MAX = invalid.
- **Wire size:** 16 B per sample instead of 36 B + 12 B overhead per message.
- **Bandwidth at 30 samples/s:** 3 per batch at 10 Hz is about 850 B/s (vs 1410); 6 per batch at 5 Hz is about
  585 B/s. The cost is 100-200 ms of extra display latency.
- **Cost:** a dialect change (new id, new dialect hash), so the firmware, companion and Ludifly pins move
  together.
- Only worth it if the link is still tight after L2.

## 5. Link budget, real 10-22 sizes (bytes on the wire per message, measured)

| Case | B/s | % of 57600 |
|---|---|---|
| Today, RF 27 Hz delivered | 5060 | 87.8 |
| Today, RF at a full 30 Hz | 5200 | 90.3 |
| Today, after the EXTRA3 fallback at RF 30 | demand about 8.9 kB/s above the 1-Hz EXTRA3 load | > 100 (saturated) |
| F1 + L1 + L2 (table above), RF 30 Hz | 3198 | 55.5 |
| ... plus phase-2 batching (3 per 10 Hz) | about 2640 | about 46 |

The 3198 B/s breaks down as:

- RF 1410;
- ATTITUDE 400;
- GDAS 305;
- GLOBAL_POSITION_INT 200;
- VFR_HUD 116;
- routed companion traffic 203;
- the rest under 100 each.

Calculation: `/var/tmp/claude-rfsonde-tests/linkstale/nofwd_rf30.json` per-message sizes times the proposed
rates.

## 6. Compatibility and risks

- **Old Ludifly + new firmware (F1).**
  - The normal RF path still works.
  - The old fallback would raise EXTRA3 **without** delivering RF, so ship L1 with F1.
- **New Ludifly + old firmware.** Per-message intervals for the standard messages work on stock 4.7 and on
  today's ARRC build.
- **A second GCS (Mission Planner) on the same Herelink channel** can still re-request groups and change the
  standard messages. After F1 it can no longer touch the RF stream.
- **The Herelink's own GCS app.** It shows up on the link as 42/190, heartbeating as a GCS. If it asks for
  groups, it raises the standard messages on the shared SERIAL1 channel whatever Ludifly asks for. Check a
  tlog of the bench link for REQUEST_DATA_STREAM from sysid 42 before trusting the L2 numbers.
- **GCS failsafe** depends only on HEARTBEAT, which is always 1 Hz and not part of any group. It is unaffected.
- **Rate buckets.** The profile uses 6 distinct intervals (25, 100, 200, 250, 500, 1000 ms), within the
  10-bucket limit.
- **Parameter and mission downloads** still slow every stream 4x, the RF stream included (stock behaviour).

## 7. Test plan

1. SITL ARRC on its own instance with the 57600 back-pressure proxy (`/var/tmp/claude-ap-review/work/r2-integrated/`).
   Run three cases: today's Ludifly request set, the fallback case, and the L2 profile. Log per-message rates and
   bytes, routed CC/SDR status rates and command ACK latency. The L2 profile also stops requesting the
   SITL-only SIMSTATE and ESC_TELEMETRY, so SITL numbers become comparable to the bench.
2. Prove that REQUEST_DATA_STREAM ALL/EXTRA3 after F1 no longer changes 228. Also check the SD-card default
   (`message-intervals-chan1.txt`) in SITL (the file sits next to the eeprom) and on the Cube.
3. Bench: real Cube + Herelink + T7 + LattePanda with `linkstale/link_stale.py` at RF 10/30, before and after.
   Target: link at or below 60 %, both statuses at 1.0 Hz, 228 at 29-30 Hz.

## 8. Effort and order

| Step | Where | Size |
|---|---|---|
| F1 | firmware | 1 commit, -4 lines + doc |
| F2(a) | SD card + operations doc | 1 file, no code |
| L1 + L2 | Ludifly `RFSonde.py`, `mavlink_rate_manager.py` + tests | ~150-200 lines |
| F3 (227 on new samples) | firmware, together with review M1 | small |
| F2(b), if wanted | firmware core + arrc.xml enum | ~40 lines + pin of the regenerated headers |
| Phase 2 batching, if wanted | dialect + firmware + Ludifly + companion pins | medium |
