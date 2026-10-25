# RFSonde 8-bit samples (SC8_Q7): review, live test at 2910 MHz, proposals and plan

Date: 2026-10-25. Code reviewed: RFSonde_Computer `a49a7d6`, `24e6d82`, `8ebead5` (installed on the LattePanda as
`20261024T234328Z_0.1.0_8ebead5`), Ludifly `9400893`. Bench: bladeRF 2.0 micro on USB 2, FPGA 0.16.0, libbladeRF
2.6.0, horn on RX1, KTLX at 2910 MHz, BW 4 MHz, waveform KOUN_SP (16 taps at 10 MSPS), DETECT at 13 dB.
Driven over the Ethernet GCS link (no Ludifly on the link), the CC's configuration restored at the end (2800 MHz).

**Status (2026-10-25, end of day):** P1 (bin centre), P2 (warning) and the docs done: CC `14ec74b`, `32990da`,
`6c93941`, Ludifly `4bc7f21` (exe rebuilt). Checked live on `6c93941`: warning at 20/40 dB only, engine 8-bit DC
+0.015 / +0.013 steps, 8-bit at 50 dB 1.0 dB above 16-bit. Bugs then fixed and checked live on `2b1edda`: RX roles
changed mid-stream (`bd15cc6`), the restart impulse behind R4 (`b9a94e0`, 50 ms settle; P5), the snapshot close
in the engine loop behind R5 (`7072c1e`; P7); docs `2b1edda`, `e3c29c8`. Still open from the plan: the packed format
(P3) and the saturation diagnostic (P4, likely unneeded now that R4 is explained). The "phase offset with no
signal" seen in 8-bit is explained in section 7 (no code change).

Evidence: `/var/tmp/claude-rfsonde-tests/sc8q7/gain_20261025T152018Z/` (live log, `summary.json`, 11 raw snapshots
of 2 s, `analyze2.jsonl`, `fidelity_0004.jsonl`); scripts `gainsweep.py`, `analyze2.py`, `fidelity.py`,
`bench/convert_bench.py` in `/var/tmp/claude-rfsonde-tests/sc8q7/`.

## 1. Summary

- The 8-bit path is implemented correctly: the stream runs at 10, 16 and 20 MSPS on USB 2 with no overruns, and
  truncating a 16-bit capture to 8 bits in software (the FPGA's `data(11 downto 4)`) reproduces the real 8-bit
  capture to within 0.01 dB of noise at 20, 30 and 40 dB gain. What remains is the physics of 8 bits.
- **The noise floor is the problem, as suspected, and it depends on the RX gain.** At the default 20 dB the 8-bit
  step, not the receiver, sets the floor: +10.2 dB over 16-bit. The penalty falls to 7.8 dB at 30 dB, 7.0 dB at 40
  and 2.1 dB at 50 dB. At 20–30 dB the noise estimate is stuck on a single value (p10 = p90), and most 8-bit
  detections are runs of quantization steps, not pulses.
- On a real KTLX main-beam pass (1879 pulses) 8-bit keeps pulse power unbiased above 15 dB SNR and adds
  0.6 dB / 3.5° of scatter at 20–30 dB SNR, 0.2 dB / 1.5° at 30–40 dB. Pulses at 15–20 dB SNR are mostly lost at
  40 dB gain (21 % detected) and kept at about 50 dB gain (99 %).
- At equal sensitivity (8-bit at 50 dB ≈ 16-bit at 40 dB, -74.6 vs -74.8 dBm) 8-bit has about 10 dB less dynamic
  range (55.6 vs 65.3 dB from full scale to the matched-filter noise). On this bench the KTLX main beam would peak
  at -1.3 dBFS at 50 dB gain; in the field that needs a pad 10 dB larger than the 16-bit setup.
- 8-bit is worth it only above what 16-bit carries on USB 2. 16-bit at 10 MSPS already runs with no overruns on
  USB 2. The FPGA (0.16.0) and libbladeRF (2.6.0) also offer **SC16_Q11_PACKED**: full 12 bits in 3 bytes per sample,
  33 % more rate (about 13 MSPS on USB 2) and no noise penalty.

## 2. Code review

Correct as written:

- FPGA truncation confirmed in the HDL: `hdl/fpga/ip/nuand/synthesis/fifo_writer.vhd` `pack_eight_bit_mode` takes
  `data_q(11 downto 4) & data_i(11 downto 4)` (floor). The 8-bit GPIO (`eightbit_en`) drives both RX and TX in
  `bladerf-hosted.vhd`, so "RX and TX in the same format" is a hardware rule, not only a design choice.
- The binding/library enum fix (`_lib_format`, value + 1 for 2.6.0 with the old `_cdef.py`) is right and tested.
- Buffer sizing (`block_n * n_rx * bps`), the per-format dtype and full scale, the peak from the raw words, the
  ADJUSTED echo when the binding lacks SC8_Q7, the restart on a format change, status/`status.json`/`rfsonde-ctl`,
  and HDF5 compatibility (`/gated/iq` stays int16 Q11) are all fine. CPU: the conversion costs 142 µs per
  65536-sample block on the N5105 (4.3 % of the reader core at 20 MSPS).

Findings:

| # | Where | Finding | Evidence | Weight |
|---|---|---|---|---|
| R1 | `bladerf_device.py` `_set_format` (offset `0.5/128`), `sim_device.py` | The bin of 8-bit value k holds the 12-bit integers 16k..16k+15; its centre is 16k + 7.5, i.e. k + 7.5/16, not k + 0.5. The extra 1/32 step leaves a DC of 0.5·P(x = 0). | Live 8-bit DC +0.031 / +0.047 steps (I/Q) at 40 dB, +0.037 / +0.051 at 20 dB. Emulated with 7.5/16: 0.000 / +0.015, DC bin -101 → -106..-110 dBFS/Hz, threshold crossings in noise at 20 dB gain 48 → 6 per s, power scatter 0.60 → 0.55 dB at 20–30 dB SNR. File words unchanged (16k + 7.5 rounds to 16k + 8). | low |
| R2 | engine / Ludifly / docs | Nothing guides the RX gain in 8-bit: at the default 20 dB the floor is quantization (+10 dB), the noise estimate is meaningless and detections at 13 dB are mostly artifacts (live: "-37 dBm, SNR 13.6" in 8-bit vs -47.4 dBm in 16-bit, same scene). The docstring, the Ludifly tooltip and DEVELOPMENT.md say "6 dB less dynamic range"; 4 bits fewer is 24 dB of quantization range, measured 7–10 dB of noise floor at the gains in use. | Section 3 tables. | **high** |
| R3 | `measure.py` noise median | In 8-bit at ≤ 30 dB gain the matched-filter power takes few discrete values; the median (÷ ln 2) locks to one of them: live noise p10 = p90 = -50.8 dBm (20 dB) and -60.8 dBm (30 dB), and the same -61.62 dBFS in three different captures. Gone at 40–50 dB (0.5 dB spread, like 16-bit). | Live `noise_dbm`, `analyze2.jsonl`. | medium (fixed by R2) |
| R4 | engine saturation flags | A sporadic per-pulse SATURATED in 8-bit at 16–20 MSPS while the period's peak reads -30 to -33 dBFS (one measurement in 750, two of three runs). Explained later the same day: every stream restart puts one impulse 8–11 ms in (up to -0.9 dBFS at 20 MSPS), only retunes were skipped, and DETECT's 10-pulse average carried that pulse into a measurement seconds later. | `meas_8b_16M_g40.jsonl` line 338, `run_20261024T234436Z/meas_8b_20M.jsonl`, `transient_20261025T160631Z`. | low, fixed (`b9a94e0`) |
| R5 | `engine.py` snapshot | A 2 s snapshot at 16–20 MSPS costs about 2 overrun blocks (the engine concatenates 2 s in its loop); none without snapshots. | `overruns_total` 2 and 4 outside the measurement windows. | low |
| R6 | docs/VERIFICATION.md (uncommitted 4p text) | "At 30 dB the levels read about 7 dB low … calibration" is not supported: at 20 dB gain the bench measurements are mostly threshold-level picks. A 2 s 16-bit capture at 20 dB has 360 crossings/s, max SNR 21.8 dB, none ≥ 25 dB, random phase; the median level is noise + 14 dB at every gain. Real KTLX passes show only in p90. | `analyze2.jsonl` file 0000, live tables. | doc fix |

## 3. Measurements

### Noise floor against RX gain (live engine, 25 s per case, 10 MSPS)

| RX gain | 16-bit noise dBm (p10..p90) | 8-bit noise dBm (p10..p90) | 8-bit penalty |
|---|---|---|---|
| 20 dB | -61.0 (-61.4..-60.9) | -50.8 (-50.8..-50.8, locked) | 10.2 dB |
| 30 dB | -68.6 (-68.7..-68.4) | -60.8 (-60.8..-60.8, locked) | 7.8 dB |
| 40 dB | -74.8 (-74.9..-74.5) | -67.8 (-68.3..-67.8) | 7.0 dB |
| 50 dB | -76.7 (-76.9..-76.5) | -74.6 (-74.8..-74.3) | 2.1 dB |

dBm use the nominal calibration (K = -gain), so only the same-gain comparison is exact.

### Raw snapshots (2 s each, offline matched filter, KOUN_SP)

| RX gain | 16-bit MF noise dBFS | 8-bit MF noise dBFS | 16-bit in-band PSD dBFS/Hz | 8-bit in-band PSD | 8-bit levels I/Q |
|---|---|---|---|---|---|
| 20 dB | -71.7 | -61.6 | -135.5 | -121.2 | 8 / 4 |
| 30 dB | -69.2 | -61.6 | -131.7 | -120.5 | 6 / 4 |
| 40 dB | -65.3 | -58.6 | -126.9 | -119.1 | 6 / 4 |
| 50 dB | -57.4 | -55.6 | -118.5 | -115.9 | 8 / 6 |

The out-of-band noise near Nyquist (|f| > 4.5 MHz) stays at -108 dBFS/Hz at every gain in both formats. It sets
the sample spread (8–11 counts on I, 4–8 on Q, 0.3–0.7 of an 8-bit step) and is what dithers the 8-bit quantizer.
The in-band receiver noise only reaches the 8-bit step's noise (about -120 dBFS/Hz at 10 MSPS) near 50 dB gain.

### Pulse fidelity on a KTLX main-beam pass (16-bit capture at 40 dB, 1879 pulses, 8-bit emulated)

Offset 0.5 → 7.5/16 where they differ.

| SNR in 16-bit | Pulses | Power bias | Power scatter | Phase scatter | Detected in 8-bit at 13 dB |
|---|---|---|---|---|---|
| 10–15 dB | 90 | +0.4 dB | 2.8 → 2.5 dB | 17.7 → 15.9° | 0 % |
| 15–20 dB | 125 | +0.1 dB | 1.4 → 1.3 dB | 8.6 → 8.0° | 21 % → 71 % |
| 20–30 dB | 761 | 0.0 dB | 0.60 → 0.55 dB | 3.5 → 3.2° | 97 % → 100 % |
| 30–40 dB | 750 | 0.0 dB | 0.23 dB | 1.5° | 100 % |
| > 40 dB | 153 | 0.0 dB | 0.05 dB | 0.3° | 100 % |

The same pulses scaled +10 dB (≈ 50 dB gain): 0.18 dB / 1.3° at 20–30 dB SNR, 99–100 % of the 15–20 dB pulses
detected, peak -1.3 dBFS. The 21 → 71 % detection change comes partly from where the locked median lands (R3); at
+6 dB the order reverses (98 % vs 91 %).

### Throughput (live, 40 dB gain, DETECT 13 dB, 25 s)

| Case | Overruns | Engine load (% of real time) | Engine core | Reader core |
|---|---|---|---|---|
| 16-bit 10 MSPS | 0 | 22–25 % | 27–28 % | 8 % |
| 8-bit 10 MSPS | 0 | 22 % | 28 % | 7 % |
| 8-bit 16 MSPS | 0 (2 at the snapshot) | 47 % | 53 % | 12 % |
| 8-bit 20 MSPS | 0 (2 at the snapshot) | 66 % | 74 % | 15 % |
| 16-bit 16 MSPS (10-24 runs) | 227 in 40 s, 6.3 % lost | 46–48 % | | |

At 20 dB gain the 8-bit 20 MSPS load was 85–93 % because the quantization runs feed the detector; at 40 dB, 66 %.
Conversion on the N5105 per 65536-sample block: int16 96 µs, int8 (now) 142 µs, numba lookup table 86 µs,
`np.take` lookup 491 µs.

## 4. Proposals

| # | Change | Gain | Cost / risk |
|---|---|---|---|
| P1 | Offset 7.5/16 of a step (`_set_format`, simulator, tests, README/docstring wording). | Removes the residual DC and its spur, fewer false crossings at low gain, slightly less scatter. | Small; files unchanged; unit tests updated. |
| P2 | Gain guidance for 8-bit: (a) CC STATUSTEXT warning when `sample_fmt` 1 is applied with RX gain below 45 dB ("8-bit: the 8-bit step sets the noise floor at this gain (user setting)"), (b) Ludifly pre-check note and a corrected tooltip, (c) DEVELOPMENT.md/OPERATIONS.md: penalty table, "raise gain and pad together". No automatic gain change: the 20 dB default is there for strong sources. | Prevents the silent 10 dB loss and artifact detections. | Small; wording per the user's error-category rule. |
| P3 | SC16_Q11_PACKED as `sample_fmt` 2: FORMATS entry (host words stay int16 Q11: libbladeRF unpacks in `sync_rx`), library value 1 via the existing `_lib_format` path, block size a multiple of 4096 (65536 is), validation, Ludifly combo, ARRC dialect enum value. | Full 12 bits at about 13 MSPS on USB 2: covers 10–13 MSPS with no noise penalty. | Dialect change (my-mavlink pin, Ludifly), reader CPU for the C unpack (to measure); sync interface only (we use sync). |
| P4 | Diagnostic for R4: when a pulse window is flagged saturated, log the raw min/max of its block and its offset (rate-limited), or take a short snapshot. | Explains or removes the sporadic SATURATED. | Small, diagnostic only. |
| P5 | Settle window after a stream restart (the retune path already skips 20 ms). | Keeps first-block transients out of the feed; a candidate cause for R4. | Very small. |
| P6 | Fused numba int8 conversion + peak in the reader. | ~70 µs per block: 2 % of a core at 20 MSPS. | Not worth it now; listed for completeness. |
| P7 | Snapshot concatenation outside the engine loop (or cap it at the ring length when ≥ 16 MSPS). | Removes the 2 overrun blocks per 2 s snapshot at high rates. | Small; diagnostic path only. |

Not proposed: digital gain in the AD9361 before the FPGA truncation (it adds nothing over analog gain) and
changing the noise estimator (R3 disappears at the gains 8-bit needs, and a new estimator would change the
reference outputs).

## 5. Plan

1. **Decide the rate need** (user): if 10 MSPS is enough, keep 16-bit (works on USB 2 today) and treat 8-bit as a
   high-rate option only. If 10–13 MSPS is needed, P3 (packed). Above about 13 MSPS, 8-bit at 45–50 dB gain with
   the pad raised by the same 10 dB.
2. **P1 + P2 + R6 docs** (CC and Ludifly): offset, warning, tooltip/manual, DEVELOPMENT/OPERATIONS rows,
   VERIFICATION 4p rewritten with section 3 of this report. Gate: CC suite + ruff, Ludifly suite.
3. **P5 + P4 + P7** (CC): settle after restart, the saturation diagnostic, snapshot outside the loop. Gate: CC
   suite.
4. **P3** if step 1 asks for it: dialect enum value in my-mavlink `ArduPilot-4.7-ARRC`, pins in CC and Ludifly,
   then CC + Ludifly. Gate: both suites.
5. **Release and install** (build `~/RFSonde_Computer`, scp, the user runs `sudo rfsonde-ctl update`), then
   **re-run** `gainsweep.py` (add packed cases at 12/13/14 MSPS) and `fidelity.py`. Pass criteria: 8-bit DC within
   ±0.01 step; no locked noise at ≥ 40 dB; 8-bit penalty ≤ 2.5 dB at 50 dB gain; packed noise within 0.3 dB of
   16-bit and no overruns at 13 MSPS on USB 2.
6. **Field check before flying 8-bit**: one KTLX pass at the chosen gain and pad; peak below -6 dBFS and the
   sidelobe-level pulses detected. Compare with the R6 field numbers (40 dB gain, 30 dB pad, peak -4.2 dBFS).

## 6. Left on the CC

Test snapshots, all erasable from Ludifly's Data tab: sessions `1792941247` (the gain sweep, 968 MB),
`1792944023` (the check of `6c93941` and the restart captures, 321 MB) and `1792945350` (the checks of `2b1edda`,
519 MB) in `/data/rfsonde/20261025/`, and the two 50 ms files of 10-24 in `/data/rfsonde/20261024/1792885444/`.

## 7. Phase of the noise picks (the "phase offset" with no signal)

A measurement without a detection carries no phase (Ludifly plots a gap). The co-pol phases shown when no radar
pulse is there are those of noise that crossed the 13 dB threshold, each the angle of 10 picks summed coherently
(`phi_c`, phase reference SELF). That angle should be random; in 8-bit at low gain it is not:

| Case | Noise-pick phase | R of the 10-pick average (0 random, 1 fixed) |
|---|---|---|
| 8-bit, offset 0.5 (`8ebead5`), 20–30 dB | 44–76°, along its DC | 0.97–1.00 |
| 8-bit, bin centre (`6c93941`), 20–40 dB | fewer picks, still clustered (live: -44°, -103°) | 0.4–1.0 |
| 8-bit, 50 dB | random | 0.17 (single picks 0.01) |
| 16-bit, 20 dB | 0° or 180° | 0.69 |
| 16-bit, 40–50 dB | random | 0.05–0.2 |

Cause: at 20–40 dB the receiver's in-band noise (below 2 MHz) is 0.5–1.4 counts of 12 bits, 0.03–0.09 of an
8-bit step, and circular (|E[x²]| / E[|x|²] 0.04). The sample spread comes from out-of-band noise rising toward
Nyquist, 7 dB stronger on I than on Q (non-circularity 0.65). Truncating to 8 bits folds that I-heavy noise into
the band as a coarse, non-circular pattern, so the strongest noise outputs of the matched filter come from
preferred directions; a DC tilts them further, and the coherent 10-pick average turns the lean into a fixed angle.

Tested on the 2 s captures (`noisephase.py`): removing the DC exactly, or a 2 MHz low-pass before the matched
filter, does not make the 8-bit pick phases random (R 0.57–0.99): the pattern is already in the samples the FPGA
sends. The low-pass does fix the 16-bit lean at 20 dB (R 0.31 → 0.01 for single picks). Real pulses are not
affected: on the KTLX pass the 8-bit phase error averages at most 1.2° at every SNR, with the scatter of the
fidelity table. Use 8-bit at 45–50 dB of gain; a steady co-pol phase with no signal means noise picks, not a
source. No code change.
