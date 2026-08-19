# echosight-sbc

FMC acquisition, compression and uplink for the Socomate SPIKE 32:64. Runs on the
PICO-RAP4 SBC inside the robot.

This service **captures and transmits**. It does not reconstruct. TFM,
measurement, classification and imaging moved to the Windows compute server; the
`internal/tfm` package and the imaging/export paths are gone from this tree.

## What was removed, and where it went

| Removed | Lines | Now lives on |
|---|---|---|
| `internal/tfm/` (engine, grid, classify, water, dsp) | 2747 | compute server |
| `internal/device/imaging.go` (live TFM worker) | 1017 | compute server |
| `internal/device/export_tfm.go` (PNG/JSON export) | 201 | compute server |
| `GET /api/tfm/last`, `GET /api/tfm/export/frame`, `POST /api/tfm/motion` | — | compute server |
| `Supervisor.CalibrateVelocity` (thickness-based) | — | compute server |

Added in their place:

| Added | Purpose |
|---|---|
| `internal/compress/` | the four-stage FMC chain + the SBC→server frame envelope |
| `internal/uplink/` | TCP sender with a bounded ring and an explicit drop policy |
| `internal/device/pipeline.go` | worker at the seam `imaging.go` used to occupy |
| `GET /api/uplink` | compression and transmit health |

Everything else is untouched: the SI5G cgo bindings, the FMC read loop and its
hard-won transfer semantics, configuration and apply, capture-to-file, presets,
pose, and the TUI.

## Running

```
echosight-sbc --uplink 10.0.0.20:9200 --headless
echosight-sbc                       # bench mode: compress, time it, discard
```

`--uplink` empty is a deliberate bench mode. The full chain runs and reports
throughput, but frames are discarded, so you can exercise and time the DSP
without the compute server existing.

Other flags are unchanged: `--listen`, `--presets`, `--captures`, `--group`,
`--serial`, `--pose`, `--open-timeout`, `--auto-resume`, `--log-file`,
`--exit-on-hang`.

## Compression

Four stages. Stages 1–3 are FMC-specific and do the work that makes stage 4
possible; on raw RF a general-purpose coder achieves almost nothing.

1. **Quadrature demodulation** — mix against cos and sin at the 5 MHz carrier.
   Produces sum and difference terms; keep the one at DC because slow content
   decimates. I and Q are both required: with the carrier gone, phase has
   nowhere else to live. Output is scaled ×2 to restore the ½ from the
   product-to-sum identity.
2. **Polyphase decimating FIR** — 65 tap, 3 MHz cutoff, D=6, evaluated only at
   surviving samples. Deletes the 2×carrier image *and* band-limits for
   decimation. The cutoff must sit well below the carrier or the image leaks
   back; `Params.Validate` enforces this.
3. **Requantise** — 11 real bits in 16 (peak 11426 counts, floor 5.5). 16, 12
   and 10 bit give identical reconstruction error.
4. **Entropy coding** — see the benchmark status below.

Measured ratios from 122 MB/s ingest, 800-point int16 baseline:

| config | ratio | MB/s |
|---|---|---|
| D=4, 12-bit, packed only | 2.67× | 46 |
| D=4, 12-bit + entropy | 3.68× | 33 |
| D=6, 12-bit + entropy | 5.44× | 22 |
| D=6, 10-bit + entropy | 7.57× | 16 |

Decimating by 4 nets only 2× because demodulation doubles the streams.
2 × 1.33 = 2.67, matching the measurement. **A ratio that comes out at a clean
4× means the Q channel was dropped and phase is gone.**

Reconstruction **must use cubic interpolation**. Linear rebuild gives 8.28 %
error at D=6 versus 4.85 % cubic — the error is the interpolator, not the
compression. D=6 was rejected once purely because it was judged with a linear
rebuild.

Note the sample rate: FMC firmware digitizes at a fixed **50 MHz**, not 100.
There is no sampling code to choose. `DefaultParams` reflects this.

## Codec status — read before choosing

- **Measured:** zlib-1 at 51 MB/s, below the 122 MB/s ingest rate. This is why
  it is disqualified. Measured on a development machine.
- **Not measured:** zstd-1 or LZ4 throughput, anywhere.
- **Not measured:** anything at all on the actual PICO-RAP4.

The bring-up default is therefore `CodecNone`, which yields 2.67× from stages
1–3 alone — about 46 MB/s, comfortably inside the link budget — and removes the
unbenchmarked variable from board bring-up. Swap it once a codec has been
measured on this hardware at 12 W with the DSP on the E-cores. The decision
procedure is written out in `internal/compress/entropy.go`.

## Open items

- **TCP vs UDP.** `internal/uplink` implements TCP; the reasoning and the case
  for UDP are documented at the top of that file. The deciding question is what
  should happen when the compute server cannot keep up, and the current answer
  is "block loudly rather than lose silently".
- **Encoder verification.** `cycleMotion` reads four 32-bit encoder words from
  the FMC frame header. The layout is documented in `acquire_fmc.go` and the PA
  header definitely carries them, but this has not been confirmed with the arm
  actually moving. Motion compensation downstream is only as good as this
  field, and zeros would look exactly like a stationary arm.
- **DMA size.** Drop `initialize_data_transfer` from 32 MB to 1–4 MB. 32 MB is
  313 ms of pointless buffering and the largest latency term in the chain.
- **Core pinning.** P-cores for the read loop and network write, E-cores for the
  DSP. Done in the systemd unit (`CPUAffinity`), not in code — Go offers no
  portable way to pin a goroutine and pretending otherwise would mislead.
- **Compression validation on real defects.** All validation so far is flat
  machined plate. Re-run D=6 + cubic on `2-bottom.txt` and a corroded capture,
  checking hole ceiling depths and layer outputs. Compression that preserves a
  flat backwall but smears a 2.5 mm ceiling passes every current test.

## The drop counters

`GET /api/uplink` reports `dropped`, rolling up two counters: the compression
queue and the uplink ring. **Either being non-zero is a defect, not a tuning
opportunity.** Both paths drop rather than block because blocking stalls the
SI5G read loop, and a stalled reader turns into short reads, then into the
`ReadPipe2` failure modes documented at the top of `acquire_fmc.go`, then into a
wedged board needing a reset. The loss is bounded and counted, but it is still
loss and it is undetectable downstream except as a gap in `Header.Seq`.

If it happens, the fix is upstream — faster codec, lower decimation, or find out
why the far end stopped reading. It is never "raise the queue size until the
number goes away".
