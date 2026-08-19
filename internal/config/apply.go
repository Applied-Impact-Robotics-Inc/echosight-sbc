package config

import (
	"fmt"

	"echosight/internal/wire"
	"echosight/spike"
)

type kind int

const (
	kInt kind = iota
	kDbl
	kStr
	kIntArr
	kDblArr
)

// Setter is one parameter write, carrying the config path it came from so a
// clip or an SI5G error can be attributed to the right control in the UI.
// Paths for hardcoded writes use a "fixed." prefix so the UI knows there is no
// control to blame and shows them as diagnostics instead.
type Setter struct {
	Path  string
	Param string
	Unit  int
	Kind  kind

	I       int
	D       float64
	S       string
	AI      []int32
	AD      []float64
	ADim    int // explicit dim when HasADim (may be 0 = clear)
	HasADim bool

	// Locked marks parameters the manual forbids changing while the device is
	// running (2.8.6.1 explicitly, and the -130 error code generally). Apply
	// always stops firing first, so this exists to drive the UI padlocks and to
	// keep the ordering honest if a fast path is ever added.
	Locked bool
}

func p(group int, name string) string { return fmt.Sprintf("dev#1.grp#%d.%s", group, name) }

// SamplingCodeFor returns the frequency_sampling code to write in FMC firmware.
//
// It is ZERO, and it is not derived from anything. FMC firmware digitizes at
// 50 MHz regardless of this parameter — the PA code table (0 = 100 MHz ...
// 7 = 12.5 MHz) does not apply — so the value is inert as a rate selector and
// the only thing that matters is writing what the board was characterised
// with. Every known-good capture header on this rig carries samplingCode 0.
//
// A previous revision of this function returned 1 for FMC and 2 for FMC_TDM,
// reasoning from a code comment rather than from a capture. Writing 1 changed
// the emitted frame length out from under the host: the board kept producing
// 800-point frames while the reader parsed 750-point ones, the cycle
// accumulator slipped 1600 words per frame, and the interface arrival marched
// +50 samples per tx position through the matrix — the exact "A-scans drift in
// time and tx rows rotate" signature documented in acquire_fmc.go. The TFM
// image went to hyperbolic mush because every tx leg was delayed against a
// different time origin.
//
// If this ever needs to change, change it against a capture, not a comment.
func SamplingCodeFor(fwMode int) int {
	return 0
}

// Plan builds the ordered write list for a config. Device-wide settings first,
// then group settings. The firing script and the acquisition enables are
// handled separately by the caller because they bracket this list.
//
// Everything the phased-array UI used to expose and this system now fixes is
// still WRITTEN, deliberately, rather than left at whatever the board happened
// to hold: gains zeroed, video mode HF, DAC disabled, pulser negative. A board
// that a previous firmware or a previous revision left in some other state
// self-heals on the next apply instead of quietly producing wrong images.
func Plan(c wire.Config, group, fwMode int) []Setter {
	b2i := func(b bool) int {
		if b {
			return 1
		}
		return 0
	}
	var s []Setter

	// --- device wide ---
	s = append(s,
		Setter{Path: "device.deviceName", Param: "dev#1.device_name", Unit: spike.UnitNone, Kind: kStr, S: c.Device.DeviceName},
		Setter{Path: "device.preamp", Param: "dev#1.receiver_preampli", Unit: spike.UnitNone, Kind: kInt, I: c.Device.Preamp, Locked: true},
		Setter{Path: "device.analogFilters", Param: "dev#1.receiver_analog_filters", Unit: spike.UnitNone, Kind: kIntArr,
			AI: []int32{int32(c.Device.AnalogFilters.HP), int32(c.Device.AnalogFilters.LP)}, Locked: true},
		Setter{Path: "receiver.digitalHpf", Param: "dev#1.receiver_digital_high_pass_filter", Unit: spike.UnitNone, Kind: kInt, I: c.Receiver.DigitalHpf},
		Setter{Path: "io.io5v", Param: "dev#1.IO_5V", Unit: spike.UnitNone, Kind: kInt, I: b2i(c.IO.IO5V)},
		Setter{Path: "io.output3", Param: "dev#1.output_3", Unit: spike.UnitNone, Kind: kInt, I: b2i(c.IO.Output3)},
		Setter{Path: "io.output4", Param: "dev#1.output_4", Unit: spike.UnitNone, Kind: kInt, I: b2i(c.IO.Output4)},
		Setter{Path: "timing.syncSource", Param: "dev#1.synchronization_source", Unit: spike.UnitNone, Kind: kInt, I: c.Timing.SyncSource},
		// A-scan acquisition, always. C-scan was a PA-era mode.
		Setter{Path: "fixed.acquisitionType", Param: "dev#1.type_of_acquisition", Unit: spike.UnitNone, Kind: kInt, I: 1, Locked: true},
	)
	// 2.8.1.6: internal_timer only matters when synchronization_source is 7.
	if c.Timing.SyncSource == 7 {
		s = append(s, Setter{Path: "timing.internalTimerUs", Param: "dev#1.internal_timer", Unit: spike.UnitUs, Kind: kDbl, D: c.Timing.InternalTimerUs})
	}
	// High voltage is written before the enable so the board never sits armed
	// at a stale voltage for even one call.
	s = append(s,
		Setter{Path: "device.highVoltage", Param: "dev#1.high_voltage", Unit: spike.UnitNone, Kind: kInt, I: c.Device.HighVoltage, Locked: true},
		Setter{Path: "device.hvEnable", Param: "dev#1.enable_high_voltage", Unit: spike.UnitNone, Kind: kInt, I: b2i(c.Device.HVEnable)},
	)

	// --- encoders ---
	// The encoder COUNT is load-bearing for the wire format: each
	// programmed encoder pair changes the frame row width by 2 words
	// (measured: 4 programmed -> 30-word rows, 2 -> 32, 0 -> 34; the
	// parser and every capture assume 32). Two wire quirks handled here:
	// the array writes must be dim 4 (a dim-2 write returns invalid
	// argument on the 5GPA library build), and UnitNone (-1) in a slot
	// PROGRAMS that encoder rather than disabling it — zeros are what
	// leave slots 3/4 unprogrammed. The config carries the semantic
	// truth (two encoders, robot X/Y in mm); the padding is wire detail.
	if n := len(c.Encoders.Units); n > 0 {
		units := make([]int32, 4)
		for i, u := range c.Encoders.Units {
			if i < 4 {
				units[i] = int32(u)
			}
		}
		s = append(s, Setter{Path: "encoders.units", Param: "dev#1.encoder_units", Unit: spike.UnitNone, Kind: kIntArr, AI: units})
	}
	if len(c.Encoders.Resolutions) > 0 {
		res := make([]float64, 4)
		copy(res, c.Encoders.Resolutions)
		s = append(s, Setter{Path: "encoders.resolutions", Param: "dev#1.encoder_resolutions", Unit: spike.UnitNone, Kind: kDblArr, AD: res})
	}
	if len(c.Encoders.Invert) > 0 {
		var mask int
		for i, inv := range c.Encoders.Invert {
			if inv {
				mask |= 1 << i
			}
		}
		// The directions write has never succeeded on this library build
		// (invalid argument every session) and an all-false mask is the
		// power-on default anyway — only spend the write, and the log
		// noise, when an inversion is actually requested.
		if mask != 0 {
			s = append(s, Setter{Path: "encoders.invert", Param: "dev#1.encoder_directions", Unit: spike.UnitNone, Kind: kInt, I: mask})
		}
	}

	// --- group ---
	s = append(s,
		Setter{Path: "fixed.samplingCode", Param: p(group, "frequency_sampling"), Unit: spike.UnitNone, Kind: kInt, I: SamplingCodeFor(fwMode), Locked: true},
		Setter{Path: "digitization.points", Param: p(group, "size_of_digitalization"), Unit: spike.UnitNone, Kind: kInt, I: c.Digitization.Points, Locked: true},
		Setter{Path: "digitization.delayOffsetUs", Param: p(group, "delay_offset"), Unit: spike.UnitUs, Kind: kDbl, D: c.Digitization.DelayOffsetUs},

		// Negative excitation, always. Bipolar was never characterised
		// against this probe/plate pair and the estimator's train-phase
		// reasoning assumes the negative-going first arrival.
		Setter{Path: "fixed.pulserType", Param: p(group, "pulser_type"), Unit: spike.UnitNone, Kind: kInt, I: 1, Locked: true},
		Setter{Path: "pulser.lengthNs", Param: p(group, "pulser_length"), Unit: spike.UnitNs, Kind: kDbl, D: c.Pulser.LengthNs, Locked: true},
		Setter{Path: "pulser.bankIndex", Param: p(group, "pulser_index"), Unit: spike.UnitNone, Kind: kInt, I: c.Pulser.BankIndex, Locked: true},

		// INVARIANT #4: gains stay at 0/0 and the interface clips at +-32k
		// BY DESIGN at HV 40 V. The skirt buries features for ~1.1 us after
		// the interface and every downstream estimator accounts for it. Do
		// not add a gain control to "fix" the clipping — that reintroduces a
		// bug class that cost multiple bench days, because the fix moves the
		// clipping without moving the skirt.
		Setter{Path: "fixed.analogGainDb", Param: p(group, "receiver_analog_gain"), Unit: spike.UnitNone, Kind: kDbl, D: 0},
		Setter{Path: "fixed.digitalGainDb", Param: p(group, "receiver_digital_gain"), Unit: spike.UnitNone, Kind: kDbl, D: 0},

		Setter{Path: "receiver.bandpassKHz", Param: p(group, "receiver_digital_band_pass_filter"), Unit: spike.UnitKHz, Kind: kDblArr,
			AD: []float64{c.Receiver.BandpassKHz[0], c.Receiver.BandpassKHz[1]}},

		// HF (raw RF), always. TFM delay-and-sum needs signed RF samples;
		// envelope or rectified video modes destroy the phase the coherent
		// sum depends on and produce a plausible-looking, wrong image.
		Setter{Path: "fixed.videoMode", Param: p(group, "scope_video_mode"), Unit: spike.UnitNone, Kind: kInt, I: 0, Locked: true},

		Setter{Path: "timing.syncPeriodUs", Param: p(group, "sync_period"), Unit: spike.UnitUs, Kind: kDbl, D: c.Timing.SyncPeriodUs},

		// The board's DAC stays OFF. There is no host-side TCG either any
		// more; disabling it on every apply self-heals a board that some
		// earlier revision programmed a curve into, which would otherwise
		// apply an invisible depth-dependent gain under the TFM sum.
		Setter{Path: "fixed.dacEnabled", Param: p(group, "receiver_dac_enable"), Unit: spike.UnitNone, Kind: kInt, I: 0},
	)

	// --- bench / simulation (2.8.12.3) ---
	s = append(s,
		Setter{Path: "testing.patternEnabled", Param: "dev#1.enable_pattern", Unit: spike.UnitNone, Kind: kInt, I: b2i(c.Testing.PatternEnabled)},
		Setter{Path: "testing.encoderSimEnabled", Param: "dev#1.enable_encoders_simulation", Unit: spike.UnitNone, Kind: kInt, I: b2i(c.Testing.EncoderSimEnabled)},
	)

	return s
}

// Exec runs a plan against the device, collecting applied paths, clip reports
// and errors instead of aborting on the first failure. A single bad field
// should not silently abandon the twenty writes queued behind it.
func Exec(plan []Setter) (wire.ApplyResult, []wire.Si5gError) {
	res := wire.ApplyResult{Applied: []string{}, Clipped: []wire.Clip{}, Errors: []string{}}
	var si []wire.Si5gError

	record := func(op string, st Setter, err error) bool {
		if err == nil {
			return false
		}
		res.Errors = append(res.Errors, fmt.Sprintf("%s: %v", st.Path, err))
		if e, ok := err.(*spike.Error); ok {
			si = append(si, wire.Si5gError{Op: op, Param: st.Param, Code: e.Code, Name: spike.ErrorName(e.Code)})
		}
		return true
	}

	for _, st := range plan {
		switch st.Kind {
		case kInt:
			got, clipped, err := spike.SetInt(st.Param, st.Unit, st.I)
			if record("SetInt", st, err) {
				continue
			}
			if clipped {
				res.Clipped = append(res.Clipped, wire.Clip{Path: st.Path, Requested: float64(st.I), Applied: float64(got)})
			}
		case kDbl:
			got, clipped, err := spike.SetDbl(st.Param, st.Unit, st.D)
			if record("SetDbl", st, err) {
				continue
			}
			if clipped {
				res.Clipped = append(res.Clipped, wire.Clip{Path: st.Path, Requested: st.D, Applied: got})
			}
		case kStr:
			if record("SetStr", st, spike.SetStr(st.Param, st.Unit, st.S)) {
				continue
			}
		case kIntArr:
			if len(st.AI) == 0 {
				continue
			}
			if record("Set1DInt", st, spike.Set1DInt(st.Param, st.Unit, st.AI)) {
				continue
			}
		case kDblArr:
			if len(st.AD) == 0 {
				continue
			}
			if record("Set1DDbl", st, spike.Set1DDbl(st.Param, st.Unit, st.AD)) {
				continue
			}
		}
		res.Applied = append(res.Applied, st.Path)
	}
	return res, si
}
