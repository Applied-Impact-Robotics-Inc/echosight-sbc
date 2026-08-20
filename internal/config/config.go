// Package config owns the config value type's defaults, validation and the
// SI5G apply plan. It no longer owns persistence: configurations live on the
// compute server and this machine holds one in RAM. FMC-only: there are no
// scan kinds, no gates, no DAC, and no backwards compatibility with the
// phased-array era.
package config

import (
	"encoding/json"
	"fmt"

	"echosight/internal/wire"
	"echosight/spike"
)

// Default is THE working FMC configuration, not a neutral starting point.
// A fresh config directory must boot straight into a streaming, imaging
// system with zero operator config actions — that requirement is what killed
// the old PA default, because reaching a working state from it meant an
// operator hitting Apply against a board that was already streaming, which is
// exactly the sequence that wedges the readout engine (invariants #1/#2).
//
// The geometry below is not adjustable folklore. aperture 2 / step 2 over a
// 64-element probe gives 32 tx positions at 1.2 mm spacing, 32 parallel rx
// channels, 800 points at the FMC-fixed 50 MHz. Every calibration in the
// profile estimator — bank mapping fingerprint, defect gate, comb sizing,
// side evidence — was fitted against exactly this geometry, and the coherent
// block sizing in imaging.go is documented against a 32 x 450 us = 14.4 ms
// cycle ("50 mm/s -> K=2", "100 mm/s -> 45 um per firing"). Changing tx count
// or sync period invalidates those constants together.
//
// Implied working point: 51,264 B per frame, 1.64 MB per cycle, ~114 MB/s
// produced, ~69.4 cycles/s — comfortably under the ~186 MB/s SBC USB ceiling.
func Default() wire.Config {
	var c wire.Config
	c.V = wire.ConfigVersion

	c.Device.HighVoltage = 40 // invariant #4: HV 40 with gains zeroed
	c.Device.HVEnable = false
	c.Device.Preamp = 1 // G2, roughly 4 dB
	c.Device.AnalogFilters.HP = 1
	c.Device.AnalogFilters.LP = 1
	c.Device.DeviceName = "SPIKE-01"

	c.IO.IO5V = false

	c.Pulser.LengthNs = 100
	c.Pulser.BankIndex = 1

	c.Receiver.DigitalHpf = 1
	c.Receiver.BandpassKHz = [2]float64{2000, 8000}

	c.Digitization.Points = 800
	c.Digitization.DelayOffsetUs = 11

	c.Timing.SyncPeriodUs = 450
	c.Timing.SyncSource = 0
	c.Timing.InternalTimerUs = 0

	c.Scan = wire.ScanConfig{
		ProbeElements: 64,
		PitchMm:       0.6,
		ProbeFreqMHz:  5,
		Aperture:      2,
		Step:          2,
		RxCount:       32,
	}

	c.Acquisition.PreviewHz = 0 // derive from the byte budget

	c.Encoders.Units = []int{spike.UnitMm, spike.UnitMm, spike.UnitNone, spike.UnitNone}
	c.Encoders.Resolutions = []float64{1, 1, 1, 1}
	c.Encoders.Invert = []bool{false, false, false, false}

	c.Processing.VelocityMps = 5900
	c.Processing.NominalMm = 6.25
	c.Processing.ToleranceMm = 1.5
	// Seed only. The interface fitter measures the real water path every
	// block and the app shows the fitted value; this is what the travel-time
	// tables get built against before the first fit lands.
	c.Processing.StandoffMm = 8.8

	return c
}

// Validate checks the config against the manual's documented ranges. It never
// touches the device, so the frontend can call it on every keystroke.
func Validate(c wire.Config) wire.ValidateResult {
	var p []wire.Problem
	add := func(path, msg string) { p = append(p, wire.Problem{Path: path, Msg: msg}) }

	// 2.8.2.4 high voltage 10..50 V
	if c.Device.HighVoltage < 10 || c.Device.HighVoltage > 50 {
		add("device.highVoltage", "must be 10 to 50 V")
	}
	if c.Device.Preamp < 0 || c.Device.Preamp > 2 {
		add("device.preamp", "must be 0 (G1), 1 (G2) or 2 (G3)")
	}
	if c.Device.AnalogFilters.HP < 0 || c.Device.AnalogFilters.HP > 1 {
		add("device.analogFilters.hp", "must be 0 (300 kHz) or 1 (600 kHz)")
	}
	if c.Device.AnalogFilters.LP < 0 || c.Device.AnalogFilters.LP > 2 {
		add("device.analogFilters.lp", "must be 0 (10 MHz), 1 (15 MHz) or 2 (25 MHz)")
	}
	// 2.8.2.3
	if c.Pulser.LengthNs < 25 || c.Pulser.LengthNs > 500 {
		add("pulser.lengthNs", "must be 25 to 500 ns")
	}
	if c.Pulser.BankIndex < 1 || c.Pulser.BankIndex > 8 {
		add("pulser.bankIndex", "must be 1 to 8")
	}
	// 2.8.3.9 band pass 0..25 MHz
	if c.Receiver.BandpassKHz[0] < 0 || c.Receiver.BandpassKHz[1] > 25000 {
		add("receiver.bandpassKHz", "must be within 0 to 25000 kHz")
	}
	if c.Receiver.BandpassKHz[0] >= c.Receiver.BandpassKHz[1] {
		add("receiver.bandpassKHz", "low cutoff must be below high cutoff")
	}
	if c.Receiver.DigitalHpf < 0 || c.Receiver.DigitalHpf > 3 {
		add("receiver.digitalHpf", "must be 0 (none), 1 (3480), 2 (1860) or 3 (900 kHz)")
	}
	// 2.8.6.3
	if c.Digitization.Points < 256 || c.Digitization.Points > 32768 {
		add("digitization.points", "must be 256 to 32768")
	}
	// 2.8.6.2 delay 0..2 ms
	if c.Digitization.DelayOffsetUs < 0 || c.Digitization.DelayOffsetUs > 2000 {
		add("digitization.delayOffsetUs", "must be 0 to 2000 us")
	}
	// 2.8.1.1 sync period 50us..10ms
	if c.Timing.SyncPeriodUs < 50 || c.Timing.SyncPeriodUs > 10000 {
		add("timing.syncPeriodUs", "must be 50 us to 10 ms (100 Hz to 20 kHz)")
	}
	// 2.8.1.4 sources 0..8
	if c.Timing.SyncSource < 0 || c.Timing.SyncSource > 8 {
		add("timing.syncSource", "must be 0 to 8")
	}
	// RxCount above the board's receiver count is only legal in FMC_TDM, where
	// the receivers are multiplexed across the aperture. Validate cannot see
	// the firmware mode, so it checks the geometry is expressible and leaves
	// the mode check to arm time, which can read dev#1.mode.
	if c.Scan.RxCount > wire.BoardReceivers && c.Scan.RxCount != c.Scan.ProbeElements {
		add("scan.rxCount", fmt.Sprintf(
			"must be <= %d (one firing) or exactly %d (FMC_TDM, muxed across the full aperture)",
			wire.BoardReceivers, c.Scan.ProbeElements))
	}
	if c.Processing.VelocityMps <= 0 {
		add("processing.velocityMps", "must be positive")
	}
	if c.Processing.NominalMm <= 0 {
		add("processing.nominalMm", "must be positive")
	}

	// Bus-integrity guard, not a manual range. The board produces
	// frameBytes/syncPeriod continuously regardless of what the host pulls;
	// once production exceeds the link, the board-side FIFO discards stream
	// segments and cycle framing slips through the holes with no error
	// anywhere (field: 1200 points predicted 220 MB/s against a ~186 MB/s
	// ceiling and the display reshuffled endlessly). Refuse to configure it
	// rather than let the read loop discover it as corruption.
	if mb := ProducedMBs(c); mb > usbCeilingMBs {
		add("timing.syncPeriodUs", fmt.Sprintf(
			"%.0f MB/s produced exceeds the ~%.0f MB/s link ceiling; raise sync period or lower points",
			mb, usbCeilingMBs))
	}

	if _, err := BuildScript(c); err != nil {
		add("scan", err.Error())
	}

	return wire.ValidateResult{Valid: len(p) == 0, Problems: p}
}

// usbCeilingMBs is the measured sustained ceiling on the SBC's USB3 link, well
// below Socomate's 316 MB/s benchmark figure. Bench-tunable; keep it
// pessimistic, because exceeding it fails silently.
const usbCeilingMBs float64 = 186

// FrameBytes is the wire size of one FMC frame: a 32-word header followed by
// rxCount*points interleaved int16 samples.
func FrameBytes(c wire.Config) int {
	rx := c.Scan.RxCount
	if rx <= 0 {
		rx = 32
	}
	return 2*rx*c.Digitization.Points + 64
}

// ProducedMBs is the rate the board pushes onto the link, independent of the
// host. One frame per transmit position, and a transmit position takes
// FiringsPerTx sync periods (2 in FMC_TDM, where the receivers are muxed
// across the full aperture).
func ProducedMBs(c wire.Config) float64 {
	if c.Timing.SyncPeriodUs <= 0 {
		return 0
	}
	period := c.Timing.SyncPeriodUs * float64(c.Scan.FiringsPerTx()) * 1e-6
	return float64(FrameBytes(c)) / period / (1 << 20)
}

// CycleRateHz is the FMC frame-set rate: one full tx sweep per cycle.
func CycleRateHz(c wire.Config) float64 {
	n := c.Scan.TxPositions()
	if n <= 0 || c.Timing.SyncPeriodUs <= 0 {
		return 0
	}
	return 1 / (float64(n) * float64(c.Scan.FiringsPerTx()) * c.Timing.SyncPeriodUs * 1e-6)
}

// BuildScript turns the ScanConfig into an SI5G firing scenario. There is one
// scan type left, so there is no dispatch.
func BuildScript(c wire.Config) (*spike.Script, error) {
	return spike.BuildFMCScan(spike.FMCScanConfig{
		ProbeElements: c.Scan.ProbeElements,
		Aperture:      c.Scan.Aperture,
		Step:          c.Scan.Step,
		RxCount:       c.Scan.RxCount,
	})
}

// SequenceMap derives the firing-to-spatial mapping and human labels. FMC
// publishes one pseudo-record per receive channel (acquire_fmc.go), so
// spatialIndex = seqIdx*rxCount + chIdx and the map is expanded to match.
func SequenceMap(c wire.Config) []wire.SequenceMapEntry {
	sc, err := BuildScript(c)
	if err != nil || len(sc.Groups) == 0 {
		return nil
	}
	var out []wire.SequenceMapEntry
	for si, s := range sc.Groups[0].Sequences {
		txFirst := 0
		if len(s.Emission.Elements) > 0 {
			txFirst = s.Emission.Elements[0]
		}
		for ci, rxEl := range s.Reception.Elements {
			idx := si*len(s.Reception.Elements) + ci
			out = append(out, wire.SequenceMapEntry{
				FiringIndex:   idx,
				SpatialIndex:  idx,
				CenterElement: rxEl,
				Label:         fmt.Sprintf("t%dc%d", txFirst, rxEl),
			})
		}
	}
	return out
}

// ScriptJSON returns the serialized scenario for GET /api/config/script.
func ScriptJSON(c wire.Config) (string, error) {
	sc, err := BuildScript(c)
	if err != nil {
		return "", err
	}
	return sc.JSON()
}

// ValidateScriptJSON does a structural check without touching the device. A
// full check requires writing dev#1.settings, which the caller does separately.
func ValidateScriptJSON(s string) wire.ValidateResult {
	var probe struct {
		Groups []spike.Group `json:"Groups"`
		// single-group flat form
		Group     int              `json:"Group"`
		Sequences []spike.Sequence `json:"Sequences"`
	}
	if err := json.Unmarshal([]byte(s), &probe); err != nil {
		return wire.ValidateResult{Problems: []wire.Problem{{Path: "json", Msg: err.Error()}}}
	}
	groups := probe.Groups
	if len(groups) == 0 && len(probe.Sequences) > 0 {
		groups = []spike.Group{{Group: probe.Group, Sequences: probe.Sequences}}
	}
	if len(groups) == 0 {
		return wire.ValidateResult{Problems: []wire.Problem{{Path: "Groups", Msg: "no groups found"}}}
	}
	if len(groups) > 8 {
		return wire.ValidateResult{Problems: []wire.Problem{{Path: "Groups", Msg: "at most 8 groups are supported"}}}
	}
	var p []wire.Problem
	for gi, g := range groups {
		for si, s := range g.Sequences {
			path := fmt.Sprintf("Groups[%d].Sequences[%d]", gi, si)
			if s.Emission.Aperture != len(s.Emission.Elements) {
				p = append(p, wire.Problem{Path: path + ".Emission", Msg: "Aperture does not match the Elements length"})
			}
			if len(s.Emission.Elements) != len(s.Emission.Delays) {
				p = append(p, wire.Problem{Path: path + ".Emission", Msg: "Elements and Delays differ in length"})
			}
			if len(s.Reception.Elements) != len(s.Reception.Delays) {
				p = append(p, wire.Problem{Path: path + ".Reception", Msg: "Elements and Delays differ in length"})
			}
		}
	}
	return wire.ValidateResult{Valid: len(p) == 0, Problems: p}
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------

// Load, Save and ErrWrongVersion are GONE.
//
// This machine no longer persists a configuration. It pulls one for the active
// session from the compute server on boot and on reconnect, holds it in RAM for
// the life of the process, and writes nothing. A file here was a second place a
// configuration could live, and the two drifted: a stale last-config.json
// holding points 750 and rxCount 30 produced a diagonal crawl that looked like
// a live display for a whole session before anyone distrusted it.
//
// Default() survives, but NOT as something this machine applies on its own. It
// is the seed the compute server uses when creating a new tank, and it is
// reachable here only so the two stay in step.
