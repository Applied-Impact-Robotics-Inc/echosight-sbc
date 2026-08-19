//go:build linux

package spike

import (
	"encoding/json"
	"fmt"
	"math"
)

// ============================================================================
// Firing scenario (manual 3.1). The scenario is a JSON string written to
// dev#1.settings. Hierarchy: Groups > Sequences > Emission/Reception, with
// per-element delay laws in microseconds.
// ============================================================================

// Law is one side (emission or reception) of a sequence.
type Law struct {
	Aperture int       `json:"Aperture"`
	Elements []int     `json:"Elements"`
	Delays   []float64 `json:"Delays"` // µsec
}

// Sequence is a single shot within a group.
type Sequence struct {
	Sequence  int `json:"Sequence"` // 1-based
	Emission  Law `json:"Emission"`
	Reception Law `json:"Reception"`
}

// Group is an area of interest. Up to 8 groups per device.
type Group struct {
	Group     int        `json:"Group"` // 1-based
	Probe     int        `json:"Probe"` // always 1 for now
	Sequences []Sequence `json:"Sequences"`
}

// Script is the top-level firing scenario. A single-group script may be
// serialized without the Groups wrapper (both forms are accepted by the
// library, per the manual samples).
type Script struct {
	Groups []Group `json:"Groups"`
}

// JSON serializes the script. Single-group scripts are emitted in the
// flat form used by the SDK samples.
func (s *Script) JSON() (string, error) {
	if len(s.Groups) == 1 {
		b, err := json.Marshal(s.Groups[0])
		return string(b), err
	}
	b, err := json.Marshal(s)
	return string(b), err
}

// FMCScanConfig describes an FMC-firmware sweep: a flat (undelayed) transmit
// aperture of Aperture elements stepped by Step across the probe, with raw
// parallel reception on RxCount channels per shot. Unlike PA mode, each
// FMC-mode sequence yields per-channel data (the frame is a [sample][channel]
// matrix), so one sequence per tx position replaces the 64 single-element
// receive sequences the PA-mode VSA needed. Requires dev#1.mode 1 (FMC,
// 32 channels on a 32:64 board) or 2 (FMC_TDM, muxed to 64).
type FMCScanConfig struct {
	ProbeElements int // total elements on the probe (e.g. 64)
	Aperture      int // tx elements per shot (X)
	Step          int // tx advance per shot (Y)
	RxCount       int // parallel receive channels per shot (e.g. 32)
}

// FMCWindows reports, for each tx position, the 1-based first tx element and
// the 1-based first rx element of the receive window — the same clamped
// centering BuildFMCScan encodes into the scenario. TFM needs this to map
// wire channel c of shot i to physical element rxFirsts[i]+c.
func FMCWindows(cfg FMCScanConfig) (txStarts, rxFirsts []int) {
	if cfg.Step <= 0 {
		cfg.Step = 1
	}
	if cfg.RxCount <= 0 {
		cfg.RxCount = 32
	}
	if cfg.RxCount > cfg.ProbeElements {
		cfg.RxCount = cfg.ProbeElements
	}
	numScans := (cfg.ProbeElements-cfg.Aperture)/cfg.Step + 1
	for i := 0; i < numScans; i++ {
		txFirst := i*cfg.Step + 1
		txCenter := float64(txFirst) + float64(cfg.Aperture-1)/2
		rxFirst := int(math.Round(txCenter - float64(cfg.RxCount-1)/2))
		if rxFirst < 1 {
			rxFirst = 1
		}
		if rxFirst+cfg.RxCount-1 > cfg.ProbeElements {
			rxFirst = cfg.ProbeElements - cfg.RxCount + 1
		}
		txStarts = append(txStarts, txFirst)
		rxFirsts = append(rxFirsts, rxFirst)
	}
	return
}

// BuildFMCScan generates the FMC sequence list. The receive window is
// RxCount elements centered on the transmit aperture, clamped to the probe —
// the Socomate FMC sample's fixed rx 1..32 with tx element 16 is the
// special case of this for a centered single-element shot.
func BuildFMCScan(cfg FMCScanConfig) (*Script, error) {
	if cfg.ProbeElements <= 0 || cfg.Aperture <= 0 {
		return nil, fmt.Errorf("spike: fmc probe elements and aperture must be > 0")
	}
	if cfg.Aperture > cfg.ProbeElements {
		return nil, fmt.Errorf("spike: fmc aperture %d exceeds probe elements %d", cfg.Aperture, cfg.ProbeElements)
	}
	if cfg.Step <= 0 {
		cfg.Step = 1
	}
	if cfg.RxCount <= 0 {
		cfg.RxCount = 32
	}
	if cfg.RxCount > cfg.ProbeElements {
		cfg.RxCount = cfg.ProbeElements
	}

	numScans := (cfg.ProbeElements-cfg.Aperture)/cfg.Step + 1
	txZeros := make([]float64, cfg.Aperture)
	rxZeros := make([]float64, cfg.RxCount)

	g := Group{Group: 1, Probe: 1}
	for i := 0; i < numScans; i++ {
		txFirst := i*cfg.Step + 1
		txElems := make([]int, cfg.Aperture)
		for j := range txElems {
			txElems[j] = txFirst + j
		}

		// Receive window centered on the tx aperture, clamped to the probe.
		txCenter := float64(txFirst) + float64(cfg.Aperture-1)/2
		rxFirst := int(math.Round(txCenter - float64(cfg.RxCount-1)/2))
		if rxFirst < 1 {
			rxFirst = 1
		}
		if rxFirst+cfg.RxCount-1 > cfg.ProbeElements {
			rxFirst = cfg.ProbeElements - cfg.RxCount + 1
		}
		rxElems := make([]int, cfg.RxCount)
		for j := range rxElems {
			rxElems[j] = rxFirst + j
		}

		g.Sequences = append(g.Sequences, Sequence{
			Sequence:  i + 1,
			Emission:  Law{Aperture: cfg.Aperture, Elements: txElems, Delays: txZeros},
			Reception: Law{Aperture: cfg.RxCount, Elements: rxElems, Delays: rxZeros},
		})
	}
	return &Script{Groups: []Group{g}}, nil
}

