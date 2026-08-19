package device

import (
	"encoding/binary"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"

	"echosight/internal/config"
	"echosight/spike"
)

// FMC capture: grab N consecutive firing cycles of RAW channel data from the
// live stream, write them to disk (JSON meta line + little-endian int16), and
// produce a compact text digest designed to be pasted into a chat/issue for
// analysis — per-record stats, header timers, cross-cycle stability, and a
// handful of envelope-decimated A-scans, ~15 KB total. Capturing copies
// memory only; it never touches the device, so it is safe mid-collection.

// fmcGeometry is published by the read loop after calibration so captures
// know how to slice what they copied.
type fmcGeometry struct {
	NumShots   int `json:"numShots"`
	Channels   int `json:"channels"`
	Points     int `json:"points"`
	WireFrame  int `json:"wireFrame"`
	FrameLead  int `json:"frameLead"`
	CycleWords int `json:"cycleWords"`
}

type fmcCaptureState struct {
	want int
	got  int
	geom fmcGeometry
	cfg  fmcCfgSummary
	data []uint16
}

type fmcCfgSummary struct {
	SyncPeriodUs  float64 `json:"syncPeriodUs"`
	// Gains are fixed at 0/0 and the sampling rate is fixed at 50 MHz in FMC
	// firmware; both are kept in the header because .fmc files are
	// self-describing and offline tools read these keys. fsMHz is explicit
	// so nothing has to know the FMC sampling-code table to read a capture.
	AnalogGainDb  float64 `json:"analogGainDb"`
	DigitalGainDb float64 `json:"digitalGainDb"`
	HighVoltage   int     `json:"highVoltage"`
	SamplingCode  int     `json:"samplingCode"`
	FsMHz         float64 `json:"fsMHz"`
	DelayOffsetUs float64 `json:"delayOffsetUs"`
	// Scan geometry, so offline tools use the EXACT tx layout instead of
	// inferring it from the shot count (which breaks on configs like
	// aperture 2 / step 1).
	Aperture      int     `json:"aperture"`
	Step          int     `json:"step"`
	ProbeElements int     `json:"probeElements"`
	PitchMm       float64 `json:"pitchMm"`
	RxCount       int     `json:"rxCount"`
}

// FMCCaptureInfo is the API-facing status of the last capture.
type FMCCaptureInfo struct {
	Done       bool        `json:"done"`
	Error      string      `json:"error,omitempty"`
	Cycles     int         `json:"cycles"`
	File       string      `json:"file,omitempty"`
	DigestFile string      `json:"digestFile,omitempty"`
	Bytes      int         `json:"bytes"`
	Geometry   fmcGeometry `json:"geometry"`
	StartedAt  time.Time   `json:"startedAt"`
}

type fmcCaptureResult struct {
	mu     sync.Mutex
	info   FMCCaptureInfo
	digest string
}

// StartFMCCapture arms a capture of n cycles from the live FMC stream.
func (s *Supervisor) StartFMCCapture(n int) error {
	if n < 1 {
		n = 1
	}
	if n > 64 {
		n = 64
	}
	if s.acq == nil {
		return fmt.Errorf("no acquisition running; start FMC collection first")
	}
	if s.fwMode != spike.ModeFMC && s.fwMode != spike.ModeFMCTDM {
		return fmt.Errorf("capture needs FMC firmware")
	}
	gp := s.fmcGeom.Load()
	if gp == nil {
		return fmt.Errorf("stream not calibrated yet; try again in a second")
	}
	s.mu.RLock()
	cfg := s.applied
	s.mu.RUnlock()
	st := &fmcCaptureState{
		want: n,
		geom: *gp,
		cfg: fmcCfgSummary{
			SyncPeriodUs:  cfg.Timing.SyncPeriodUs,
			AnalogGainDb:  0,
			DigitalGainDb: 0,
			HighVoltage:   cfg.Device.HighVoltage,
			SamplingCode:  config.SamplingCodeFor(s.fwMode),
			FsMHz:         50,
			DelayOffsetUs: cfg.Digitization.DelayOffsetUs,
			Aperture:      cfg.Scan.Aperture,
			Step:          cfg.Scan.Step,
			ProbeElements: cfg.Scan.ProbeElements,
			PitchMm:       cfg.Scan.PitchMm,
			RxCount:       cfg.Scan.RxCount,
		},
		data: make([]uint16, 0, n*gp.CycleWords),
	}
	if !s.fmcCap.CompareAndSwap(nil, st) {
		return fmt.Errorf("a capture is already in progress")
	}
	s.fmcLast.mu.Lock()
	s.fmcLast.info = FMCCaptureInfo{Cycles: n, Geometry: *gp, StartedAt: time.Now()}
	s.fmcLast.digest = ""
	s.fmcLast.mu.Unlock()
	s.logf("info", "fmc capture armed: %d cycles (%d MB)", n, n*gp.CycleWords*2>>20)
	return nil
}

// captureCycle is called by the read loop for every extracted cycle.
func (s *Supervisor) captureCycle(cycle []uint16) {
	st := s.fmcCap.Load()
	if st == nil {
		return
	}
	st.data = append(st.data, cycle...)
	st.got++
	if st.got >= st.want {
		s.fmcCap.Store(nil)
		go s.finishFMCCapture(st)
	}
}

// LastFMCCapture reports the status of the most recent capture.
func (s *Supervisor) LastFMCCapture() FMCCaptureInfo {
	s.fmcLast.mu.Lock()
	defer s.fmcLast.mu.Unlock()
	return s.fmcLast.info
}

// FMCDigest returns the paste-friendly analysis text of the last capture.
func (s *Supervisor) FMCDigest() (string, error) {
	s.fmcLast.mu.Lock()
	defer s.fmcLast.mu.Unlock()
	if !s.fmcLast.info.Done {
		return "", fmt.Errorf("no completed capture")
	}
	if s.fmcLast.info.Error != "" {
		return "", fmt.Errorf("%s", s.fmcLast.info.Error)
	}
	return s.fmcLast.digest, nil
}

// FMCFilePath returns the binary file path of the last capture.
func (s *Supervisor) FMCFilePath() (string, error) {
	s.fmcLast.mu.Lock()
	defer s.fmcLast.mu.Unlock()
	if !s.fmcLast.info.Done || s.fmcLast.info.File == "" {
		return "", fmt.Errorf("no completed capture")
	}
	return s.fmcLast.info.File, nil
}

func (s *Supervisor) finishFMCCapture(st *fmcCaptureState) {
	dir := s.opt.CaptureDir
	if dir == "" {
		dir = "captures"
	}
	_ = os.MkdirAll(dir, 0o755)
	ts := time.Now().Format("20060102-150405")
	binPath := filepath.Join(dir, "fmc-"+ts+".fmc")
	digPath := filepath.Join(dir, "fmc-"+ts+".txt")

	fail := func(err error) {
		s.fmcLast.mu.Lock()
		s.fmcLast.info.Done = true
		s.fmcLast.info.Error = err.Error()
		s.fmcLast.mu.Unlock()
		s.logf("warn", "fmc capture failed: %v", err)
	}

	// Binary: one JSON meta line, then raw LE int16.
	meta := struct {
		Kind     string        `json:"kind"`
		Geometry fmcGeometry   `json:"geometry"`
		Config   fmcCfgSummary `json:"config"`
		Cycles   int           `json:"cycles"`
		SavedAt  time.Time     `json:"savedAt"`
	}{"fmc-raw", st.geom, st.cfg, st.got, time.Now()}
	f, err := os.Create(binPath)
	if err != nil {
		fail(err)
		return
	}
	enc := json.NewEncoder(f)
	if err := enc.Encode(&meta); err != nil {
		f.Close()
		fail(err)
		return
	}
	raw := make([]byte, len(st.data)*2)
	for i, w := range st.data {
		binary.LittleEndian.PutUint16(raw[i*2:], w)
	}
	if _, err := f.Write(raw); err != nil {
		f.Close()
		fail(err)
		return
	}
	f.Close()

	digest := buildFMCDigest(st)
	if err := os.WriteFile(digPath, []byte(digest), 0o644); err != nil {
		fail(err)
		return
	}

	s.fmcLast.mu.Lock()
	s.fmcLast.info.Done = true
	s.fmcLast.info.File = binPath
	s.fmcLast.info.DigestFile = digPath
	s.fmcLast.info.Bytes = len(raw)
	s.fmcLast.digest = digest
	s.fmcLast.mu.Unlock()
	s.logf("info", "fmc capture complete: %s (%d cycles, %.1f MB), digest %s",
		binPath, st.got, float64(len(raw))/(1<<20), digPath)
}

// ---------------------------------------------------------------------------

func buildFMCDigest(st *fmcCaptureState) string {
	g := st.geom
	var b strings.Builder
	fmt.Fprintf(&b, "ECHOSIGHT FMC CAPTURE DIGEST v1\n")
	fmt.Fprintf(&b, "geometry: %d tx x %d rx, points=%d, wireFrame=%dw, lead=%dw, cycle=%dw\n",
		g.NumShots, g.Channels, g.Points, g.WireFrame, g.FrameLead, g.CycleWords)
	fmt.Fprintf(&b, "config: syncUs=%.3f analogDb=%.1f digitalDb=%.1f hv=%d fsCode=%d delayUs=%.2f\n",
		st.cfg.SyncPeriodUs, st.cfg.AnalogGainDb, st.cfg.DigitalGainDb,
		st.cfg.HighVoltage, st.cfg.SamplingCode, st.cfg.DelayOffsetUs)
	fmt.Fprintf(&b, "cycles=%d\n", st.got)

	cycle := func(ci int) []uint16 { return st.data[ci*g.CycleWords : (ci+1)*g.CycleWords] }
	frame := func(ci, fi int) []uint16 { c := cycle(ci); return c[fi*g.WireFrame : (fi+1)*g.WireFrame] }
	sample := func(fr []uint16, c, smp int) int16 { return int16(fr[g.FrameLead+smp*g.Channels+c]) }

	// Raw head of frame 0: direct wire forensics — the parsed header must
	// show timer words then zeros; anything else means the lock is off.
	b.WriteString("\n-- frame0 words 0..47 (hex)\n")
	f0 := frame(0, 0)
	for w := 0; w < 48 && w < len(f0); w++ {
		fmt.Fprintf(&b, "%04x ", f0[w])
		if w%16 == 15 {
			b.WriteString("\n")
		}
	}
	b.WriteString("\n")

	// Header timers: absolute for cycle 0, then per-frame deltas per cycle.
	if g.FrameLead >= 4 {
		b.WriteString("\n-- frame timers (us): cycle: t0, then deltas f1-f0 .. f7-f6\n")
		for ci := 0; ci < st.got; ci++ {
			var ts []uint64
			for fi := 0; fi < g.NumShots; fi++ {
				fr := frame(ci, fi)
				ts = append(ts, uint64(fr[0])|uint64(fr[1])<<16|uint64(fr[2])<<32|uint64(fr[3])<<48)
			}
			fmt.Fprintf(&b, "c%d: %d", ci, ts[0])
			for i := 1; i < len(ts); i++ {
				fmt.Fprintf(&b, " +%d", ts[i]-ts[i-1])
			}
			b.WriteString("\n")
		}
	}

	// Per-record stats, cycle 0: peak, peak index, rms, clip.
	b.WriteString("\n-- records cycle0: tx,rx,peak,peakIdx,rms,clip\n")
	type rec struct {
		peak, idx int
		rms       float64
	}
	stats := make([][]rec, st.got)
	for ci := 0; ci < st.got; ci++ {
		stats[ci] = make([]rec, g.NumShots*g.Channels)
		for fi := 0; fi < g.NumShots; fi++ {
			fr := frame(ci, fi)
			for c := 0; c < g.Channels; c++ {
				var pk, pi int
				var sq float64
				for smp := 0; smp < g.Points; smp++ {
					v := int(sample(fr, c, smp))
					a := v
					if a < 0 {
						a = -a
					}
					if a > pk {
						pk, pi = a, smp
					}
					sq += float64(v) * float64(v)
				}
				stats[ci][fi*g.Channels+c] = rec{pk, pi, sq}
			}
		}
	}
	for fi := 0; fi < g.NumShots; fi++ {
		for c := 0; c < g.Channels; c++ {
			r := stats[0][fi*g.Channels+c]
			clip := 0
			if r.peak >= 32700 {
				clip = 1
			}
			fmt.Fprintf(&b, "%d,%d,%d,%d,%.0f,%d\n", fi, c, r.peak, r.idx,
				math.Sqrt(r.rms/float64(g.Points)), clip)
		}
	}

	// Cross-cycle stability: with a parked probe, peak index jitter measures
	// framing alignment and peak drift measures signal stability.
	if st.got > 1 {
		b.WriteString("\n-- cross-cycle stability (all cycles): record, peakIdx min..max (jitter), peak min..max (drift%)\n")
		type mov struct {
			fi, c, jit             int
			pmin, pmax, imin, imax int
			drift                  float64
		}
		var movers []mov
		var jitSum, driftSum float64
		n := g.NumShots * g.Channels
		for k := 0; k < n; k++ {
			imin, imax := stats[0][k].idx, stats[0][k].idx
			pmin, pmax := stats[0][k].peak, stats[0][k].peak
			for ci := 1; ci < st.got; ci++ {
				r := stats[ci][k]
				if r.idx < imin {
					imin = r.idx
				}
				if r.idx > imax {
					imax = r.idx
				}
				if r.peak < pmin {
					pmin = r.peak
				}
				if r.peak > pmax {
					pmax = r.peak
				}
			}
			drift := 0.0
			if pmax > 0 {
				drift = 100 * float64(pmax-pmin) / float64(pmax)
			}
			jitSum += float64(imax - imin)
			driftSum += drift
			movers = append(movers, mov{k / g.Channels, k % g.Channels, imax - imin, pmin, pmax, imin, imax, drift})
		}
		fmt.Fprintf(&b, "mean jitter=%.2f samples, mean drift=%.1f%%\n", jitSum/float64(n), driftSum/float64(n))
		sort.Slice(movers, func(i, j int) bool { return movers[i].jit > movers[j].jit })
		b.WriteString("top movers: ")
		for i := 0; i < 6 && i < len(movers); i++ {
			m := movers[i]
			fmt.Fprintf(&b, "t%dc%d idx%d..%d pk%d..%d(%.0f%%); ", m.fi, m.c, m.imin, m.imax, m.pmin, m.pmax, m.drift)
		}
		b.WriteString("\n")
	}

	// Representative A-scans, envelope-decimated to <=160 points.
	b.WriteString("\n-- ascans cycle0 (envelope max-abs decimated, csv of int16)\n")
	decim := 1
	for g.Points/decim > 160 {
		decim++
	}
	fmt.Fprintf(&b, "decim=%d (each value spans %d samples)\n", decim, decim)
	picks := [][2]int{{0, 0}, {0, g.Channels / 2}, {0, g.Channels - 1},
		{g.NumShots / 2, g.Channels / 2}, {g.NumShots - 1, g.Channels - 1}}
	for _, p := range picks {
		fr := frame(0, p[0])
		fmt.Fprintf(&b, "t%dc%d:", p[0], p[1])
		for o := 0; o*decim < g.Points; o++ {
			best := int16(0)
			mag := -1
			for j := 0; j < decim && o*decim+j < g.Points; j++ {
				v := sample(fr, p[1], o*decim+j)
				a := int(v)
				if a < 0 {
					a = -a
				}
				if a > mag {
					mag, best = a, v
				}
			}
			fmt.Fprintf(&b, "%d,", best)
		}
		b.WriteString("\n")
	}
	return b.String()
}
