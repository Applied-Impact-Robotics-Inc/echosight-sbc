package device

import (
	"context"
	"fmt"
	"time"

	"echosight/internal/config"
	"echosight/internal/state"
	"echosight/spike"
)

// Acquisition lifecycle. FMC firmware is the only mode this server runs, so
// there is no dispatch here any more: startAcqLocked IS the FMC path. The PA
// polling loop, the on-demand A-scan flow, frame slicing, footer gates and the
// host-side DAC LUT all left with the phased-array era.

type acquisition struct {
	cancel context.CancelFunc
	done   chan struct{}
	mode   string
	cycles int
}

// firmwareOK reports whether the board booted into an FMC firmware mode.
//
// INVARIANT #1: dev#1.mode is NEVER switched at runtime. Every wedged FMC
// session in this project's history followed a runtime mode switch, after which
// the readout engine returns nbBytes=0 forever while reporting success. The
// mode comes from config-devices.ini at boot and the only remedy for a wrong
// one is to fix that file and power-cycle.
func (s *Supervisor) firmwareOK() bool {
	return s.fwMode == spike.ModeFMC || s.fwMode == spike.ModeFMCTDM
}

// startAcqLocked arms the board and starts the FMC read loop, then starts the
// TFM worker. There is no separate "start TFM" control anywhere in the system:
// imaging is what this machine does, and the worker's decouple state machine
// already handles the probe being out of water, so always-on costs nothing.
func (s *Supervisor) startAcqLocked(mode string, cycles int) error {
	if s.acq != nil {
		return nil
	}
	if !s.opt.Store.Snapshot().State.DeviceOpen {
		return fmt.Errorf("device is not open")
	}
	s.mu.RLock()
	have := s.hasApplied
	s.mu.RUnlock()
	if !have {
		// Firing an unconfigured board produces data against a geometry
		// nobody chose, and nothing downstream can tell that from real data.
		return fmt.Errorf("no configuration: choose a tank on the compute server, " +
			"then this machine will pull its configuration and arm")
	}
	if !s.firmwareOK() {
		return fmt.Errorf(
			"board booted in %s firmware; this server is FMC-only. Set Mode=FMC in config-devices.ini and power-cycle the board",
			s.opt.Store.Snapshot().Device.FirmwareMode)
	}
	if s.needsReopen {
		return fmt.Errorf("read loop leaked on a previous stop; reopen the device before starting acquisition")
	}

	s.mu.RLock()
	cfg := s.applied
	s.mu.RUnlock()

	// Shots: prefer the device's own count; fall back to the built script.
	numShots := 0
	if n, err := spike.GetInt(fmt.Sprintf("dev#1.grp#%d.number_of_shots", s.opt.Group), spike.UnitNone); err == nil && n > 0 {
		numShots = n
	} else if sc, err := config.BuildScript(cfg); err == nil && len(sc.Groups) > 0 {
		numShots = len(sc.Groups[0].Sequences)
	}
	if numShots <= 0 {
		return fmt.Errorf("fmc: cannot determine sequence count")
	}
	s.numShots = numShots

	frameWords, err := spike.GetInt(fmt.Sprintf("dev#1.grp#%d.frame_size", s.opt.Group), spike.UnitNone)
	if err != nil || frameWords <= fmcHeaderWords {
		return fmt.Errorf("fmc frame_size: %d, %v", frameWords, err)
	}
	points, err := spike.GetInt(fmt.Sprintf("dev#1.grp#%d.size_of_digitalization", s.opt.Group), spike.UnitNone)
	if err != nil || points <= 0 {
		points = cfg.Digitization.Points
	}
	// Header width is NOT a constant. It is 32 words in FMC, and FMC_TDM
	// widens it with sampling rate (32/64/128 at 50/100/200 MHz, manual
	// 2.8.6.19.2). Rather than assume, solve for the width that makes the
	// frame decompose into a whole number of plausible channels — then the
	// same code arms both modes and a wrong guess fails here with the
	// candidates printed instead of silently mis-slicing every frame.
	lead, channels := 0, 0
	for _, w := range []int{fmcHeaderWords, 2 * fmcHeaderWords, 4 * fmcHeaderWords} {
		d := frameWords - w
		if d > 0 && points > 0 && d%points == 0 {
			if ch := d / points; ch > 0 && ch <= cfg.Scan.ProbeElements {
				lead, channels = w, ch
				break
			}
		}
	}
	if channels == 0 {
		return fmt.Errorf(
			"fmc frame: %d words with %d points does not decompose with a 32, 64 or 128-word header",
			frameWords, points)
	}
	if lead != fmcHeaderWords {
		s.logf("info", "fmc: %d-word frame header (wider than FMC's %d — FMC_TDM at elevated sampling)",
			lead, fmcHeaderWords)
	}

	// The board's readback is AUTHORITATIVE for framing, and it legitimately
	// differs from the config: dev#1 clamps size_of_digitalization to what
	// fits its fixed frame buffer (in FMC_TDM, (24576-128)/32 = 764 points).
	//
	// An earlier revision made a mismatch here a hard refusal, to catch the
	// stride slip where the reader parses one frame length while the board
	// emits another. That was the wrong place for it: a normal clamp is
	// indistinguishable from a slip at this point, and refusing meant
	// acquisition silently never started. The stride is now protected
	// properly downstream — the header-width solver above and the timer-delta
	// check in lockPhase both verify framing against the data itself.
	if points != cfg.Digitization.Points {
		s.logf("warn", "board is running %d points, applied config asked for %d; "+
			"using the board's value for framing", points, cfg.Digitization.Points)
	}

	// In FMC_TDM the board may deliver the full aperture in one wide frame
	// (channels == RxCount) or as one firing's worth at a time
	// (channels == RxCount/FiringsPerTx). Both are legal; anything else is
	// the config and the board disagreeing and is worth saying loudly.
	if rx := cfg.Scan.RxCount; rx > 0 {
		if fp := cfg.Scan.FiringsPerTx(); channels != rx && channels != rx/fp {
			s.logf("warn", "board reports %d channels; config says rxCount %d (firings/tx %d)", channels, rx, fp)
		} else if channels != rx {
			s.logf("info", "fmc_tdm: %d channels per frame, %d firings per tx position", channels, fp)
		}
	}

	if channels <= 0 || channels > 64 {
		return fmt.Errorf("fmc frame: implausible channel count %d (frame %d, points %d)", channels, frameWords, points)
	}
	// Nominal wire frame per manual 2.8.6.19.2: header + matrix. The
	// calibration corrects the lead if this nominal is wrong for the mode
	// (e.g. FMC_TDM's wider headers).
	wireFrame := channels*points + lead

	// Refuse to arm a configuration the link cannot carry. The board produces
	// frameBytes/syncPeriod continuously whatever the host manages to pull;
	// past the ceiling its FIFO discards stream segments, cycle framing slips
	// through the holes, and nothing reports an error — the display just
	// reshuffles forever. Config validation catches this too, but the board's
	// settled values are the ones that matter and they are only readable here.
	framePeriodUs := cfg.Timing.SyncPeriodUs * float64(cfg.Scan.FiringsPerTx())
	if mbs := float64(2*channels*points+2*lead) / (framePeriodUs * 1e-6) / (1 << 20); mbs > 186 {
		return fmt.Errorf("configuration produces %.0f MB/s, above the link ceiling; raise sync period or lower points", mbs)
	}

	// Manual 3.3/3.5 flow: acquisition, then firing. Deliberately NOTHING
	// else touches the data path here: the fmcprobe bisection proved that
	// the vendor-faithful sequence — params, settings, enable, initiate,
	// blocking reads, and NO available_data polling or flush reads — works
	// on every configuration and survives mid-session reconfiguration,
	// while every host revision that mixed polling/flush reads into the
	// transfer discipline wedged. Manual 3.5's "no more wasted time
	// interrogating the device" is a contract, not an optimization.
	if _, _, err := spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 1); err != nil {
		return err
	}
	if _, _, err := spike.SetInt("dev#1.enable_prf", spike.UnitNone, 1); err != nil {
		_, _, _ = spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 0)
		return err
	}

	ctx, cancel := context.WithCancel(context.Background())
	a := &acquisition{cancel: cancel, done: make(chan struct{}), mode: mode, cycles: cycles}
	s.acq = a
	s.wasFiring = true
	s.opt.Store.Update(func(sn *state.Snapshot) { sn.Status.Acq = "firing" })

	cycleHz := 0.0
	if framePeriodUs > 0 {
		cycleHz = 1 / (float64(numShots) * framePeriodUs * 1e-6)
	}
	s.logf("info", "fmc acquisition started (%s): %d shots x %d ch x %d pts, frame_size %d, wire frame %d words, %.1f cycles/s",
		mode, numShots, channels, points, frameWords, wireFrame, cycleHz)
	s.logf("info", "fmc geometry: header %d words, %d firings/tx, frame period %.0f us",
		lead, cfg.Scan.FiringsPerTx(), framePeriodUs)

	s.pipeStart()
	go s.readLoopFMC(ctx, a, cfg, wireFrame, points, channels)
	return nil
}

// stopAcqLocked disables firing and waits for the read loop to exit. It reports
// whether the loop actually drained.
//
// A false return is load-bearing, not advisory: the loop is only ever leaked
// because a blocking dev#1.data read did not return, which means the library
// may still have a transfer in flight on the OS thread that read pinned.
// Writing dev#1.settings underneath that is the classic wedge (invariant #2:
// the transfer method owns the link). Callers that were about to reconfigure
// the board must refuse and force a close/reopen instead.
func (s *Supervisor) stopAcqLocked() (clean bool) {
	s.pipeStop()
	if s.acq == nil {
		return true
	}
	a := s.acq
	s.acq = nil
	a.cancel()

	// Disable BEFORE waiting for the loop: the FMC path uses the manual 3.5
	// transfer method whose dev#1.data reads BLOCK until a packet fills —
	// with the firing stopped first, the pending read returns (or errors)
	// instead of pinning the supervisor forever.
	s.opt.Store.Update(func(sn *state.Snapshot) { sn.Status.Acq = "draining" })
	_, _, _ = spike.SetInt("dev#1.enable_prf", spike.UnitNone, 0)
	_, _, _ = spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 0)

	select {
	case <-a.done:
		clean = true
	case <-time.After(3 * time.Second):
		// A blocking read the disable didn't unstick. Leak the goroutine
		// (it exits whenever the read finally returns) rather than hang the
		// whole supervisor; the OS thread it pinned goes with it. The board
		// is now in an unknown transfer state — see the doc comment.
		s.logf("warn", "acquisition loop did not exit within 3s; leaking it")
		clean = false
	}

	s.opt.Store.Update(func(sn *state.Snapshot) { sn.Status.Acq = "idle" })
	s.wasFiring = false
	s.logf("info", "acquisition stopped (clean=%v)", clean)
	return clean
}

// StartAcq / StopAcq are the API entry points; both marshal onto the
// supervisor goroutine via do().
func (s *Supervisor) StartAcq(ctx context.Context, mode string, cycles int) error {
	return s.do(ctx, func() error { return s.startAcqLocked(mode, cycles) })
}

func (s *Supervisor) StopAcq(ctx context.Context) error {
	return s.do(ctx, func() error {
		if !s.stopAcqLocked() {
			s.needsReopen = true
			s.opt.Store.Update(func(sn *state.Snapshot) { sn.State.NeedsReopen = true })
			return fmt.Errorf("read loop did not drain; reopen the device before reconfiguring")
		}
		return nil
	})
}
