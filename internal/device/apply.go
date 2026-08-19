package device

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"time"

	"echosight/internal/config"
	"echosight/internal/state"
	"echosight/internal/wire"
	"echosight/spike"
)

// Apply performs the documented stop, apply, restart sequence.
//
// INVARIANT #6: apply = stop -> drain -> clear settings -> write scenario ->
// exec plan -> restart-if-was-firing. 2.8.7.1 requires the clear before the
// write or the library merges the old and new scenarios.
//
// The no-op short-circuit below is not an optimisation. Rewriting a scenario
// against a board whose readout engine is mid-transfer is the operation that
// wedges it (invariants #1/#2), and the single most common way that happened
// in the field was an operator pressing Apply having changed nothing. If
// staged and applied are byte-identical there is nothing the board could
// possibly learn from the write, so it does not happen.
func (s *Supervisor) Apply(ctx context.Context) (wire.ApplyResult, error) {
	var res wire.ApplyResult
	err := s.do(ctx, func() error {
		s.mu.RLock()
		staged, applied := s.staged, s.applied
		s.mu.RUnlock()

		if same, err := configEqual(staged, applied); err == nil && same {
			s.logf("info", "apply: config unchanged, board untouched")
			res = wire.ApplyResult{
				Applied: []string{}, Clipped: []wire.Clip{}, Errors: []string{},
				Skipped: "config unchanged",
			}
			return nil
		}

		var e error
		res, e = s.applyLocked(staged)
		return e
	})
	return res, err
}

// configEqual compares two configs by their canonical JSON. Marshal-compare
// rather than reflect.DeepEqual because the wire form is what persists and what
// the frontend diffs; if two configs serialize identically, nothing downstream
// can tell them apart.
func configEqual(a, b wire.Config) (bool, error) {
	ab, err := json.Marshal(a)
	if err != nil {
		return false, err
	}
	bb, err := json.Marshal(b)
	if err != nil {
		return false, err
	}
	return bytes.Equal(ab, bb), nil
}

func (s *Supervisor) progress(step, detail string) {
	s.opt.Broker.PublishJSON(wire.ApplyProgressMsg{T: "applyProgress", Step: step, Detail: detail})
}

// settleAfterStop is the pause between a clean read-loop exit and the first
// scenario write. The goroutine has returned by then, but the library's own
// transfer teardown runs on the OS thread the read pinned and is not
// observable from Go. Bench-tune; 100-200 ms has been sufficient.
const settleAfterStop = 150 * time.Millisecond

// drainBoardLocked empties whatever the board still holds, and is the step
// whose absence made every live Apply produce a scrambled stream.
//
// Stopping the read loop stops US reading. It does not empty the board's
// FIFO, and it does not un-produce the records already sitting in it — records
// laid out under the OUTGOING geometry. The scenario is then rewritten and
// acquisition restarted, and the fresh read loop's very first bufferful is
// old-format residue followed by new-format stream, with a seam somewhere in
// the middle. detectHeaderGrid cannot find a consistent grid across that seam,
// the autocorrelation fallback recovers the frame size but not the phase, and
// the timer sanity test then reads sample data as a header and reports
// nonsense intervals like 2.47e17 us.
//
// The field log settled this. Discarding and retrying on later buffers was
// tried first, on the theory that the residue was finite: four attempts, ~9
// cycles and 14.7 M words discarded, and the header grid was missing on EVERY
// one of them. That is not what bounded residue looks like — the board keeps
// emitting the old layout until the FIFO is properly drained. What actually
// recovered the session was the resulting error tearing the device down and
// reopening it, which drains as a side effect. This does the drain
// deliberately instead, and costs milliseconds rather than a seven-second
// close and reopen.
//
// Legal to call here and only here: the read loop has exited, so s.acq is nil,
// so quietDevice() is false. The 3.5 transfer-method prohibition on
// interleaved device calls applies to a LIVE collection; this is the gap
// between two of them.
func (s *Supervisor) drainBoardLocked() int {
	remaining, err := spike.GetInt("dev#1.flush_data", spike.UnitNone)
	if err != nil {
		s.logf("warn", "apply: flush_data query failed (%v); the first frames after restart may be scrambled", err)
		return 0
	}
	drained := 0
	// Bounded: a board that reports data forever must not hang the apply.
	for i := 0; remaining > 0 && i < 64; i++ {
		avail, err := spike.GetInt("dev#1.available_data", spike.UnitNone)
		if err != nil || avail <= 0 {
			break
		}
		buf := make([]uint16, avail)
		n, err := spike.GetData("dev#1.data", avail, buf)
		if err != nil {
			s.logf("warn", "apply: drain read failed after %d words (%v)", drained, err)
			break
		}
		drained += n
		if remaining, err = spike.GetInt("dev#1.flush_data", spike.UnitNone); err != nil {
			break
		}
	}
	return drained
}

// applyLocked must run on the supervisor goroutine.
func (s *Supervisor) applyLocked(cfg wire.Config) (wire.ApplyResult, error) {
	if !s.opt.Store.Snapshot().State.DeviceOpen {
		return wire.ApplyResult{}, fmt.Errorf("device is not open")
	}

	// INVARIANT #10, promoted from warning to refusal. The board's firmware
	// mode is set at boot from config-devices.ini and must never be switched
	// at runtime (invariant #1). A board that came up in PA firmware will
	// silently BEAMFORM an FMC scan — one summed record per shot where the
	// imaging path expects raw channels — and produce images that look
	// plausible and are wrong. There is nothing to fall back to any more, so
	// this is a hard stop, not a log line.
	if !s.firmwareOK() {
		return wire.ApplyResult{}, fmt.Errorf(
			"board booted in %s firmware; this server is FMC-only. Set Mode=FMC in config-devices.ini and power-cycle the board",
			s.opt.Store.Snapshot().Device.FirmwareMode)
	}

	// A previous stop that leaked its read loop means a transfer may still be
	// in flight on a pinned OS thread. Writing dev#1.settings underneath it is
	// precisely the wedge. Close/reopen is the only recovery that has ever
	// worked, so say so and refuse.
	if s.needsReopen {
		return wire.ApplyResult{}, fmt.Errorf(
			"read loop leaked on a previous stop; reopen the device before reconfiguring")
	}

	wasFiring := s.acq != nil

	s.progress("stopping", "")
	s.opt.Store.Update(func(sn *state.Snapshot) { sn.Status.Acq = "configuring" })

	if clean := s.stopAcqLocked(); !clean {
		s.needsReopen = true
		s.opt.Store.Update(func(sn *state.Snapshot) { sn.State.NeedsReopen = true })
		s.logf("error", "read loop did not drain; refusing to rewrite the scenario")
		return wire.ApplyResult{}, fmt.Errorf(
			"read loop did not drain within its deadline; the board was left untouched. Reopen the device (Reconnect) before applying")
	}
	if wasFiring {
		// Only meaningful when something was actually streaming; a cold apply
		// has nothing to settle from.
		time.Sleep(settleAfterStop)

		// Then throw away everything the board produced under the OUTGOING
		// geometry, before a single word of the new one exists. See
		// drainBoardLocked: without this the next read loop locks onto a
		// buffer that straddles two record layouts and never recovers.
		s.progress("draining", "")
		if n := s.drainBoardLocked(); n > 0 {
			s.logf("info", "apply: drained %d words of pre-apply stream", n)
		}
	}

	// The firing script defines how many sequences exist, so it is written
	// before the parameter plan.
	s.progress("script", "")
	sc, err := config.BuildScript(cfg)
	if err != nil {
		return wire.ApplyResult{Errors: []string{err.Error()}}, err
	}
	js, err := sc.JSON()
	if err != nil {
		return wire.ApplyResult{Errors: []string{err.Error()}}, err
	}
	// 2.8.7.1: clearing settings before writing a new scenario avoids the
	// library merging the two.
	if err := spike.SetStr("dev#1.settings", spike.UnitNone, ""); err != nil {
		return wire.ApplyResult{Errors: []string{err.Error()}}, err
	}
	if err := spike.SetStr("dev#1.settings", spike.UnitNone, js); err != nil {
		return wire.ApplyResult{Errors: []string{err.Error()}}, err
	}

	numSeq, err := spike.GetInt(fmt.Sprintf("dev#1.grp#%d.number_of_shots", s.opt.Group), spike.UnitNone)
	if err != nil || numSeq <= 0 {
		numSeq = len(sc.Groups[0].Sequences)
	}
	s.numShots = numSeq

	if want := cfg.Scan.TxPositions(); want > 0 && numSeq != want {
		s.logf("warn", "board reports %d shots but the geometry implies %d tx positions", numSeq, want)
	}

	s.progress("applying", fmt.Sprintf("%d sequences", numSeq))
	// NOTE: 3-arg form, matching a config/apply.go WITHOUT the restored
	// gate/DAC writes. If you later apply that change, Plan takes numSeq
	// as its third argument and this call needs it back.
	plan := config.Plan(cfg, s.opt.Group, s.fwMode)
	res, si := config.Exec(plan)

	// Name every failure. A bare count ("1 SI5G errors during apply") is
	// unactionable and hid a per-apply failure for the whole of this
	// project's debugging history — if the failing parameter is a reception
	// law, every scenario written has been silently partial and the board
	// has been falling back to its own defaults.
	for _, e := range res.Errors {
		s.logf("error", "apply failed: %s", e)
	}
	for _, e := range si {
		s.logf("error", "apply SI5G: op=%s param=%s code=%d (%s)", e.Op, e.Param, e.Code, e.Name)
	}
	if len(res.Clipped) > 0 {
		for _, c := range res.Clipped {
			s.logf("warn", "apply clipped: %s requested %g, board took %g", c.Path, c.Requested, c.Applied)
		}
	}
	if len(si) > 0 || len(res.Errors) > 0 {
		s.logf("warn", "%d SI5G errors, %d failed setters out of %d in the plan",
			len(si), len(res.Errors), len(plan))
	}

	// Re-read what the device actually settled on.
	s.frame, err = spike.ReadFrameLayout(s.opt.Group)
	if err != nil {
		res.Errors = append(res.Errors, err.Error())
	}
	if v, err := spike.GetDbl(fmt.Sprintf("dev#1.grp#%d.sync_period_min", s.opt.Group), spike.UnitUs); err == nil {
		s.syncMinUs = v
	}
	if v, err := spike.GetDbl(fmt.Sprintf("dev#1.grp#%d.sync_period", s.opt.Group), spike.UnitUs); err == nil && v > 0 {
		s.opt.Store.Update(func(sn *state.Snapshot) { sn.Status.PrfRequestedHz = 1e6 / v })
	}

	s.mu.Lock()
	s.applied = cfg
	s.staged = cfg
	s.mu.Unlock()

	// Travel-time tables, the profile's velocity and the geometry the imaging
	// worker derives its rx mapping from all came from the OLD applied config.
	// Bumping the generation makes the worker rebuild at the current fitted
	// water path on its next block instead of waiting for an unrelated
	// standoff drift to notice.
	s.bumpGeom()

	s.opt.Store.Update(func(sn *state.Snapshot) { sn.Status.HvOn = cfg.Device.HVEnable })
	if s.opt.OnApplied != nil {
		go s.opt.OnApplied(cfg)
	}

	if wasFiring {
		// Second drain, immediately before firing resumes. The parameter plan
		// re-enables HV and PRF, so the board can begin producing between the
		// plan and the read loop actually starting. That data is new-geometry
		// and would lock fine, but it is unread and unbounded, and discarding
		// it costs milliseconds against a failure that costs the session.
		if n := s.drainBoardLocked(); n > 0 {
			s.logf("info", "apply: drained %d words produced during reconfiguration", n)
		}
		s.progress("restarting", "")
		if err := s.startAcqLocked("continuous", 0); err != nil {
			res.Errors = append(res.Errors, err.Error())
		}
	} else {
		s.opt.Store.Update(func(sn *state.Snapshot) { sn.Status.Acq = "idle" })
	}
	s.progress("done", "")
	return res, nil
}