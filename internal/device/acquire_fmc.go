package device

import (
	"context"
	"fmt"
	"runtime"
	"time"

	"echosight/internal/state"
	"echosight/internal/wire"
	"echosight/spike"
)

// FMC-firmware acquisition (dev#1.mode 1 or 2).
//
// WIRE LAYOUT (manual 2.8.6.19.2): each frame is a header — 32 words in FMC
// (64-bit timer µs + 4x 32-bit encoders + reserved); FMC_TDM widens it with
// sampling rate (32/64/128 words at 50/100/200 MHz) — followed by
// channels*points int16 samples interleaved [sample][channel] (channel
// varies fastest). No PA metadata block, no footer, no gates, no
// ascans_on_demand.
//
// COLLECTION (manual 3.5, "how to collect FMC data efficiently"): the
// poll-available-then-read approach is the wrong contract for FMC rates and
// produced every pathology in this file's history — intermittent ReadPipe2
// zeros, short reads one USB packet shy, and post-apply wedges where
// available_data counted words the pipe would never deliver. The intended
// method initiates a large transfer (dev#1.initialize_data_transfer, units
// of 1 KB blocks) and then reads dev#1.data in packets; the read BLOCKS
// until the packet fills, so there is no polling and no race. Socomate
// benchmarks 316 MB/s on a 32x64 over USB3 this way.
//
// READ GRANULARITY (established by differential against cmd/fmcprobe): the
// library's ReadPipe2 wants KB-quantized read sizes. A firing cycle here is
// NEVER KB-aligned — frame bytes = 64*points+64, so a 8-frame cycle is
// 512*points+512, always 512 B off a 1 KB boundary — and every cycle-sized
// read in this file's history failed with nbBytes=0 at exactly such sizes
// (541184, 1049088, 614912...) while the probe's 1 MB packet reads never
// failed once. Therefore reads are fixed 1 MB packets, fully decoupled from
// frame geometry, and cycles are parsed out of an accumulator. Cycle phase
// is pure arithmetic: the stream starts at seq 1, so position mod cycle is
// the phase and no realignment reads are ever needed.
//
// A startup calibration still measures the true frame stride from the data
// (autocorrelation over a static scene) rather than trusting the
// frame_size/points arithmetic — settled record lengths and header variants
// have disagreed with readbacks in the field, and a constant stride error
// shows up as A-scans drifting in time and tx rows rotating through the
// matrix.

const fmcHeaderWords = 32

// Preview budget: the FMC display needs shape, not fidelity, but it should
// feel live under a moving probe. Records are decimated to <=512 points and
// the publish rate is whatever fits ~5 MB/s of WebSocket traffic — ~28 Hz at
// 700-point scans, ~20 Hz at 2048 — instead of a flat cap. A fixed 8 Hz cap
// (an earlier revision, sized for 1 MB bundles) made hand-scanning feel like
// slideshow. Full-rate data stays server-side for capture/reconstruction.
const (
	fmcPreviewBudgetBytes = 5 << 20 // per second, to clients
	fmcPreviewMaxPoints   = 512
	fmcPreviewMinHz       = 4
	fmcPreviewMaxHz       = 30
)

// fmcInitTransfer schedules kb 1-kilobyte blocks of bus transfer (manual
// 3.5). Reads then block until their packet fills.
func fmcInitTransfer(kb int) error {
	_, _, err := spike.SetInt("dev#1.initialize_data_transfer", spike.UnitNone, kb)
	return err
}

// readLoopFMC implements the manual 3.5 transfer method with fixed 1 MB
// packet reads (see the file comment for why read size must not follow the
// frame geometry) and an accumulator from which whole firing cycles are
// parsed.
func (s *Supervisor) readLoopFMC(ctx context.Context, a *acquisition, cfg wire.Config, wireFrame, points, channels int) {
	defer close(a.done)

	// Pin data reads to one OS thread, like the PA loops.
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()

	numShots := s.numShots
	cycleWords := wireFrame * numShots

	const packetWords = 512 * 1024 // 1 MB, probe-proven, KB- and 4MB-rule-safe
	const transferKB = 32 * 1024   // 32 MB per initiate, probe-proven

	transferWordsLeft := 0
	pkt := make([]uint16, packetWords)
	streamPos := 0 // words consumed since the stream (re)started at seq 1

	consecFails := 0
	rekicks := 0

	var lastReadErrAt time.Time
	readErrsSuppressed := 0
	logReadErr := func(err error) {
		if time.Since(lastReadErrAt) >= time.Second {
			lastReadErrAt = time.Now()
			if readErrsSuppressed > 0 {
				s.setErr("fmc read data: %v (%d more suppressed)", err, readErrsSuppressed)
			} else {
				s.setErr("fmc read data: %v", err)
			}
			readErrsSuppressed = 0
		} else {
			readErrsSuppressed++
		}
	}

	// rekick recovers a stalled stream: disable/enable only — no flush, no
	// polling. The stream restarts at seq 1; callers reset accumulator and
	// phase.
	rekick := func() bool {
		if rekicks >= 3 {
			return false
		}
		rekicks++
		s.logf("warn", "fmc stream stalled; re-kicking acquisition (%d/3)", rekicks)
		_, _, _ = spike.SetInt("dev#1.enable_prf", spike.UnitNone, 0)
		_, _, _ = spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 0)
		_, _, _ = spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 1)
		_, _, _ = spike.SetInt("dev#1.enable_prf", spike.UnitNone, 1)
		return true
	}

	// readPacket blocks until stream words arrive, re-initiating the bus
	// transfer as the schedule is consumed. Returns the number of words now
	// valid in pkt (usually a full packet); ok=false means ctx is done or
	// the acquisition must end (phase already set).
	//
	// CRITICAL: a SHORT read (0 < n < packetWords) still delivered n real
	// stream words. An earlier revision discarded them and retried, which
	// silently slipped the framing by n words until the next rekick — seen
	// in the field as the whole display reshuffling every ~20 s with the
	// probe untouched. Partial data is appended like any other data;
	// framing is preserved by construction. Only n<=0 counts as failure.
	readPacket := func() (int, bool) {
		for {
			select {
			case <-ctx.Done():
				return 0, false
			default:
			}
			if transferWordsLeft <= 0 {
				if err := fmcInitTransfer(transferKB); err != nil {
					if spike.IsDeviceGone(err) {
						s.setErr("fmc initialize_data_transfer: %v", err)
						s.opt.Store.SetPhase(state.PhaseDegraded)
						return 0, false
					}
					logReadErr(err)
					time.Sleep(50 * time.Millisecond)
					continue
				}
				transferWordsLeft = transferKB * 512
			}
			n, err := spike.GetData("dev#1.data", packetWords, pkt) // blocks
			if err != nil && spike.IsDeviceGone(err) {
				s.setErr("fmc read data: %v", err)
				s.opt.Store.SetPhase(state.PhaseDegraded)
				return 0, false
			}
			if n <= 0 {
				if err != nil {
					logReadErr(err)
				} else {
					logReadErr(fmt.Errorf("empty read"))
				}
				consecFails++
				transferWordsLeft = 0 // schedule position unknown: re-initiate
				if consecFails >= 10 {
					if !rekick() {
						s.setErr("fmc stream dead after %d re-kicks; forcing a device reconnect", rekicks)
						s.opt.Store.SetPhase(state.PhaseDegraded)
						return 0, false
					}
					consecFails = 0
				}
				continue
			}
			consecFails = 0
			transferWordsLeft -= n
			if n < packetWords {
				logReadErr(fmt.Errorf("short packet %d/%d words — appended, framing preserved", n, packetWords))
			}
			return n, true
		}
	}

	// --- Wire-frame + phase lock ------------------------------------------
	// The stream does NOT reliably begin at a cycle boundary: without a
	// stop-time drain (naked reads wedge the pipe), stale board-FIFO
	// residue from the previous geometry precedes the fresh stream, so
	// "position 0 = seq 1" is false after any re-apply. Field signature:
	// garbage frame timers, the next frame's header embedded at a fixed
	// sample index of channel 0, interface echoes hundreds of samples late,
	// and a TFM image of pure speckle. The lock is content-based and
	// wedge-safe: detectHeaderGrid reads the true frame stride AND phase
	// from the header zero runs, timers verify it, and the per-frame peak
	// channel identifies which frame is tx 0. Re-run after every rekick.
	frameLead := fmcHeaderWords
	skipWords := 0

	txStarts, _ := spike.FMCWindows(spike.FMCScanConfig{
		ProbeElements: cfg.Scan.ProbeElements,
		Aperture:      cfg.Scan.Aperture,
		Step:          cfg.Scan.Step,
		RxCount:       cfg.Scan.RxCount,
	})

	// lockPhase inspects buffered stream content beginning at absolute
	// stream position bufStart and returns how many words to discard so the
	// next parse boundary is a true tx-0 cycle start. It may update
	// wireFrame/points/cycleWords from the measured grid.
	lockWhy := ""
	lockPhase := func(buf []uint16, bufStart int) (int, bool) {
		lockWhy = ""
		F, delta, ok := detectHeaderGrid(buf)
		if !ok {
			if F2 := detectCycleFrame(buf, numShots, wireFrame, 2048); F2 > 0 {
				F, delta, ok = F2, 0, true
				s.logf("warn", "fmc lock: header grid not found; autocorrelation frame %d, phase unverified", F2)
			} else {
				lockWhy = "no header grid and no cycle periodicity in the buffer"
				return 0, false
			}
		}
		if F != wireFrame {
			lead := F - channels*points
			if lead != fmcHeaderWords && lead != 0 && F%channels == 0 {
				points = F / channels
				lead = 0
			}
			if lead < 0 {
				points = F / channels
				lead = F - channels*points
			}
			frameLead = lead
			wireFrame = F
			cycleWords = numShots * F
			if usable := (wireFrame - frameLead) / channels; usable < points {
				points = usable
			}
		}
		// Timer sanity across the first headers — and it is a TEST, not a
		// log line. Consecutive frame headers are one sync period apart by
		// construction, so reading the timer at +wireFrame is an independent
		// check on the stride: if the stride is wrong, the words read at that
		// offset are not a header at all and the delta is garbage.
		//
		// This is the check that would have caught a board reporting one
		// record length while emitting another (readbacks have lied in the
		// field before — see the file header). Previously the delta was
		// printed and then trusted anyway, so a bad grid locked silently and
		// the only symptom downstream was the interface arrival marching
		// through the record.
		syncUs := cfg.Timing.SyncPeriodUs
		// The subtraction is SIGNED in intent even though the timers are
		// uint64. Reading two headers that straddle a re-apply gives a second
		// timer SMALLER than the first, and an unsigned difference then wraps
		// to ~2^64 — which is what produced the 18437736857273892824 us
		// interval in the field report. That number is not a measurement of
		// anything; it is a negative delta in disguise, and printing it hid
		// the actual finding, which is that the two headers came from
		// different streams.
		timerOK := func(F int) (int64, bool) {
			if syncUs <= 0 || delta+2*F+4 > len(buf) {
				return 0, true // cannot test; do not block on it
			}
			t0, t1 := headerTimer(buf, delta), headerTimer(buf, delta+F)
			if t1 < t0 {
				return -int64(t0 - t1), false // timer went backwards
			}
			d := int64(t1 - t0)
			return d, d >= int64(syncUs*0.8) && d <= int64(syncUs*1.25)
		}
		if d, ok := timerOK(wireFrame); !ok {
			// The measured grid disagrees with the clock. Fall back to the
			// nominal arithmetic and see whether THAT validates before giving
			// up: a spurious periodicity in the content is a likelier failure
			// than the sync period being wrong.
			nominal := channels*points + fmcHeaderWords
			if nd, nok := timerOK(nominal); nok && nominal != wireFrame {
				s.logf("warn", "fmc lock: grid frame %d gave timer delta %d us (sync %.0f us); "+
					"falling back to nominal frame %d (delta %d us)", wireFrame, d, syncUs, nominal, nd)
				wireFrame = nominal
				frameLead = fmcHeaderWords
				cycleWords = numShots * nominal
			} else {
				// NOT a device error. A failed timer test here most often
				// means the buffer straddles a re-apply — stale residue from
				// the previous geometry ahead of fresh stream — which is a
				// condition that clears itself once the residue is past. It
				// used to call setErr, which put the device into an error
				// state that only a server restart cleared, for something the
				// next buffer would have fixed. The caller retries.
				if d < 0 {
					lockWhy = fmt.Sprintf("frame %d words: header timer went backwards by %d us "+
						"(the two headers are from different streams — buffer straddles a re-apply)",
						wireFrame, -d)
				} else {
					lockWhy = fmt.Sprintf("frame %d words gives a %d us header interval against a "+
						"%.0f us sync period", wireFrame, d, syncUs)
				}
				return 0, false
			}
		}
		if d, _ := timerOK(wireFrame); true {
			s.logf("info", "fmc lock: frame %d words, header phase %d, timer delta %d us (sync %.0f us)",
				wireFrame, delta, d, syncUs)
		}
		r0 := 0
		if len(txStarts) == numShots {
			// Use the first position with a full cycle of frames available.
			r0 = detectTxRotation(buf, delta, wireFrame, frameLead, channels, points, numShots, cfg.Scan.Aperture, txStarts)
		}
		// Absolute stream position of the next tx-0 cycle boundary.
		boundary := bufStart + delta + ((numShots-r0)%numShots)*wireFrame
		end := bufStart + len(buf)
		for boundary < end {
			boundary += cycleWords
		}
		s.logf("info", "fmc lock: tx rotation %d, next cycle boundary at stream word %d (skip %d)", r0, boundary, boundary-end)
		return boundary - end, true
	}

	// Shared by the startup lock below and the relock path further down. At
	// ~39 cycles/s a handful of attempts is a fraction of a second of blank
	// display, and it is what separates "stale residue from an apply", which
	// clears itself, from "the board is lying about its record length", which
	// does not.
	const maxRelockTries = 8

	{
		need := cycleWords*2 + 8192
		cal := make([]uint16, 0, need+packetWords)
		calStart := 0
		for len(cal) < need {
			beforeKicks := rekicks
			n, ok := readPacket()
			if !ok {
				return
			}
			if rekicks != beforeKicks {
				cal = cal[:0]
				streamPos = 0
				calStart = 0
			}
			cal = append(cal, pkt[:n]...)
			streamPos += n
		}
		// Retry on FRESH content rather than parsing unlocked.
		//
		// This is the path that runs after every config apply, and it is the
		// one that mattered in the field. On a cold start the board's FIFO is
		// empty, so stream position 0 really is the start of a cycle and the
		// first attempt succeeds. On an apply the board is reconfigured with
		// data still in flight: residue from the PREVIOUS geometry sits ahead
		// of the fresh stream, this buffer straddles the seam, the header grid
		// is not consistent across it, and the timer test correctly rejects
		// the result. That is exactly the message the field log showed —
		// "the two headers are from different streams".
		//
		// The old response was to log a warning and set skipWords = 0, i.e.
		// parse from stream start with a phase known to be wrong. Every record
		// then lands at the wrong offset for the rest of the session, which is
		// the speckle B-scan and the confidently wrong wall numbers, and the
		// only listed cure — "until a rekick relocks" — never fired because
		// nothing rekicks on its own. A server restart was the only way out.
		//
		// The residue is bounded, so the cure is simply later data. Discard
		// what was examined, refill, and try again.
		for try := 1; ; try++ {
			skip, ok := lockPhase(cal, calStart)
			if ok {
				skipWords = skip
				break
			}
			if try >= maxRelockTries {
				s.setErr("fmc lock failed %d times: %s. Captures and imaging would be "+
					"silently wrong, so acquisition is stopping rather than parsing unlocked",
					try, lockWhy)
				s.opt.Store.SetPhase(state.PhaseDegraded)
				return
			}
			s.logf("warn", "fmc lock attempt %d/%d failed (%s); discarding %d words and retrying on fresh stream",
				try, maxRelockTries, lockWhy, len(cal))
			calStart = streamPos
			cal = cal[:0]
			for len(cal) < need {
				beforeKicks := rekicks
				n, ok := readPacket()
				if !ok {
					return
				}
				if rekicks != beforeKicks {
					cal = cal[:0]
					streamPos = 0
					calStart = 0
				}
				cal = append(cal, pkt[:n]...)
				streamPos += n
			}
		}
	}
	// ----------------------------------------------------------------------

	s.fmcGeom.Store(&fmcGeometry{
		NumShots: numShots, Channels: channels, Points: points,
		WireFrame: wireFrame, FrameLead: frameLead, CycleWords: cycleWords,
	})
	s.pipe.geomGen.Add(1)

	relock := false
	// (maxRelockTries is declared above, before the calibration block.)
	// A lock attempt over a buffer that straddles a re-apply legitimately
	// fails; the residue is finite so a later buffer succeeds. Retry rather
	// than either giving up or, worse, parsing unlocked. The cap is what
	// separates "stale residue" from "the board really is lying about its
	// record length" — at ~39 cycles/s a handful of attempts is well under a
	// second of blank display.
	relockTries := 0

	// Accumulator: packets in, whole cycles out. Capacity covers the worst
	// case of a nearly-complete cycle plus one fresh packet plus the initial
	// phase skip.
	acc := make([]uint16, 0, 2*cycleWords+packetWords)

	slab := make([]uint16, numShots*channels*points) // holds previewPoints/record
	recs := make([]wire.AscanRecord, 0, numShots*channels)

	var encBufs [2][]byte
	encIdx := 0

	decim := 1
	for points/decim > fmcPreviewMaxPoints {
		decim++
	}
	previewPoints := points / decim
	if previewPoints < 1 {
		previewPoints = 1
	}

	// Publish rate: fill the byte budget with the actual bundle size. The
	// budget is live-adjustable (SetPreviewBudget); rateFor re-derives the
	// gap and is re-run every stats tick so a slider in the app takes
	// effect within a second, with no device interaction.
	bundleBytes := wire.BundleHeaderBytes + numShots*channels*(wire.RecordHeaderBytes+2*previewPoints)
	previewHz := 0
	var minPublishGap time.Duration
	rateFor := func() {
		hz := s.PreviewBudgetBytes() / bundleBytes
		if hz < fmcPreviewMinHz {
			hz = fmcPreviewMinHz
		}
		if hz > fmcPreviewMaxHz {
			hz = fmcPreviewMaxHz
		}
		if cfg.Acquisition.PreviewHz > 0 && cfg.Acquisition.PreviewHz < hz {
			hz = cfg.Acquisition.PreviewHz
		}
		previewHz = hz
		minPublishGap = time.Second / time.Duration(hz)
	}
	rateFor()
	s.logf("info", "fmc preview: %d pts decimated %dx -> %d pts/record, %d KB bundles at %d Hz (%.1f MB/s to clients)",
		points, decim, previewPoints, bundleBytes/1024, previewHz,
		float64(bundleBytes*previewHz)/(1024*1024))
	s.logf("info", "fmc collection: %d KB packets, %d MB transfers, cycle %d words parsed from accumulator",
		packetWords*2/1024, transferKB/1024, cycleWords)

	var (
		lastLossWarn   time.Time
		lastPublish    time.Time
		lastStat       = time.Now()
		wordsSince     int
		framesSince    int
		cyclesDone     int
		publishedBytes int
	)

	processCycle := func(cycle []uint16) {
		cyclesDone++
		framesSince += numShots
		s.captureCycle(cycle)
		s.pipeOffer(cycle)

		now := time.Now()
		if now.Sub(lastPublish) >= minPublishGap {
			lastPublish = now
			recs = recs[:0]
			var meta wire.BundleMeta
			meta.Group = uint8(s.opt.Group)
			meta.HostTimeUs = uint64(time.Now().UnixMicro())

			for fi := 0; fi < numShots; fi++ {
				frame := cycle[fi*wireFrame : (fi+1)*wireFrame]
				var timerUs uint64
				if frameLead >= 12 {
					timerUs = uint64(frame[0]) | uint64(frame[1])<<16 | uint64(frame[2])<<32 | uint64(frame[3])<<48
					if fi == 0 {
						for e := 0; e < 4; e++ {
							meta.Encoders[e] = int32(uint32(frame[4+e*2]) | uint32(frame[5+e*2])<<16)
						}
					}
				}
				data := frame[frameLead:]
				for c := 0; c < channels; c++ {
					dst := slab[(fi*channels+c)*previewPoints : (fi*channels+c+1)*previewPoints]
					if decim == 1 {
						for smp := 0; smp < points; smp++ {
							dst[smp] = data[smp*channels+c]
						}
					} else {
						for o := 0; o < previewPoints; o++ {
							best := int16(0)
							mag := int32(-1)
							for j := 0; j < decim; j++ {
								v := int16(data[(o*decim+j)*channels+c])
								a := int32(v)
								if a < 0 {
									a = -a
								}
								if a > mag {
									mag, best = a, v
								}
							}
							dst[o] = uint16(best)
						}
					}
					idx := uint16(fi*channels + c)
					recs = append(recs, wire.AscanRecord{
						FiringIndex:  idx,
						SpatialIndex: idx,
						TimerUs:      timerUs,
						// Always HF: FMC delay-and-sum needs signed RF, and
						// the apply plan pins scope_video_mode to 0.
						Kind:         0,
						Samples:      dst,
					})
				}
			}

			if s.opt.Pose != nil {
				p, age, ok := s.opt.Pose.At(meta.HostTimeUs)
				meta.Pose = p
				meta.Pose.Valid = ok
				s.opt.Store.Update(func(sn *state.Snapshot) {
					sn.PoseValid = ok
					sn.PoseAgeMs = age
				})
			}

			s.bundleSeq++
			meta.BundleSeq = s.bundleSeq
			encBufs[encIdx] = wire.EncodeBundle(encBufs[encIdx], meta, recs)
			publishedBytes += len(encBufs[encIdx])
			s.opt.Broker.PublishBinary(encBufs[encIdx])
			encIdx ^= 1
		}

		if d := time.Since(lastStat); d >= time.Second {
			rateFor() // pick up live budget changes
			mbs := float64(wordsSince*2) / d.Seconds() / (1024 * 1024)

			// Bus-integrity check: the board produces cycleBytes/syncPeriod
			// per shot regardless of what the host manages to pull. When
			// measured throughput sits materially below production with an
			// empty backlog and no read errors, the board-side FIFO is
			// overflowing and DISCARDING stream segments — framing then
			// slips through the missing chunks and no amount of host-side
			// locking can hold it (field: 1200 pts = 220 MB/s predicted vs
			// a ~186 MB/s SBC USB ceiling, display reshuffling endlessly).
			// The only fix is reducing the data rate below the ceiling.
			if cfg.Timing.SyncPeriodUs > 0 {
				produced := float64(cycleWords*2) / (float64(numShots) * cfg.Timing.SyncPeriodUs * 1e-6) / (1 << 20)
				if produced > 1 && mbs < produced*0.93 {
					if time.Since(lastLossWarn) > 15*time.Second {
						lastLossWarn = time.Now()
						s.setErr("bus saturated: receiving %.0f of %.0f MB/s — the board is discarding data and framing cannot hold. Reduce points or PRF below the link ceiling", mbs, produced)
					}
				}
			}
			pmbs := float64(publishedBytes) / d.Seconds() / (1024 * 1024)
			budget := float64(s.PreviewBudgetBytes()) / (1 << 20)
			hz := previewHz
			drops := s.opt.Broker.TotalDropped()
			s.opt.Store.Update(func(sn *state.Snapshot) {
				sn.Status.DataRateMBs = mbs
				sn.Status.FramesPerSec = int(float64(framesSince) / d.Seconds())
				// Cycles, not frames, is the number every other rate in the
				// system is derived from: TFM image rate = cycles/s / K.
				if numShots > 0 {
					sn.Status.CyclesPerSec = float64(framesSince) / d.Seconds() / float64(numShots)
				}
				sn.Status.PrfActualHz = float64(framesSince) / d.Seconds()
				sn.Status.DroppedFrames = int(drops)
				sn.Status.BacklogWords = 0
				sn.Status.PreviewHz = hz
				sn.Status.PreviewMBs = pmbs
				sn.Status.PreviewBudget = budget
				sn.Clients = s.opt.Broker.Count()
				sn.ClientDrops = drops
			})
			wordsSince, framesSince, publishedBytes, lastStat = 0, 0, 0, time.Now()
		}
	}

	for {
		beforeKicks := rekicks
		n, ok := readPacket()
		if !ok {
			return
		}
		if rekicks != beforeKicks {
			// The stream restarted during the read. Position-zero alignment
			// cannot be assumed (same stale-residue trap as startup), so
			// drop everything and relock from fresh content.
			acc = acc[:0]
			streamPos = 0
			skipWords = 0
			relock = true
		}
		acc = append(acc, pkt[:n]...)
		streamPos += n
		wordsSince += n

		if relock {
			if len(acc) < cycleWords*2+8192 {
				continue
			}
			prevFrame := wireFrame
			if skip, ok := lockPhase(acc, streamPos-len(acc)); ok {
				if wireFrame != prevFrame {
					// The measured grid changed without an apply: buffers are
					// sized for the old geometry. Anomalous — hand it to the
					// reconnect machine rather than risk an overrun.
					s.setErr("fmc relock measured frame %d vs %d in-session; forcing reconnect", wireFrame, prevFrame)
					s.opt.Store.SetPhase(state.PhaseDegraded)
					return
				}
				skipWords = len(acc) + skip // discard buffer + run-in to the boundary
				relockTries = 0
			} else {
				// DO NOT parse unlocked. That is what turns a recoverable
				// straddle into the failure the operator actually sees: the
				// frame grid is wrong, every record is read at the wrong
				// offset, and the result is a full-speckle B-scan with a
				// jagged surface line and confident wall numbers computed
				// from noise. Silence for a few hundred milliseconds is a
				// much better failure than fluent nonsense.
				//
				// The residue is finite, so the cure is fresh content: drop
				// what was examined and try again on the next bufferful.
				relockTries++
				acc = acc[:0]
				if relockTries >= maxRelockTries {
					s.setErr("fmc lock failed %d times: %s. The board is emitting a different "+
						"record length than it reports; captures and imaging would be silently wrong",
						relockTries, lockWhy)
					s.opt.Store.SetPhase(state.PhaseDegraded)
					return
				}
				s.logf("warn", "fmc relock attempt %d/%d failed (%s); waiting for fresh stream",
					relockTries, maxRelockTries, lockWhy)
				continue
			}
			relock = false
		}

		// Discard the phase remainder once, then extract whole cycles.
		if skipWords > 0 {
			if len(acc) < skipWords {
				continue
			}
			acc = acc[:copy(acc, acc[skipWords:])]
			skipWords = 0
		}
		nCycles := len(acc) / cycleWords
		for i := 0; i < nCycles; i++ {
			processCycle(acc[i*cycleWords : (i+1)*cycleWords])
			if a.mode == "snapshot" && a.cycles > 0 && cyclesDone >= a.cycles {
				s.logf("info", "fmc snapshot complete (%d cycles)", cyclesDone)
				return
			}
		}
		if nCycles > 0 {
			acc = acc[:copy(acc, acc[nCycles*cycleWords:])]
		}
	}
}

// detectCycleFrame finds the wire frame size by autocorrelation: for a static
// scene the stream repeats with period numShots*F, so the true F maximizes
// sum(|x[i]|*|x[i+numShots*F]|). Search F in [nominal-span, nominal+span].
// Returns 0 when the buffer is too short for the search.
func detectCycleFrame(a []uint16, numShots, nominalF, span int) int {
	mag := make([]int32, len(a))
	for i, w := range a {
		v := int32(int16(w))
		if v < 0 {
			v = -v
		}
		mag[i] = v
	}
	const stride = 29
	bestF, bestS := 0, int64(-1)
	for F := nominalF - span; F <= nominalF+span; F++ {
		if F <= 0 {
			continue
		}
		lag := numShots * F
		limit := len(a) - lag
		if limit > numShots*nominalF {
			limit = numShots * nominalF // compare one nominal cycle's worth
		}
		if limit < 8192 {
			continue
		}
		var sum int64
		for i := 0; i < limit; i += stride {
			sum += int64(mag[i]) * int64(mag[i+lag])
		}
		if sum > bestS {
			bestS, bestF = sum, F
		}
	}
	return bestF
}

// detectHeaderGrid locates the true frame grid from content. Header words
// 12..31 stream as exact zeros (manual 2.8.6.19.2.1; encoders in 4..11 are
// also zero with nothing connected), and twenty consecutive near-zero words
// never occur inside signal. Maximal silent runs therefore mark headers:
// each run ends at header word 31, so a run ending at e implies a header
// start at e-31. Fitting the arithmetic progression of those starts yields
// the true frame stride AND its phase, immune to frame_size lies, settled
// points, and stale-residue offsets. ok=false when no consistent grid.
func detectHeaderGrid(a []uint16) (stride, phase int, ok bool) {
	// Anchor on where each silent run STARTS, not ends: the run's tail can
	// extend into near-zero early-time samples (field evidence: a stride
	// measured 4 words short, betrayed by the next header drifting +4
	// words/frame through the parse), but its head is pinned by timer word
	// 1, which is nonzero for any uptime past 65 ms. Stride = majority gap
	// between run starts (exact, since the anchor bias is constant); the
	// start's offset inside the header (2 with encoders idle and uptime
	// under ~71 min, 3 after, 12 with encoders active) is resolved by
	// testing which one yields sane, consistent timers.
	var starts []int
	run := 0
	for w := 0; w <= len(a); w++ {
		zero := w < len(a) && a[w] == 0
		if zero {
			run++
			continue
		}
		if run >= 18 && run <= 40 {
			starts = append(starts, w-run)
		}
		run = 0
	}
	if len(starts) < 4 {
		return 0, 0, false
	}
	gaps := map[int]int{}
	for i := 1; i < len(starts); i++ {
		gaps[starts[i]-starts[i-1]]++
	}
	best, votes := 0, 0
	for g, n := range gaps {
		if n > votes {
			best, votes = g, n
		}
	}
	if best <= 64 || votes < 3 {
		return 0, 0, false
	}
	// Majority residue = the consistent run-start phase.
	res := map[int]int{}
	for _, h := range starts {
		res[h%best]++
	}
	bestR, rv := 0, 0
	for r, n := range res {
		if n > rv {
			bestR, rv = r, n
		}
	}
	if rv < 3 {
		return 0, 0, false
	}
	// Resolve the run-start offset within the header via timer sanity:
	// consecutive frame timers must advance by a positive, consistent,
	// plausible amount.
	for _, off := range []int{2, 3, 12, 1, 4} {
		h0 := bestR - off
		for h0 < 0 {
			h0 += best
		}
		if h0+2*best+4 > len(a) {
			continue
		}
		t0 := headerTimer(a, h0)
		t1 := headerTimer(a, h0+best)
		t2 := headerTimer(a, h0+2*best)
		d1, d2 := int64(t1-t0), int64(t2-t1)
		if d1 > 10 && d1 < 1_000_000 && d2 > 10 && d2 < 1_000_000 {
			diff := d1 - d2
			if diff < 0 {
				diff = -diff
			}
			if diff*20 <= d1 { // within 5%
				return best, h0, true
			}
		}
	}
	return 0, 0, false
}

// headerTimer decodes the 64-bit microsecond timer at a header start.
func headerTimer(a []uint16, h int) uint64 {
	return uint64(a[h]) | uint64(a[h+1])<<16 | uint64(a[h+2])<<32 | uint64(a[h+3])<<48
}

// detectTxRotation identifies which tx index the frame at `delta` carries by
// matching each frame's peak receive channel against the expected position
// of the transmit-aperture center inside its sliding window. Returns r0 such
// that the frame at delta carries tx r0.
func detectTxRotation(a []uint16, delta, frame, lead, channels, points, numShots, aperture int, txStarts []int) int {
	if numShots <= 1 || len(txStarts) != numShots || len(a) < delta+numShots*frame {
		return 0
	}
	pc := make([]int, numShots)
	for f := 0; f < numShots; f++ {
		data := a[delta+f*frame+lead : delta+(f+1)*frame]
		bi, bv := 0, 0
		for c := 0; c < channels; c++ {
			pk := 0
			for smp := 0; smp < points; smp++ {
				v := int(int16(data[smp*channels+c]))
				if v < 0 {
					v = -v
				}
				if v > pk {
					pk = v
				}
			}
			if pk > bv {
				bv, bi = pk, c
			}
		}
		pc[f] = bi
	}
	exp := make([]int, numShots)
	for i := 0; i < numShots; i++ {
		center := int(float64(txStarts[i]) + float64(aperture-1)/2)
		// Frames are ordered by receiver index, so the expected peak CHANNEL
		// is the receiver carrying the near-normal element, not that
		// element's offset within the requested window. Under the 32:64 mux
		// an element in the upper half arrives one mux span down; this
		// degrades correctly to `center` when channels == probe elements.
		r := center
		if r > channels {
			r -= channels
		}
		exp[i] = r - 1
	}
	return bestRotation(pc, exp)
}

// bestRotation returns r minimizing sum |pc[(i+r)%n] - exp[i]|.
func bestRotation(pc, exp []int) int {
	n := len(pc)
	bestR, bestS := 0, int(^uint(0)>>1)
	for r := 0; r < n; r++ {
		s := 0
		for i := 0; i < n; i++ {
			d := pc[(i+r)%n] - exp[i]
			if d < 0 {
				d = -d
			}
			s += d
		}
		if s < bestS {
			bestS, bestR = s, r
		}
	}
	return bestR
}