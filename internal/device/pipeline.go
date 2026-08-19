package device

import (
	"context"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"echosight/internal/compress"
	"echosight/internal/state"
	"echosight/internal/uplink"
	"echosight/internal/wire"
)

// ============================================================================
// COMPRESSION + UPLINK WORKER
//
// This file occupies the seam the live-TFM imaging worker used to hold, and
// keeps the same lifecycle shape (pipeInit / pipeStart / pipeStop / pipeOffer)
// so acquire.go and acquire_fmc.go needed only a rename. Reconstruction has
// moved to the compute server; this machine now captures, compresses and
// transmits, and does nothing else with the samples.
//
// ---------------------------------------------------------------------------
// THE ONE SEMANTIC DIFFERENCE FROM THE IMAGING WORKER
// ---------------------------------------------------------------------------
//
// The imaging worker used a one-slot DROP-STALE mailbox. That was correct for
// it: a display only ever wants the newest frame, and dropping an old one
// costs nothing.
//
// This worker must NOT drop. Every cycle is inspection data that has to reach
// the record. So the mailbox is a real queue, and when it fills the loss is
// counted and shouted about rather than being business as usual.
//
// A copy of that old drop-stale pattern applied here would silently destroy
// inspection data under load and look completely healthy while doing it. If
// you are reading this because you are adding a second consumer, give it its
// own drop-stale mailbox rather than making this one lossy.
//
// ---------------------------------------------------------------------------
// CORE AFFINITY
// ---------------------------------------------------------------------------
//
// On the PICO-RAP4 the intent is: P-cores run the SI5G read loop and the
// network write; E-cores run the DSP chain. That pinning is done outside this
// process (systemd CPUAffinity / taskset on the unit file, see systemd/), not
// here, because Go gives no portable way to pin a goroutine to a core and
// pretending otherwise in code would be misleading.
//
// The DSP is ~4.3 GFLOP/s, about a quarter of one AVX2 core, so a single
// worker was expected to be sufficient. It was not — see numWorkers below.
// ============================================================================

// pipeQueueFrames bounds the cycle queue between the read loop and the
// compressors. Small on purpose: this is a handover buffer, not storage. If
// it is filling, compression is too slow and the answer is a faster codec or
// lower decimation, not a deeper queue.
const pipeQueueFrames = 32

// numWorkers is how many goroutines run the DSP chain.
//
// One was the original guess, on the estimate that the chain costs ~4.3
// GFLOP/s or about a quarter of an AVX2 core. Measurement on the PICO-RAP4
// said otherwise: 41 ms per cycle against a 14.4 ms budget at 69.4 cycles/s.
// Part of that was a division in the filter inner loop (since removed), and
// part is simply that scalar Go on an E-core is not an AVX2 core.
//
// Cycles are independent — each carries its own geometry and encoder position
// and is compressed in isolation — so this parallelises cleanly. Frames may
// then complete out of order, which is fine: Header.Seq is assigned inside
// the Compressor and the receiver sorts on it.
//
// Sized from the machine rather than hardcoded, leaving two cores for the
// SI5G read loop and the network write, which must not be starved.
var numWorkers = func() int {
	n := runtime.NumCPU() - 2
	if n < 1 {
		n = 1
	}
	if n > 8 {
		n = 8
	}
	return n
}()

type pipeState struct {
	// running is driven by acquisition start/stop, exactly as imaging was.
	// There is no operator toggle: compressing and sending is what this
	// machine does.
	running atomic.Bool
	geomGen atomic.Int64
	queue   chan pipeJob

	send *uplink.Sender

	// seq is the single frame sequence across all workers. Workers finish out
	// of order; the sequence must not.
	seq atomic.Uint64

	// cfg is the compression configuration, swappable at the bench.
	mu     sync.Mutex
	params compress.Params
	codec  compress.Codec
	codecID uint8

	// Aggregate stats across workers.
	framesIn   atomic.Uint64
	bytesIn    atomic.Uint64
	bytesOut   atomic.Uint64
	compressNs atomic.Uint64
	queueDrops atomic.Uint64

	wg     sync.WaitGroup
	cancel context.CancelFunc
}

// pipeJob is one acquisition cycle plus the position it was fired from.
type pipeJob struct {
	cycle  []uint16
	geom   compress.Geometry
	motion compress.Motion
}

// pipeInit starts the workers and the uplink. Called once from New().
func (s *Supervisor) pipeInit() {
	s.pipe.queue = make(chan pipeJob, pipeQueueFrames)
	s.pipe.params = compress.DefaultParams()
	// CodecNone is the bring-up default and is a deliberate choice, not an
	// oversight. It yields 2.67x from stages 1-3 alone, which puts the stream
	// at ~46 MB/s — comfortably inside the link budget — and it removes the
	// unbenchmarked variable from board bring-up. Switch to zstd-1 or LZ4
	// once one of them has been measured on this hardware; see entropy.go.
	s.pipe.codec = compress.CodecNone{}
	s.pipe.codecID = compress.CodecIDNone

	addr := s.opt.UplinkAddr
	if addr == "" {
		s.logf("warn", "no uplink address configured; compressed frames will "+
			"be discarded (bench mode)")
	}
	cfg := uplink.DefaultConfig(addr)
	cfg.Logf = s.logf
	s.pipe.send = uplink.NewSender(cfg)

	ctx, cancel := context.WithCancel(context.Background())
	s.pipe.cancel = cancel
	if addr != "" {
		go s.pipe.send.Run(ctx)
	}
	s.logf("info", "compression: %d workers (%d cpus, 2 reserved for io)",
		numWorkers, runtime.NumCPU())
	for i := 0; i < numWorkers; i++ {
		s.pipe.wg.Add(1)
		go s.pipeWorker(ctx, i)
	}
}

// pipeStart / pipeStop are called by the acquisition lifecycle only
// (acquire.go), mirroring what tfmStart / tfmStop did.
func (s *Supervisor) pipeStart() {
	if !s.pipe.running.Swap(true) {
		p := s.pipe.params
		s.logf("info", "compression started: D=%d %d-bit codec=%s (cutoff %.1f MHz, carrier %.1f MHz)",
			p.Decimation, p.Bits, s.pipe.codec.Name(), p.CutoffMHz, p.CarrierMHz)
	}
}

func (s *Supervisor) pipeStop() {
	if s.pipe.running.Swap(false) {
		// Drain rather than discard: these cycles are already captured and
		// still have to reach the record.
		for {
			select {
			case j := <-s.pipe.queue:
				s.pipe.queue <- j
			default:
			}
			break
		}
		st := s.pipe.send.Stats()
		s.logf("info", "compression stopped: %d frames, %.2fx, %d dropped",
			s.pipe.framesIn.Load(), s.CompressionRatio(), st.Dropped+s.pipe.queueDrops.Load())
	}
}

// pipeOffer is called by the read loop for every cycle. See the DROP note at
// the top of this file: unlike the imaging worker this queue is not
// drop-stale, and a full queue is an error condition.
//
// It still does not BLOCK, because blocking here stalls the SI5G read loop
// and a stalled reader wedges the board. The compromise is: never block,
// always count, always complain.
func (s *Supervisor) pipeOffer(cycle []uint16) {
	if !s.pipe.running.Load() {
		return
	}
	g := s.pipeGeometry()
	job := pipeJob{
		cycle:  append([]uint16(nil), cycle...),
		geom:   g,
		motion: cycleMotion(cycle, g.FrameLead),
	}
	select {
	case s.pipe.queue <- job:
	default:
		n := s.pipe.queueDrops.Add(1)
		if n == 1 || n%100 == 0 {
			s.logf("error", "compression queue full: %d cycles dropped — "+
				"DSP is not keeping up with acquisition", n)
		}
	}
}

// pipeGeometry reads the current acquisition geometry. WireFrame and
// FrameLead come from the read loop's measured values, never recomputed —
// see the comment on compress.Geometry.
func (s *Supervisor) pipeGeometry() compress.Geometry {
	fg := s.fmcGeom.Load()
	if fg == nil {
		return compress.Geometry{}
	}
	s.mu.RLock()
	cfg := s.applied
	s.mu.RUnlock()
	return compress.Geometry{
		Channels:  fg.Channels,
		Points:    fg.Points,
		NumShots:  fg.NumShots,
		Aperture:  cfg.Scan.Aperture,
		WireFrame: fg.WireFrame,
		FrameLead: fg.FrameLead,
		// FMC firmware digitizes at a fixed 50 MHz; there is no sampling
		// code to read back.
		SampleRate: 50_000_000,
	}
}

// cycleMotion pulls the device timer and encoder counts out of the first
// shot's frame header. Layout per acquire_fmc.go: 64-bit microsecond timer in
// words 0-3, then four 32-bit encoders in words 4-11.
//
// Encoders are forwarded as RAW DEVICE COUNTS. Converting to arm arc position
// happens on the compute server, deliberately: a calibration change must not
// require reflashing a robot that is inside a tank.
//
// TODO(sbc): verify with the arm actually moving that this firmware populates
// the encoder words. The FMC header comment in acquire_fmc.go documents four
// 32-bit encoders and the PA header definitely carries them, but it has not
// been confirmed under motion. Downstream motion compensation is only as good
// as this field, and zeros here would look exactly like a stationary arm.
func cycleMotion(cycle []uint16, frameLead int) compress.Motion {
	var m compress.Motion
	if frameLead < 12 || len(cycle) < 12 {
		return m
	}
	m.TimerUs = uint64(cycle[0]) | uint64(cycle[1])<<16 |
		uint64(cycle[2])<<32 | uint64(cycle[3])<<48
	for e := 0; e < 4; e++ {
		m.Encoder[e] = int32(uint32(cycle[4+e*2]) | uint32(cycle[5+e*2])<<16)
	}
	return m
}

func (s *Supervisor) pipeWorker(ctx context.Context, id int) {
	defer s.pipe.wg.Done()

	s.pipe.mu.Lock()
	params, codec, codecID := s.pipe.params, s.pipe.codec, s.pipe.codecID
	s.pipe.mu.Unlock()

	c, err := compress.New(params, codec, codecID)
	if err != nil {
		s.logf("error", "compression worker %d: %v", id, err)
		return
	}
	c.Shared = &s.pipe.seq

	// int16 view of the cycle. The device delivers uint16 words; samples are
	// signed. HF (raw RF) is mandatory here — see apply.go, which forces it —
	// because demodulation on a rectified signal is meaningless.
	var signed []int16
	var lastStat time.Time
	var lastFrames, lastIn, lastOut, lastNs uint64

	for {
		select {
		case <-ctx.Done():
			return
		case job := <-s.pipe.queue:
			if cap(signed) < len(job.cycle) {
				signed = make([]int16, len(job.cycle))
			}
			signed = signed[:len(job.cycle)]
			for i, v := range job.cycle {
				signed[i] = int16(v)
			}

			frame, err := c.Frame(signed, job.geom, job.motion)
			if err != nil {
				s.logf("error", "compression worker %d: %v", id, err)
				continue
			}
			s.pipe.send.Send(frame)

			// Deltas, not absolutes: with several workers each holding its
			// own Compressor, Store would make the totals whichever worker
			// reported last instead of the sum.
			s.pipe.framesIn.Add(c.FramesIn - lastFrames)
			s.pipe.bytesIn.Add(c.BytesIn - lastIn)
			s.pipe.bytesOut.Add(c.BytesOut - lastOut)
			s.pipe.compressNs.Add(c.CompressNs - lastNs)
			lastFrames, lastIn, lastOut, lastNs = c.FramesIn, c.BytesIn, c.BytesOut, c.CompressNs

			if time.Since(lastStat) > time.Second {
				lastStat = time.Now()
				s.publishUplink()
			}
		}
	}
}

// CompressionRatio is cumulative raw bytes over wire bytes.
func (s *Supervisor) CompressionRatio() float64 {
	out := s.pipe.bytesOut.Load()
	if out == 0 {
		return 0
	}
	return float64(s.pipe.bytesIn.Load()) / float64(out)
}

// UplinkStats reports compression and transmit health. Served by
// GET /api/uplink and shown in the TUI.
func (s *Supervisor) UplinkStats() wire.UplinkStats {
	st := s.pipe.send.Stats()
	frames := s.pipe.framesIn.Load()
	var ms float64
	if frames > 0 {
		// Per-cycle CPU time. Divide by numWorkers for the wall-clock rate,
		// since workers run concurrently: 40 ms of CPU across 4 workers is
		// 10 ms of wall clock per cycle.
		ms = float64(s.pipe.compressNs.Load()) / float64(frames) / 1e6 / float64(numWorkers)
	}
	return wire.UplinkStats{
		Connected:  st.Connected,
		Target:     st.Target,
		FramesSent: st.FramesSent,
		BytesIn:    s.pipe.bytesIn.Load(),
		BytesOut:   s.pipe.bytesOut.Load(),
		Ratio:      s.CompressionRatio(),
		MBps:       st.MBps,
		Queued:     st.Queued + len(s.pipe.queue),
		// Both drop counters roll up here. Either one being non-zero is a
		// defect; the endpoint does not distinguish because the operator
		// response is the same — stop and find out why.
		Dropped:    st.Dropped + s.pipe.queueDrops.Load(),
		CompressMs: ms,
	}
}

func (s *Supervisor) publishUplink() {
	u := s.UplinkStats()
	s.opt.Store.Update(func(sn *state.Snapshot) {
		sn.UplinkConnected = u.Connected
		sn.UplinkTarget = u.Target
		sn.UplinkMBps = u.MBps
		sn.UplinkRatio = u.Ratio
		sn.UplinkQueued = u.Queued
		sn.UplinkDropped = u.Dropped
		sn.FramesSent = u.FramesSent
	})
}

// SetCompression swaps the chain configuration. Only valid while stopped:
// changing decimation or bit depth mid-stream would produce frames the
// receiver cannot associate with a single configuration, and the per-frame
// header would be the only thing preventing silent corruption. Refusing is
// cheaper than relying on that.
func (s *Supervisor) SetCompression(p compress.Params, codec compress.Codec, id uint8) error {
	if s.pipe.running.Load() {
		return errQuiet("set compression while acquiring")
	}
	if err := p.Validate(); err != nil {
		return err
	}
	s.pipe.mu.Lock()
	s.pipe.params, s.pipe.codec, s.pipe.codecID = p, codec, id
	s.pipe.mu.Unlock()
	s.logf("info", "compression reconfigured: D=%d %d-bit codec=%s",
		p.Decimation, p.Bits, codec.Name())
	return nil
}

// pipeShutdown stops the workers and the uplink.
func (s *Supervisor) pipeShutdown() {
	if s.pipe.cancel != nil {
		s.pipe.cancel()
	}
	s.pipe.send.Close()
	s.pipe.wg.Wait()
}