package uplink

import (
	"context"
	"encoding/binary"
	"fmt"
	"net"
	"sync"
	"sync/atomic"
	"time"
)

// ============================================================================
// UPLINK - SBC to compute server over the 1 GbE fibre umbilical
//
// LAN only. This link never crosses the internet, so there is no TLS, no
// auth, and no retry-with-backoff-forever semantics. It is a cable.
//
// BUDGET
//
//	compressed stream : 22-35 MB/s   (D=6, 12-bit, entropy coded)
//	usable on 1 GbE   : 70-80 MB/s sustained
//	utilisation       : ~25-30%
//
// There is real headroom here, which is why the design below spends bandwidth
// freely on per-frame headers and CRCs rather than economising.
//
// ---------------------------------------------------------------------------
// TCP vs UDP - the open decision
// ---------------------------------------------------------------------------
//
// This implements TCP. That is a defensible default rather than a settled
// answer, and the choice should be made deliberately before the robot goes in
// a tank. The argument each way:
//
//	TCP  - the kernel handles reassembly, ordering and retransmission. On a
//	       dedicated LAN with 25% utilisation there is essentially nothing to
//	       retransmit, so the classic objection (head-of-line blocking under
//	       loss) barely applies. Cost is that a stall anywhere downstream
//	       propagates back into the send buffer and then into this ring, so
//	       backpressure has to be handled explicitly. It is.
//
//	UDP  - no head-of-line blocking, and a lost frame is simply a lost frame
//	       rather than a stalled stream. Cost is that we own reassembly,
//	       ordering, gap detection and any retransmission we decide we want,
//	       and every FMC frame is far larger than an MTU so fragmentation is
//	       ours to manage.
//
// The deciding question is what SHOULD happen when the compute server cannot
// keep up. Under TCP the answer is "the SBC blocks, the ring fills, and we
// find out loudly". Under UDP it is "frames vanish silently". For an
// inspection record that has to be defensible, blocking loudly is the better
// failure. That is the actual reason for the default, not throughput.
// ============================================================================

// Config configures the sender.
type Config struct {
	// Addr is host:port on the compute server.
	Addr string
	// RingFrames bounds the in-memory queue. At ~50 KB per compressed frame
	// and 78 fps, 512 frames is roughly 25 MB and 6.5 seconds of slack. That
	// is deliberately modest: the big write-back buffer lives on the compute
	// server's 512 GB, not on a 12 W SBC with limited RAM. The SBC's job is
	// to notice trouble, not to absorb it.
	RingFrames int
	// WriteTimeout bounds a single frame write. Exceeded means the far end
	// or the link is in trouble; the connection is dropped and rebuilt
	// rather than hanging the sender forever.
	WriteTimeout time.Duration
	// DialTimeout bounds connection attempts.
	DialTimeout time.Duration
	// ReconnectMin/Max bound the backoff between attempts.
	ReconnectMin, ReconnectMax time.Duration
	// Logf receives operational messages.
	Logf func(level, format string, a ...any)
}

// DefaultConfig returns sensible values for the umbilical.
func DefaultConfig(addr string) Config {
	return Config{
		Addr:         addr,
		RingFrames:   512,
		WriteTimeout: 2 * time.Second,
		DialTimeout:  5 * time.Second,
		ReconnectMin: 500 * time.Millisecond,
		ReconnectMax: 5 * time.Second,
		Logf:         func(string, string, ...any) {},
	}
}

// Stats is a snapshot of sender health.
type Stats struct {
	Connected  bool
	Target     string
	FramesSent uint64
	BytesSent  uint64
	Queued     int
	Dropped    uint64
	MBps       float64
	Reconnects uint64
	LastError  string
}

// Sender owns one TCP connection and a bounded ring of frames waiting to go
// out.
type Sender struct {
	cfg Config

	// discard is set when Addr is empty (bench mode). Frames are counted and
	// thrown away at Send rather than queued, because with no Run goroutine
	// there is nothing to drain the ring: it would fill in a few seconds and
	// every subsequent frame would register as a drop. Bench mode must look
	// healthy, since its whole purpose is timing the DSP without a far end.
	discard bool

	ring    chan []byte
	pool    sync.Pool
	closing atomic.Bool

	connected  atomic.Bool
	framesSent atomic.Uint64
	bytesSent  atomic.Uint64
	dropped    atomic.Uint64
	reconnects atomic.Uint64

	mu        sync.Mutex
	lastErr   string
	rateBytes uint64
	rateAt    time.Time
	rateMBps  float64
}

// NewSender builds a sender. Call Run to start it.
func NewSender(cfg Config) *Sender {
	if cfg.RingFrames <= 0 {
		cfg.RingFrames = 512
	}
	if cfg.Logf == nil {
		cfg.Logf = func(string, string, ...any) {}
	}
	return &Sender{
		cfg:     cfg,
		discard: cfg.Addr == "",
		ring:    make(chan []byte, cfg.RingFrames),
		rateAt:  time.Now(),
	}
}

// Send enqueues one compressed frame. The slice is copied, so the caller may
// reuse its buffer immediately.
//
// BACKPRESSURE POLICY - this is the important part of the file.
//
// Send drops the frame and increments Dropped if the ring is full, and it
// does so rather than blocking BECAUSE blocking here would stall the SI5G
// read loop. That loop cannot stall: it is reading a device that keeps firing
// regardless, and a stalled reader turns into short reads, then into the
// ReadPipe2 failure modes documented at the top of acquire_fmc.go, then into
// a wedged board that needs a reset.
//
// So the loss is deliberate and bounded, but it is still LOSS, and it is
// unrecoverable and undetectable downstream except through the Seq gap. Any
// non-zero Dropped count is a defect, not a tuning opportunity. If it happens
// in the field the fix is upstream: a faster codec, a lower decimation, or
// finding out why the compute server stopped reading. It is never "raise
// RingFrames until the number goes away".
func (s *Sender) Send(frame []byte) {
	if s.closing.Load() {
		return
	}
	if s.discard {
		// Bench mode: account for the frame so throughput and ratio are still
		// measured, then drop it on the floor. Not a drop in the error sense.
		n := len(frame) + 4
		s.framesSent.Add(1)
		s.bytesSent.Add(uint64(n))
		s.tickRate(uint64(n))
		return
	}
	buf := s.get(len(frame))
	copy(buf, frame)
	select {
	case s.ring <- buf:
	default:
		s.put(buf)
		n := s.dropped.Add(1)
		if n == 1 || n%100 == 0 {
			s.cfg.Logf("error", "uplink ring full: %d frames dropped — "+
				"this is data loss, not backpressure", n)
		}
	}
}

// Run connects, sends, and reconnects until ctx is cancelled.
func (s *Sender) Run(ctx context.Context) {
	backoff := s.cfg.ReconnectMin
	for ctx.Err() == nil {
		conn, err := s.dial(ctx)
		if err != nil {
			s.setErr(err)
			s.cfg.Logf("warn", "uplink dial %s: %v (retry in %s)",
				s.cfg.Addr, err, backoff)
			select {
			case <-ctx.Done():
				return
			case <-time.After(backoff):
			}
			backoff = min(backoff*2, s.cfg.ReconnectMax)
			continue
		}
		backoff = s.cfg.ReconnectMin
		s.connected.Store(true)
		s.cfg.Logf("info", "uplink connected to %s", s.cfg.Addr)

		err = s.pump(ctx, conn)
		conn.Close()
		s.connected.Store(false)
		if ctx.Err() != nil {
			return
		}
		s.reconnects.Add(1)
		s.setErr(err)
		s.cfg.Logf("warn", "uplink lost: %v", err)
	}
}

func (s *Sender) dial(ctx context.Context) (net.Conn, error) {
	d := net.Dialer{Timeout: s.cfg.DialTimeout}
	conn, err := d.DialContext(ctx, "tcp", s.cfg.Addr)
	if err != nil {
		return nil, err
	}
	if tc, ok := conn.(*net.TCPConn); ok {
		// Nagle would coalesce our already-large frames and add latency for
		// no benefit; every write here is tens of kilobytes.
		_ = tc.SetNoDelay(true)
		// A generous send buffer absorbs brief scheduling hiccups on the far
		// end before they reach our ring.
		_ = tc.SetWriteBuffer(4 << 20)
	}
	return conn, nil
}

func (s *Sender) pump(ctx context.Context, conn net.Conn) error {
	var lenBuf [4]byte
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case frame := <-s.ring:
			// Length prefix so the receiver can frame without parsing the
			// envelope header. The envelope has its own magic and CRC; this
			// is purely a transport concern and stays separate on purpose.
			binary.LittleEndian.PutUint32(lenBuf[:], uint32(len(frame)))
			if err := conn.SetWriteDeadline(time.Now().Add(s.cfg.WriteTimeout)); err != nil {
				s.put(frame)
				return err
			}
			if _, err := conn.Write(lenBuf[:]); err != nil {
				s.put(frame)
				return err
			}
			if _, err := conn.Write(frame); err != nil {
				s.put(frame)
				return err
			}
			n := len(frame) + 4
			s.framesSent.Add(1)
			s.bytesSent.Add(uint64(n))
			s.tickRate(uint64(n))
			s.put(frame)
		}
	}
}

// tickRate maintains a 1-second windowed throughput figure.
func (s *Sender) tickRate(n uint64) {
	s.mu.Lock()
	s.rateBytes += n
	if d := time.Since(s.rateAt); d >= time.Second {
		s.rateMBps = float64(s.rateBytes) / d.Seconds() / (1 << 20)
		s.rateBytes = 0
		s.rateAt = time.Now()
	}
	s.mu.Unlock()
}

func (s *Sender) setErr(err error) {
	s.mu.Lock()
	if err != nil {
		s.lastErr = err.Error()
	}
	s.mu.Unlock()
}

// Stats snapshots sender health.
func (s *Sender) Stats() Stats {
	s.mu.Lock()
	mbps, lastErr := s.rateMBps, s.lastErr
	s.mu.Unlock()
	return Stats{
		// Bench mode reports connected so the TUI and the endpoint do not
		// show a fault that does not exist. Target is empty, which is how a
		// reader tells the two apart.
		Connected:  s.connected.Load() || s.discard,
		Target:     s.cfg.Addr,
		FramesSent: s.framesSent.Load(),
		BytesSent:  s.bytesSent.Load(),
		Queued:     len(s.ring),
		Dropped:    s.dropped.Load(),
		MBps:       mbps,
		Reconnects: s.reconnects.Load(),
		LastError:  lastErr,
	}
}

// Close stops accepting frames.
func (s *Sender) Close() { s.closing.Store(true) }

func (s *Sender) get(n int) []byte {
	if v := s.pool.Get(); v != nil {
		b := v.([]byte)
		if cap(b) >= n {
			return b[:n]
		}
	}
	return make([]byte, n)
}

func (s *Sender) put(b []byte) { s.pool.Put(b[:cap(b)]) } //nolint:staticcheck

func min(a, b time.Duration) time.Duration {
	if a < b {
		return a
	}
	return b
}

var _ = fmt.Sprintf