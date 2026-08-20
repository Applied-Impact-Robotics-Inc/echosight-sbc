// Package state holds the process-wide snapshot every consumer reads (HTTP,
// WebSocket, TUI) and the broker that fans frames out to WebSocket clients.
//
// The broker is deliberately drop-oldest per subscriber. A slow browser must
// never apply backpressure to the acquisition loop; it gets a gap in bundleSeq
// instead, which the frontend surfaces as a counter.
package state

import (
	"encoding/json"
	"sync"
	"sync/atomic"
	"time"

	"echosight/internal/wire"
)

type Phase string

const (
	PhaseAbsent      Phase = "absent"      // library loaded, no matching board detected
	PhaseOpening     Phase = "opening"     // Open() in flight, deadline armed
	PhaseConfiguring Phase = "configuring" // reapplying config after a fresh open
	PhaseRunning     Phase = "running"     // healthy
	PhaseDegraded    Phase = "degraded"    // device-gone error code seen
	PhaseClosing     Phase = "closing"     // HV off, firing stopped, Close() pending
	PhaseHung        Phase = "hung"        // Open() blew its deadline; unrecoverable in-process
)

// Snapshot is a consistent view of everything the TUI and /api/state need.
type Snapshot struct {
	Phase        Phase
	PhaseSince   time.Time
	Detected     int    // number_of_devices, valid even before Open
	LastError    string // last transport or SI5G error text
	Reconnects   int
	LibPath      string
	StartedAt    time.Time

	Device wire.DeviceInfo
	State  wire.AppState
	Status wire.AppStatus

	// Uplink headline, for the TUI.
	LastSweepID     int
	UplinkConnected bool
	UplinkTarget    string
	UplinkMBps      float64
	UplinkRatio     float64
	UplinkQueued    int
	UplinkDropped   uint64
	FramesSent      uint64

	Clients     int
	ClientDrops uint64
}

type Store struct {
	mu   sync.RWMutex
	snap Snapshot

	logMu   sync.Mutex
	logRing []LogLine
	logSeq  uint64
	mirror  func(level, msg string)
}

// SetMirror registers a sink that receives every log line in addition to the
// ring. Used in --headless mode to copy supervisor errors to stderr, which
// otherwise only ever reach the TUI's log panel and are invisible under
// systemd. The sink must not call Logf (deadlock).
func (s *Store) SetMirror(fn func(level, msg string)) {
	s.logMu.Lock()
	s.mirror = fn
	s.logMu.Unlock()
}

type LogLine struct {
	At    time.Time
	Level string
	Msg   string
}

const logRingSize = 300

func NewStore() *Store {
	return &Store{
		snap:    Snapshot{Phase: PhaseAbsent, PhaseSince: time.Now(), StartedAt: time.Now()},
		logRing: make([]LogLine, 0, logRingSize),
	}
}

// Update mutates the snapshot under lock. Keep the closure short.
func (s *Store) Update(fn func(*Snapshot)) {
	s.mu.Lock()
	fn(&s.snap)
	s.mu.Unlock()
}

func (s *Store) Snapshot() Snapshot {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.snap
}

func (s *Store) SetPhase(p Phase) {
	s.mu.Lock()
	if s.snap.Phase != p {
		s.snap.Phase = p
		s.snap.PhaseSince = time.Now()
	}
	s.mu.Unlock()
}

func (s *Store) Logf(level, msg string) {
	s.logMu.Lock()
	s.logSeq++
	if len(s.logRing) == logRingSize {
		copy(s.logRing, s.logRing[1:])
		s.logRing = s.logRing[:logRingSize-1]
	}
	s.logRing = append(s.logRing, LogLine{At: time.Now(), Level: level, Msg: msg})
	mirror := s.mirror
	s.logMu.Unlock()
	if mirror != nil {
		mirror(level, msg)
	}
}

// Tail returns the last n log lines, oldest first.
func (s *Store) Tail(n int) []LogLine {
	s.logMu.Lock()
	defer s.logMu.Unlock()
	if n > len(s.logRing) {
		n = len(s.logRing)
	}
	out := make([]LogLine, n)
	copy(out, s.logRing[len(s.logRing)-n:])
	return out
}

// ---------------------------------------------------------------------------
// Broker
// ---------------------------------------------------------------------------

type Frame struct {
	Binary bool
	Data   []byte
}

type Sub struct {
	C       chan Frame
	dropped atomic.Uint64
}

func (s *Sub) Dropped() uint64 { return s.dropped.Load() }

type Broker struct {
	mu   sync.RWMutex
	subs map[*Sub]struct{}
}

func NewBroker() *Broker { return &Broker{subs: make(map[*Sub]struct{})} }

func (b *Broker) Subscribe(buf int) *Sub {
	s := &Sub{C: make(chan Frame, buf)}
	b.mu.Lock()
	b.subs[s] = struct{}{}
	b.mu.Unlock()
	return s
}

func (b *Broker) Unsubscribe(s *Sub) {
	b.mu.Lock()
	delete(b.subs, s)
	b.mu.Unlock()
}

func (b *Broker) Count() int {
	b.mu.RLock()
	defer b.mu.RUnlock()
	return len(b.subs)
}

func (b *Broker) TotalDropped() uint64 {
	b.mu.RLock()
	defer b.mu.RUnlock()
	var n uint64
	for s := range b.subs {
		n += s.dropped.Load()
	}
	return n
}

// PublishJSON marshals v once and fans it out as a text frame.
func (b *Broker) PublishJSON(v any) {
	p, err := json.Marshal(v)
	if err != nil {
		return
	}
	b.publish(Frame{Binary: false, Data: p})
}

// PublishBinary fans out a bundle. The caller must not mutate p afterwards;
// the acquisition loop double-buffers for exactly this reason.
func (b *Broker) PublishBinary(p []byte) { b.publish(Frame{Binary: true, Data: p}) }

func (b *Broker) publish(f Frame) {
	b.mu.RLock()
	defer b.mu.RUnlock()
	for s := range b.subs {
		select {
		case s.C <- f:
		default:
			// Drop the oldest queued frame and take the slot. Losing the
			// stalest sweep is always better than losing the newest.
			select {
			case <-s.C:
				s.dropped.Add(1)
			default:
			}
			select {
			case s.C <- f:
			default:
				s.dropped.Add(1)
			}
		}
	}
}
