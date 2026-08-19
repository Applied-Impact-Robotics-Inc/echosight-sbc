// Package pose ingests robot pose from the arm controller so each sweep can be
// stamped with where the probe actually was.
//
// This has to happen on the robot, before the data leaves. Correlating by
// arrival time on the surface would smear the C-scan by whatever the tether
// latency happens to be that second, and the error is invisible until the map
// looks subtly wrong.
package pose

import (
	"context"
	"encoding/json"
	"net"
	"sync"
	"time"

	"echosight/internal/wire"
)

// Sample is one pose report. TUs is the sender's clock in microseconds; both
// sides run on the same SBC so the clocks agree.
type Sample struct {
	TUs uint64  `json:"tUs"`
	X   float32 `json:"x"`
	Y   float32 `json:"y"`
	Z   float32 `json:"z"`
	QW  float32 `json:"qw"`
	QX  float32 `json:"qx"`
	QY  float32 `json:"qy"`
	QZ  float32 `json:"qz"`
}

const ringSize = 512

// StaleAfterUs bounds how far a sweep timestamp may sit past the newest sample
// before the pose is reported invalid rather than extrapolated.
const StaleAfterUs = 100_000 // 100 ms

type Source struct {
	addr string
	conn *net.UDPConn

	mu   sync.RWMutex
	ring [ringSize]Sample
	n    int
	head int
}

func NewUDP(addr string) (*Source, error) {
	ua, err := net.ResolveUDPAddr("udp", addr)
	if err != nil {
		return nil, err
	}
	c, err := net.ListenUDP("udp", ua)
	if err != nil {
		return nil, err
	}
	return &Source{addr: addr, conn: c}, nil
}

func (s *Source) Addr() string { return s.addr }

func (s *Source) Run(ctx context.Context) {
	go func() {
		<-ctx.Done()
		_ = s.conn.Close()
	}()
	buf := make([]byte, 512)
	for {
		n, _, err := s.conn.ReadFromUDP(buf)
		if err != nil {
			select {
			case <-ctx.Done():
				return
			default:
			}
			time.Sleep(50 * time.Millisecond)
			continue
		}
		var smp Sample
		if json.Unmarshal(buf[:n], &smp) != nil {
			continue
		}
		if smp.TUs == 0 {
			smp.TUs = uint64(time.Now().UnixMicro())
		}
		s.push(smp)
	}
}

func (s *Source) push(smp Sample) {
	s.mu.Lock()
	s.ring[s.head] = smp
	s.head = (s.head + 1) % ringSize
	if s.n < ringSize {
		s.n++
	}
	s.mu.Unlock()
}

// At returns the pose interpolated to tUs, the age of the newest sample in
// milliseconds, and whether the result is trustworthy.
func (s *Source) At(tUs uint64) (wire.Pose, int64, bool) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	if s.n == 0 {
		return wire.Pose{}, 0, false
	}

	newest := s.ring[(s.head-1+ringSize)%ringSize]
	age := (int64(time.Now().UnixMicro()) - int64(newest.TUs)) / 1000

	// Walk backwards for the first sample at or before tUs.
	var before, after Sample
	haveBefore, haveAfter := false, false
	after = newest
	haveAfter = newest.TUs >= tUs
	for i := 1; i <= s.n; i++ {
		smp := s.ring[(s.head-i+ringSize)%ringSize]
		if smp.TUs <= tUs {
			before = smp
			haveBefore = true
			break
		}
		after = smp
		haveAfter = true
	}

	switch {
	case haveBefore && haveAfter && after.TUs > before.TUs:
		f := float32(float64(tUs-before.TUs) / float64(after.TUs-before.TUs))
		return wire.Pose{
			PosMm: [3]float32{
				lerp(before.X, after.X, f),
				lerp(before.Y, after.Y, f),
				lerp(before.Z, after.Z, f),
			},
			// Nearest-neighbour on orientation: a proper slerp is not worth it
			// at these rates, and a normalised lerp of nearly identical
			// quaternions is indistinguishable anyway.
			Quat:  [4]float32{after.QW, after.QX, after.QY, after.QZ},
			Valid: true,
		}, age, true

	case haveBefore:
		// The sweep is newer than every sample. Hold the last pose, but only
		// while it is fresh enough to mean anything.
		ok := int64(tUs)-int64(before.TUs) < StaleAfterUs
		return wire.Pose{
			PosMm: [3]float32{before.X, before.Y, before.Z},
			Quat:  [4]float32{before.QW, before.QX, before.QY, before.QZ},
			Valid: ok,
		}, age, ok

	default:
		return wire.Pose{}, age, false
	}
}

func lerp(a, b, f float32) float32 { return a + (b-a)*f }
