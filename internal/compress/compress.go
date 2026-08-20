package compress

import (
	"encoding/binary"
	"hash/crc32"
	"math"
	"sync/atomic"
	"time"
)

// ============================================================================
// FRAME ENVELOPE - the SBC to compute-server contract
//
// This is the wire format for everything crossing the fibre umbilical. It is
// versioned because the two ends will be deployed independently and a silent
// mismatch here corrupts a whole scan.
//
// Every frame carries the ENCODER POSITION at the moment its shots were
// fired. This is not optional metadata: TFM on the compute server does
// motion compensation with it, placing each shot where it was actually fired
// rather than at a frame average. The arm moves during the 128-firing
// sequence and without per-shot position the coherent sum smears.
// ============================================================================

// MagicV1 identifies the envelope. Bytes chosen to be recognisable in a hex
// dump when someone is staring at a truncated TCP stream at 2am.
const MagicV1 uint32 = 0x45534831 // "ESH1"

// FormatVersion increments on any layout change. The receiver must reject
// frames it does not recognise rather than guessing.
//
// v2: the payload now begins with a per-trace scale table.
//
// v1 stored ONE QScale in the header while quantising each trace against its
// own peak. Every trace therefore decoded to full scale and the true
// amplitude of all but the last was unrecoverable. Harmless for looking at a
// single A-scan's shape; fatal for TFM, where delay-and-sum combines channels
// coherently and the relative amplitude BETWEEN channels is the image. A
// strong near-tx channel and a weak far one decoded identically.
//
// Found by decoding a real capture: all 1024 traces came back at exactly
// ±2047.
const FormatVersion uint16 = 2

// HeaderBytes is the fixed-size header preceding the payload.
const HeaderBytes = 96

// Header is the fixed part of the envelope. Little-endian on the wire; both
// ends are x86 so this costs nothing.
//
// Field order is deliberate: everything needed to route, sequence and verify
// a frame comes before everything needed to reconstruct it, so a receiver can
// validate and enqueue without parsing acquisition geometry.
type Header struct {
	Magic   uint32 // MagicV1
	Version uint16
	Flags   uint16 // reserved; 0

	// --- routing and sequencing ---
	Seq       uint64 // monotonic from acquisition start; gaps mean loss
	TimerUs   uint64 // device 64-bit microsecond timer from the frame header
	WallNanos int64  // SBC wall clock at capture, for coarse correlation only

	// --- motion, per shot ---
	// Encoder values as read from the FMC frame header. Four channels, raw
	// device counts. The compute server converts to arm arc position using
	// the calibration it holds; the SBC deliberately does not, so a
	// calibration change does not require reflashing the robot.
	Encoder [4]int32

	// --- acquisition geometry ---
	// Repeated in every frame rather than sent once at stream start. It is 20
	// bytes against a ~50 KB payload, and it means a receiver that joins mid
	// stream, or a file replayed out of context, is self-describing. A stream
	// where geometry is only in a preamble is one dropped preamble away from
	// unreconstructable.
	Channels   uint16
	Points     uint16
	NumShots   uint16
	Aperture   uint16
	SampleRate uint32 // Hz

	// --- compression parameters ---
	// Also per-frame, for the same reason, and because parameters can be
	// changed live at the bench.
	Decimation uint8
	Bits       uint8
	CodecID    uint8
	_          uint8
	CarrierHz  uint32
	CutoffHz   uint32
	// QScale is the LARGEST per-trace scale in the frame, kept for quick
	// "how hot was this frame" checks and for logs. It is NOT sufficient to
	// reconstruct: use the per-trace table at the head of the payload.
	QScale     float64

	// --- integrity ---
	RawBytes   uint32 // payload size before stage 4
	WireBytes  uint32 // payload size as transmitted
	PayloadCRC uint32 // CRC32-C over the transmitted payload
	HeaderCRC  uint32 // CRC32-C over bytes [0, 92)
}

// Codec IDs. These are on the wire, so values must never be reused.
const (
	CodecIDNone uint8 = 0
	CodecIDZlib uint8 = 1
	CodecIDZstd uint8 = 2
	CodecIDLZ4  uint8 = 3
)

var castagnoli = crc32.MakeTable(crc32.Castagnoli)

// MarshalHeader writes h into a 96-byte buffer, filling HeaderCRC.
func MarshalHeader(h *Header, dst []byte) []byte {
	dst = growB(dst, HeaderBytes)
	le := binary.LittleEndian
	le.PutUint32(dst[0:], h.Magic)
	le.PutUint16(dst[4:], h.Version)
	le.PutUint16(dst[6:], h.Flags)
	le.PutUint64(dst[8:], h.Seq)
	le.PutUint64(dst[16:], h.TimerUs)
	le.PutUint64(dst[24:], uint64(h.WallNanos))
	for i := 0; i < 4; i++ {
		le.PutUint32(dst[32+i*4:], uint32(h.Encoder[i]))
	}
	le.PutUint16(dst[48:], h.Channels)
	le.PutUint16(dst[50:], h.Points)
	le.PutUint16(dst[52:], h.NumShots)
	le.PutUint16(dst[54:], h.Aperture)
	le.PutUint32(dst[56:], h.SampleRate)
	dst[60] = h.Decimation
	dst[61] = h.Bits
	dst[62] = h.CodecID
	dst[63] = 0
	le.PutUint32(dst[64:], h.CarrierHz)
	le.PutUint32(dst[68:], h.CutoffHz)
	le.PutUint64(dst[72:], f64bits(h.QScale))
	le.PutUint32(dst[80:], h.RawBytes)
	le.PutUint32(dst[84:], h.WireBytes)
	le.PutUint32(dst[88:], h.PayloadCRC)
	h.HeaderCRC = crc32.Checksum(dst[:92], castagnoli)
	le.PutUint32(dst[92:], h.HeaderCRC)
	return dst
}

// UnmarshalHeader parses and validates a header. It rejects rather than
// repairs: a header that fails CRC means the stream is desynchronised, and
// continuing from a guessed offset produces plausible-looking garbage.
func UnmarshalHeader(src []byte) (Header, error) {
	var h Header
	if len(src) < HeaderBytes {
		return h, errf("short header: %d bytes, need %d", len(src), HeaderBytes)
	}
	le := binary.LittleEndian
	if got := le.Uint32(src[0:]); got != MagicV1 {
		return h, errf("bad magic %#08x, expected %#08x", got, MagicV1)
	}
	if want := crc32.Checksum(src[:92], castagnoli); want != le.Uint32(src[92:]) {
		return h, errf("header CRC mismatch; stream is desynchronised")
	}
	h.Magic = le.Uint32(src[0:])
	h.Version = le.Uint16(src[4:])
	if h.Version != FormatVersion {
		return h, errf("unsupported envelope version %d (this build speaks %d)",
			h.Version, FormatVersion)
	}
	h.Flags = le.Uint16(src[6:])
	h.Seq = le.Uint64(src[8:])
	h.TimerUs = le.Uint64(src[16:])
	h.WallNanos = int64(le.Uint64(src[24:]))
	for i := 0; i < 4; i++ {
		h.Encoder[i] = int32(le.Uint32(src[32+i*4:]))
	}
	h.Channels = le.Uint16(src[48:])
	h.Points = le.Uint16(src[50:])
	h.NumShots = le.Uint16(src[52:])
	h.Aperture = le.Uint16(src[54:])
	h.SampleRate = le.Uint32(src[56:])
	h.Decimation = src[60]
	h.Bits = src[61]
	h.CodecID = src[62]
	h.CarrierHz = le.Uint32(src[64:])
	h.CutoffHz = le.Uint32(src[68:])
	h.QScale = bitsf64(le.Uint64(src[72:]))
	h.RawBytes = le.Uint32(src[80:])
	h.WireBytes = le.Uint32(src[84:])
	h.PayloadCRC = le.Uint32(src[88:])
	h.HeaderCRC = le.Uint32(src[92:])
	return h, nil
}

// ---------------------------------------------------------------------------
// Compressor
// ---------------------------------------------------------------------------

// Geometry describes the acquisition the samples came from. Supplied by the
// device layer; the compressor copies it into every frame header.
type Geometry struct {
	Channels int
	Points   int
	NumShots int
	Aperture int
	// SampleRate in Hz. Fixed at 50 MHz by the FMC firmware.
	SampleRate int
	// WireFrame is the per-shot stride in 16-bit words as delivered by the
	// read loop, and FrameLead is the device header inside it. A cycle is
	// NumShots frames of WireFrame words; samples start at FrameLead and are
	// interleaved [sample][channel].
	//
	// These are not derivable from Channels and Points. The read loop
	// measures the true stride by autocorrelation because settled record
	// lengths and header variants have disagreed with the device's own
	// frame_size readback in the field, and a constant stride error shows up
	// as A-scans drifting in time and tx rows rotating through the matrix.
	// Always take these from fmcGeometry, never recompute them.
	WireFrame int
	FrameLead int
}

// Motion is the per-cycle position information.
type Motion struct {
	TimerUs uint64
	Encoder [4]int32
}

// TraceScales is the per-trace scale table that prefixes every v2 payload:
// float32, little-endian, one per trace, in the same [shot][channel] order as
// the packed codes that follow.
//
// float32 rather than float64 because the scales span a handful of orders of
// magnitude and 24 bits of mantissa is far more than the 12-bit codes can
// use. 1024 traces costs 4 KB against a ~412 KB payload — under 1%, to
// recover information that cannot be reconstructed any other way.
//
// Per-trace scaling is kept rather than switching to one scale per frame,
// because FMC channel amplitudes span a wide range within a single shot and
// a single frame-wide scale would leave the weakest channels with only a few
// usable bits. Store the scales; do not throw away the headroom.
const TraceScaleBytes = 4

// Compressor runs the full chain for one worker. It is NOT safe for
// concurrent use: each worker holds its own, so the scratch buffers below can
// be reused without allocation. Sustained 122 MB/s through a garbage-
// collected runtime is exactly the workload where per-frame allocation shows
// up as GC pauses in the acquisition loop.
type Compressor struct {
	k     *Kernel
	codec Codec
	id    uint8

	bb      Baseband
	q       Quantised
	trace   []int16
	scratch []byte
	scales  []float32
	packed  []byte
	wire   []byte
	hdr    []byte
	out    []byte

	// seq is local when Shared is nil. With more than one worker it MUST be
	// shared, or every worker emits 1,2,3... and the receiver sees
	// duplicates instead of a monotonic stream — which silently defeats gap
	// detection, the only mechanism that reveals dropped frames.
	seq    uint64
	Shared *atomic.Uint64

	// Stats, read by the pipeline for the TUI and the uplink endpoint.
	FramesIn   uint64
	BytesIn    uint64
	BytesOut   uint64
	CompressNs uint64
}

// New builds a Compressor. codecID must match codec; it travels on the wire.
func New(p Params, codec Codec, codecID uint8) (*Compressor, error) {
	k, err := NewKernel(p)
	if err != nil {
		return nil, err
	}
	return &Compressor{k: k, codec: codec, id: codecID}, nil
}

// Params returns the active configuration.
func (c *Compressor) Params() Params { return c.k.P }

// Frame compresses one acquisition cycle.
//
// cycle is the raw interleaved int16 sample block as the read loop delivers
// it: [sample][channel], channel varying fastest, with the device frame
// header already stripped. The returned slice is owned by the Compressor and
// is valid only until the next call to Frame — the uplink must copy it into
// the ring, which it does.
//
// Ordering note: traces are de-interleaved to channel-major before the DSP
// runs. Working on the interleaved layout would stride the cache by
// channels*2 bytes on every tap of a 65-tap filter, which is the difference
// between this stage costing a quarter of a core and costing several.
func (c *Compressor) Frame(cycle []int16, g Geometry, m Motion) ([]byte, error) {
	start := time.Now()
	ch, pts, shots := g.Channels, g.Points, g.NumShots
	if ch <= 0 || pts <= 0 || shots <= 0 {
		return nil, errf("bad geometry: %d channels, %d points, %d shots", ch, pts, shots)
	}
	if g.WireFrame < g.FrameLead+ch*pts {
		return nil, errf("wireFrame %d cannot hold %d lead + %d*%d samples",
			g.WireFrame, g.FrameLead, ch, pts)
	}
	if len(cycle) < shots*g.WireFrame {
		return nil, errf("short cycle: %d words, need %d", len(cycle), shots*g.WireFrame)
	}

	c.packed = c.packed[:0]
	c.trace = growI16(c.trace, pts)
	nTraces := shots * ch
	if cap(c.scales) < nTraces {
		c.scales = make([]float32, nTraces)
	}
	c.scales = c.scales[:0]

	// De-interleave to channel-major before the DSP runs. Filtering in place
	// on the interleaved layout would stride the cache by channels*2 bytes on
	// every one of 65 taps, which is the difference between this stage
	// costing a quarter of a core and costing several.
	for sh := 0; sh < shots; sh++ {
		frame := cycle[sh*g.WireFrame : (sh+1)*g.WireFrame]
		data := frame[g.FrameLead:]
		for cIdx := 0; cIdx < ch; cIdx++ {
			for smp := 0; smp < pts; smp++ {
				c.trace[smp] = data[smp*ch+cIdx]
			}
			c.k.Demodulate(c.trace, &c.bb)
			c.k.Quantise(&c.bb, &c.q)
			// Record THIS trace's scale. Without it the trace decodes to
			// full scale and its true amplitude is gone.
			c.scales = append(c.scales, float32(c.q.Scale))
			c.scratch = PackBits(&c.q, c.scratch)
			c.packed = append(c.packed, c.scratch...)
		}
	}

	// Prefix the scale table. Done here rather than in PackBits so the
	// packing stays a pure function of one trace.
	tbl := make([]byte, len(c.scales)*TraceScaleBytes)
	var maxScale float64
	for i, sc := range c.scales {
		binary.LittleEndian.PutUint32(tbl[i*4:], math.Float32bits(sc))
		if float64(sc) > maxScale {
			maxScale = float64(sc)
		}
	}
	c.packed = append(tbl, c.packed...)

	rawBytes := len(c.packed)
	c.wire = c.wire[:0]
	var err error
	c.wire, err = c.codec.Compress(c.wire, c.packed)
	if err != nil {
		return nil, errf("stage 4: %w", err)
	}

	var seq uint64
	if c.Shared != nil {
		seq = c.Shared.Add(1)
	} else {
		c.seq++
		seq = c.seq
	}
	h := Header{
		Magic:      MagicV1,
		Version:    FormatVersion,
		Seq:        seq,
		TimerUs:    m.TimerUs,
		WallNanos:  time.Now().UnixNano(),
		Encoder:    m.Encoder,
		Channels:   uint16(ch),
		Points:     uint16(pts),
		NumShots:   uint16(g.NumShots),
		Aperture:   uint16(g.Aperture),
		SampleRate: uint32(g.SampleRate),
		Decimation: uint8(c.k.P.Decimation),
		Bits:       uint8(c.k.P.Bits),
		CodecID:    c.id,
		CarrierHz:  uint32(c.k.P.CarrierMHz * 1e6),
		CutoffHz:   uint32(c.k.P.CutoffMHz * 1e6),
		QScale:     maxScale,
		RawBytes:   uint32(rawBytes),
		WireBytes:  uint32(len(c.wire)),
		PayloadCRC: crc32.Checksum(c.wire, castagnoli),
	}
	c.hdr = MarshalHeader(&h, c.hdr)

	c.out = growB(c.out, HeaderBytes+len(c.wire))[:0]
	c.out = append(c.out, c.hdr[:HeaderBytes]...)
	c.out = append(c.out, c.wire...)

	c.FramesIn++
	c.BytesIn += uint64(len(cycle) * 2)
	c.BytesOut += uint64(len(c.out))
	c.CompressNs += uint64(time.Since(start).Nanoseconds())
	return c.out, nil
}

// Ratio is cumulative input bytes over output bytes.
func (c *Compressor) Ratio() float64 {
	if c.BytesOut == 0 {
		return 0
	}
	return float64(c.BytesIn) / float64(c.BytesOut)
}

// ---------------------------------------------------------------------------
// buffer helpers - grow in place, never shrink, never allocate on the steady
// state path.
// ---------------------------------------------------------------------------

func grow(b []float64, n int) []float64 {
	if cap(b) >= n {
		return b[:n]
	}
	return make([]float64, n)
}

func growI32(b []int32, n int) []int32 {
	if cap(b) >= n {
		return b[:n]
	}
	return make([]int32, n)
}

func growI16(b []int16, n int) []int16 {
	if cap(b) >= n {
		return b[:n]
	}
	return make([]int16, n)
}

func growB(b []byte, n int) []byte {
	if cap(b) >= n {
		return b[:n]
	}
	return make([]byte, n)
}

func f64bits(f float64) uint64 { return math.Float64bits(f) }
func bitsf64(u uint64) float64 { return math.Float64frombits(u) }