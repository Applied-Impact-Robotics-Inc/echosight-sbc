package wire

import (
	"encoding/binary"
)

/*
Binary A-scan bundle, little-endian. One bundle per firing cycle (sweep).

Header, 32 bytes:

	u8   msgType      0x03 A-scan
	u8   group
	u16  count        number of A-scan records
	u32  bundleSeq    monotonic per WS connection; gaps mean the backend dropped
	u64  hostTimeUs   backend wall clock at the first firing of the sweep
	i32  encoders[4]  from the SI5G frame header of the first firing

Record header, 16 bytes, then nPoints i16 samples:

	u16 firingIndex
	u16 spatialIndex
	u64 timerUs       board timer, 1 us resolution, from the frame header
	u16 nPoints
	u8  kind          0 HF raw, 1 envelope
	u8  pad

Every offset stays 2-byte aligned so the browser can take a zero-copy
Int16Array view over the sample block.

---------------------------------------------------------------------------
POSE IS GONE, AND THE MESSAGE TYPE MOVED BECAUSE OF IT
---------------------------------------------------------------------------

The header used to be 64 bytes and carried an interpolated robot pose: 12
bytes of position, 16 of orientation, a validity byte and 3 of padding. It was
left over from when this machine did reconstruction and needed to place an
image.

It does not any more. Reconstruction moved to the compute server, which owns
the arm and therefore owns the pose, and stamping it here would have meant a
second pose path measured against a second clock — a discrepancy that would
surface eventually as a placement error nobody could source.

msgType moved 0x01 -> 0x03 in the same change. That is the whole point of
touching it: a decoder written for the 64-byte layout reading a 32-byte one
does not fail, it silently reads the first record's header as pose and every
sample offset is then 32 bytes wrong. Producing garbage that parses is the
failure mode this system keeps having to design against, so the type byte
makes a stale consumer stop instead.

0x01 is the retired 64-byte pose-bearing layout and must never be reused.
0x02 stays reserved for FMC.
*/
const (
	BundleHeaderBytes = 32
	RecordHeaderBytes = 16

	// MsgTypeAscan is the current layout.
	MsgTypeAscan = 0x03
	// MsgTypeAscanLegacyPose is the retired 64-byte header. Declared so the
	// value stays claimed and a reader can name what it is refusing.
	MsgTypeAscanLegacyPose = 0x01
)

// AscanRecord references sample data owned by the caller. Encode copies it.
type AscanRecord struct {
	FiringIndex  uint16
	SpatialIndex uint16
	TimerUs      uint64
	Kind         uint8
	// Samples are raw device words reinterpreted as signed amplitude,
	// percent full scale * 327.67. Passed through untouched.
	Samples []uint16
}

// BundleMeta is the per-sweep header content.
type BundleMeta struct {
	Group      uint8
	BundleSeq  uint32
	HostTimeUs uint64
	// Encoders are RAW DEVICE COUNTS, as read from the frame header. Nothing
	// converts them here: turning counts into arm arc position is a
	// calibration, and a calibration change must never require reflashing a
	// robot that is inside a tank.
	Encoders [4]int32
}

// BundleSize reports the exact byte length EncodeBundle will produce, so the
// caller can reuse a right-sized buffer instead of growing one per sweep.
func BundleSize(recs []AscanRecord) int {
	n := BundleHeaderBytes
	for i := range recs {
		n += RecordHeaderBytes + 2*len(recs[i].Samples)
	}
	return n
}

// EncodeBundle serialises a sweep into dst, growing it only if needed, and
// returns the filled slice. Pass the previous bundle's slice to avoid churn.
func EncodeBundle(dst []byte, m BundleMeta, recs []AscanRecord) []byte {
	need := BundleSize(recs)
	if cap(dst) < need {
		dst = make([]byte, need)
	}
	b := dst[:need]

	b[0] = MsgTypeAscan
	b[1] = m.Group
	binary.LittleEndian.PutUint16(b[2:], uint16(len(recs)))
	binary.LittleEndian.PutUint32(b[4:], m.BundleSeq)
	binary.LittleEndian.PutUint64(b[8:], m.HostTimeUs)
	for i := 0; i < 4; i++ {
		binary.LittleEndian.PutUint32(b[16+4*i:], uint32(m.Encoders[i]))
	}

	off := BundleHeaderBytes
	for i := range recs {
		r := &recs[i]
		binary.LittleEndian.PutUint16(b[off:], r.FiringIndex)
		binary.LittleEndian.PutUint16(b[off+2:], r.SpatialIndex)
		binary.LittleEndian.PutUint64(b[off+4:], r.TimerUs)
		binary.LittleEndian.PutUint16(b[off+12:], uint16(len(r.Samples)))
		b[off+14] = r.Kind
		b[off+15] = 0
		off += RecordHeaderBytes
		for _, w := range r.Samples {
			binary.LittleEndian.PutUint16(b[off:], w)
			off += 2
		}
	}
	return b
}