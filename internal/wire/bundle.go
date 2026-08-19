package wire

import (
	"encoding/binary"
	"math"
)

/*
Binary A-scan bundle, little-endian. One bundle per firing cycle (sweep).

Header, 64 bytes:

	u8   msgType      0x01 A-scan, 0x02 reserved for FMC
	u8   group
	u16  count        number of A-scan records
	u32  bundleSeq    monotonic per WS connection; gaps mean the backend dropped
	u64  hostTimeUs   backend wall clock at the first firing of the sweep
	i32  encoders[4]  from the SI5G frame header of the first firing
	f32  posMm[3]     robot pose interpolated to the sweep, millimetres
	f32  quat[4]      orientation, w x y z
	u8   poseValid    0 when no pose source is configured or the sample is stale
	u8   pad[3]

Record header, 16 bytes, then nPoints i16 samples:

	u16 firingIndex
	u16 spatialIndex
	u64 timerUs       board timer, 1 us resolution, from the frame header
	u16 nPoints
	u8  kind          0 HF raw, 1 envelope
	u8  pad

Every offset stays 2-byte aligned so the browser can take a zero-copy
Int16Array view over the sample block.
*/
const (
	BundleHeaderBytes = 64
	RecordHeaderBytes = 16
	MsgTypeAscan      = 0x01
)

// Pose is a robot pose sample interpolated to a sweep timestamp.
type Pose struct {
	PosMm [3]float32
	Quat  [4]float32 // w x y z
	Valid bool
}

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
	Encoders   [4]int32
	Pose       Pose
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
	for i := 0; i < 3; i++ {
		binary.LittleEndian.PutUint32(b[32+4*i:], math.Float32bits(m.Pose.PosMm[i]))
	}
	for i := 0; i < 4; i++ {
		binary.LittleEndian.PutUint32(b[44+4*i:], math.Float32bits(m.Pose.Quat[i]))
	}
	if m.Pose.Valid {
		b[60] = 1
	} else {
		b[60] = 0
	}
	b[61], b[62], b[63] = 0, 0, 0

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
