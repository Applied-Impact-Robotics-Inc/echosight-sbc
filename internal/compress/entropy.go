package compress

import (
	"bytes"
	"compress/zlib"
	"fmt"
)

// ============================================================================
// STAGE 4 - ENTROPY CODING
//
// This is the only general-purpose stage in the chain, and it only earns its
// place because stages 1-3 ran first. On raw RF, zlib-1 achieved 1.42x: a
// noisy carrier is close to incompressible and the CPU spent is wasted. On
// decimated baseband the same coder reaches 1.38-1.63x because the data is
// now smooth rather than oscillating.
//
// BENCHMARK STATUS - read this before choosing a codec.
//
//	MEASURED : zlib-1 throughput = 51 MB/s. This is BELOW the ~122 MB/s
//	           ingest rate, which is precisely why zlib is disqualified.
//	           The number came from a development machine.
//	NOT MEASURED : zstd-1 and LZ4 throughput, on any machine.
//	NOT MEASURED : anything at all on the actual PICO-RAP4 board.
//
// zstd-1 and LZ4 are the RECOMMENDED swap, not the tested one. Do not treat
// the choice below as validated until someone runs Benchmark against a real
// capture on the target hardware with PL1 clamped to 12 W and the DSP pinned
// to the E-cores, because thermal headroom is the whole question and a
// desktop measurement will not answer it.
// ============================================================================

// Codec is stage 4. Implementations must be safe for concurrent use by
// multiple goroutines, or the pipeline must hold one per worker.
type Codec interface {
	// Name identifies the codec in the frame header so the reconstruction
	// side knows what to run.
	Name() string
	// Compress appends to dst and returns it.
	Compress(dst, src []byte) ([]byte, error)
	// Decompress appends to dst and returns it.
	Decompress(dst, src []byte) ([]byte, error)
}

// CodecNone passes bytes through. Useful for isolating how much of the ratio
// comes from stages 1-3 alone (measured: 2.67x at D=4/12-bit) and for
// bringing the board up before a codec is wired.
type CodecNone struct{}

func (CodecNone) Name() string { return "none" }
func (CodecNone) Compress(dst, src []byte) ([]byte, error) {
	return append(dst, src...), nil
}
func (CodecNone) Decompress(dst, src []byte) ([]byte, error) {
	return append(dst, src...), nil
}

// CodecZlib is the historical implementation, kept ONLY so the 51 MB/s
// measurement stays reproducible and so an A/B against the replacement is
// possible on the target board.
//
// Do not ship this. At 51 MB/s against 122 MB/s of ingest it cannot keep up,
// and the failure mode is the ring filling and frames being dropped.
type CodecZlib struct{}

func (CodecZlib) Name() string { return "zlib-1" }

func (CodecZlib) Compress(dst, src []byte) ([]byte, error) {
	var buf bytes.Buffer
	w, err := zlib.NewWriterLevel(&buf, zlib.BestSpeed)
	if err != nil {
		return dst, err
	}
	if _, err := w.Write(src); err != nil {
		return dst, err
	}
	if err := w.Close(); err != nil {
		return dst, err
	}
	return append(dst, buf.Bytes()...), nil
}

func (CodecZlib) Decompress(dst, src []byte) ([]byte, error) {
	r, err := zlib.NewReader(bytes.NewReader(src))
	if err != nil {
		return dst, err
	}
	defer r.Close()
	var buf bytes.Buffer
	if _, err := buf.ReadFrom(r); err != nil {
		return dst, err
	}
	return append(dst, buf.Bytes()...), nil
}

// ---------------------------------------------------------------------------
// TODO(sbc): the actual production codec.
//
// Both candidates are C libraries, so they arrive through cgo — which this
// module already uses for the SI5G board, so no new build machinery is
// needed. Suggested bindings:
//
//	zstd : github.com/klauspost/compress/zstd  (pure Go, no cgo, ~1.5 GB/s
//	       decompress; encoder level 1 is the relevant setting)
//	LZ4  : github.com/pierrec/lz4/v4           (pure Go)
//
// Pure-Go implementations are listed first deliberately. They remove the cgo
// call from the hot path entirely and are fast enough on paper; if a
// measurement says otherwise, the C bindings (DataDog/zstd, cloudflare/golz4)
// are the fallback.
//
// The decision procedure, in order:
//  1. Run Benchmark below on a real capture, on the PICO-RAP4, at 12 W.
//  2. Require sustained throughput >= 150 MB/s (122 MB/s ingest plus margin;
//     the DSP stages are sharing those E-cores).
//  3. Among codecs that clear the bar, take the best ratio.
//  4. If none clear it, drop to CodecNone and accept 2.67x. The link has the
//     headroom for 46 MB/s. Dropping frames to chase a ratio is the one
//     outcome that is definitely wrong.
// ---------------------------------------------------------------------------

// Result is one Benchmark measurement.
type Result struct {
	Codec      string
	InBytes    int
	OutBytes   int
	Ratio      float64
	MBps       float64
	SecondsRun float64
}

func (r Result) String() string {
	return fmt.Sprintf("%-8s %6.2fx  %7.1f MB/s  (%d -> %d bytes over %.2fs)",
		r.Codec, r.Ratio, r.MBps, r.InBytes, r.OutBytes, r.SecondsRun)
}

func errf(format string, a ...any) error { return fmt.Errorf(format, a...) }
