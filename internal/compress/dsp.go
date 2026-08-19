package compress

import "math"

// ============================================================================
// STAGE 1-3 DSP KERNELS
//
// The chain is FMC-specific, not a general-purpose compressor. Stages 1-3 do
// the work that MAKES stage 4 possible: a raw RF trace is a noisy 5 MHz
// carrier and is very nearly incompressible (zlib-1 measured 1.42x on raw RF).
// Once demodulated and decimated, the same coder reaches 1.38-1.63x.
//
// Measured end-to-end ratios against 800-point int16, from 122 MB/s ingest:
//
//	D=4  12-bit  packed only   2.67x   ->  46 MB/s
//	D=4  12-bit  + entropy     3.68x   ->  33 MB/s
//	D=4  10-bit  + entropy     5.20x   ->  24 MB/s
//	D=6  12-bit  + entropy     5.44x   ->  22 MB/s   <- production default
//	D=6  10-bit  + entropy     7.57x   ->  16 MB/s
//
// Note the arithmetic if the numbers ever look wrong: decimating by 4 nets
// only a factor of 2 because demodulation turns one real stream into two
// (I and Q). 2 x 1.33 = 2.67, which is exactly the measured D=4 12-bit
// figure. If a change makes the ratio come out at a clean 4x, something has
// silently dropped the Q channel and phase information is gone.
//
// Total cost of stages 1-3 is ~4.3 GFLOP/s, about a quarter of one AVX2 core.
// This is not the bottleneck and should not be optimised speculatively.
// ============================================================================

// Params configures the chain. Defaults are the production settings.
type Params struct {
	SampleRateMHz float64 // ADC rate
	CarrierMHz    float64 // mixing frequency, = probe centre frequency
	CutoffMHz     float64 // low-pass corner; MUST be well below CarrierMHz
	Taps          int     // FIR length
	Decimation    int     // D
	Bits          int     // 10 or 12
}

// DefaultParams is the validated production configuration.
//
// Cutoff sits at 3 MHz against a 5 MHz carrier deliberately. Stage 2 is not
// only a decimation anti-alias filter: it must also delete the sum-frequency
// image that stage 1 parks at 2*carrier (10 MHz). A real filter has a slope,
// not a wall, so the image has to sit far enough up that slope to be crushed.
// Moving this cutoff up toward the carrier lets the image leak back into the
// kept copy and corrupt it. That was one of the two bugs in the original
// ad-hoc implementation and it does not announce itself: the output still
// looks like a plausible A-scan.
func DefaultParams() Params {
	return Params{
		SampleRateMHz: 50, // FMC firmware digitizes at a fixed 50 MHz; there is
		// no sampling code to choose. See DigitizationConfig in wire.go.
		CarrierMHz:    5,
		CutoffMHz:     3,
		Taps:          65,
		Decimation:    6,
		Bits:          12,
	}
}

// Validate rejects configurations known to produce silently wrong output.
func (p Params) Validate() error {
	switch {
	case p.Decimation < 1:
		return errf("decimation must be >= 1, got %d", p.Decimation)
	case p.Taps < 9 || p.Taps%2 == 0:
		return errf("taps must be odd and >= 9, got %d", p.Taps)
	case p.Bits != 10 && p.Bits != 12 && p.Bits != 16:
		return errf("bits must be 10, 12 or 16, got %d", p.Bits)
	case p.CutoffMHz >= p.CarrierMHz*0.75:
		// See the DefaultParams comment. This is a hard guard, not a style
		// preference: above roughly 0.75*carrier the 2*carrier image is no
		// longer adequately attenuated by a filter of this length.
		return errf("cutoff %.2f MHz is too close to carrier %.2f MHz; the "+
			"2x-carrier image will leak into the kept baseband copy",
			p.CutoffMHz, p.CarrierMHz)
	case p.CutoffMHz*2*float64(p.Decimation) > p.SampleRateMHz*1.001:
		return errf("cutoff %.2f MHz aliases at D=%d on a %.1f MHz sampler",
			p.CutoffMHz, p.Decimation, p.SampleRateMHz)
	}
	return nil
}

// Kernel holds the precomputed mixing tables and FIR taps. Build once, reuse
// for every trace; it is read-only after construction and safe to share
// across goroutines.
type Kernel struct {
	P    Params
	cos  []float64 // mixer, one period-aligned table
	sin  []float64
	taps []float64
	// period is the length of the mixer tables. Sample rate and carrier are
	// chosen so this is exact; if it ever isn't, the tables get long rather
	// than wrong.
	period int
}

// NewKernel precomputes the mixer and filter for a given configuration.
func NewKernel(p Params) (*Kernel, error) {
	if err := p.Validate(); err != nil {
		return nil, err
	}
	k := &Kernel{P: p}
	k.period = mixPeriod(p.SampleRateMHz, p.CarrierMHz)
	k.cos = make([]float64, k.period)
	k.sin = make([]float64, k.period)
	w := 2 * math.Pi * p.CarrierMHz / p.SampleRateMHz
	for i := 0; i < k.period; i++ {
		k.cos[i] = math.Cos(w * float64(i))
		k.sin[i] = math.Sin(w * float64(i))
	}
	k.taps = lowpassTaps(p.Taps, p.CutoffMHz/p.SampleRateMHz)
	return k, nil
}

// mixPeriod returns a table length over which the mixer repeats exactly, or a
// pragmatic cap when the ratio is irrational.
func mixPeriod(fsMHz, f0MHz float64) int {
	const cap = 4096
	for n := 1; n <= cap; n++ {
		cycles := f0MHz * float64(n) / fsMHz
		if math.Abs(cycles-math.Round(cycles)) < 1e-12 {
			return n
		}
	}
	return cap
}

// lowpassTaps builds a windowed-sinc low-pass. fc is normalised (cycles per
// sample), i.e. cutoffHz / sampleRateHz.
func lowpassTaps(n int, fc float64) []float64 {
	t := make([]float64, n)
	mid := (n - 1) / 2
	var sum float64
	for i := 0; i < n; i++ {
		x := float64(i - mid)
		var s float64
		if x == 0 {
			s = 2 * fc
		} else {
			s = math.Sin(2*math.Pi*fc*x) / (math.Pi * x)
		}
		// Blackman window: the stopband attenuation matters here because the
		// stopband is where the 2x-carrier image has to die.
		a := 2 * math.Pi * float64(i) / float64(n-1)
		w := 0.42 - 0.5*math.Cos(a) + 0.08*math.Cos(2*a)
		t[i] = s * w
		sum += t[i]
	}
	for i := range t {
		t[i] /= sum // unity DC gain
	}
	return t
}

// Baseband is one trace after stages 1 and 2: complex, decimated, still
// floating point. Length is ceil(inputLen / D).
type Baseband struct {
	I []float64
	Q []float64
	// Mixer scratch, one trace long. Lives here rather than on Kernel
	// because Kernel is shared read-only across workers while Baseband is
	// per-Compressor.
	mi []float64
	mq []float64
}

// Demodulate runs stage 1 and stage 2 together in one pass.
//
// STAGE 1 - quadrature demodulation. Multiplying the trace by a reference at
// the carrier frequency produces two terms, not one:
//
//	cos(A)*cos(B) = 0.5 * [ cos(A-B) + cos(A+B) ]
//
// The difference term lands at DC and the sum term at 2*carrier. Both carry
// the same envelope; we keep the one at DC because content near DC is
// slow-moving and can therefore be decimated. Mixing against cos and sin
// gives I and Q. Both are required: with the carrier removed, phase has
// nowhere else to live, and the envelope is sqrt(I^2 + Q^2). Dropping Q
// yields an envelope that collapses to zero whenever the phase happens to sit
// at 90 degrees.
//
// This costs 2 multiplies per sample. The equivalent via a 65-tap Hilbert
// transform is 5.9x more expensive for the same result.
//
// THE FACTOR OF 2. Note the 0.5 in the identity above. Mixing a real signal
// halves its amplitude, so the output is scaled by 2 to restore it. Omitting
// this was the other of the two bugs in the ad-hoc implementation and every
// reconstruction came out 6 dB low. It does not look like an error; it looks
// like a quiet probe.
//
// STAGE 2 - polyphase decimating low-pass. The filter is only ever evaluated
// at output positions that survive decimation, so the cost is 1/D of the
// naive filter-then-drop ordering. It performs two jobs at once: deleting the
// 2*carrier image from stage 1, and band-limiting ahead of the decimation.
func (k *Kernel) Demodulate(trace []int16, dst *Baseband) {
	n := len(trace)
	d := k.P.Decimation
	outN := (n + d - 1) / d
	dst.I = grow(dst.I, outN)
	dst.Q = grow(dst.Q, outN)
	dst.mi = grow(dst.mi, n)
	dst.mq = grow(dst.mq, n)

	// PASS 1 - mix. Walk the mixer table with a rotating counter.
	//
	// The obvious formulation folds the mixer into the filter loop and
	// indexes the table with idx%period. That costs one integer division per
	// tap, which at 65 taps x 134 outputs x 32 channels x 32 shots is ~8.9
	// million divisions per acquisition cycle — measured at 41 ms/cycle
	// against a 14.4 ms budget on the PICO-RAP4. Mixing separately costs n
	// extra multiplies per trace and removes every division.
	mi, mq := dst.mi, dst.mq
	cos, sin := k.cos, k.sin
	m := 0
	for i := 0; i < n; i++ {
		v := float64(trace[i])
		mi[i] = v * cos[m]
		mq[i] = -v * sin[m]
		m++
		if m == k.period {
			m = 0
		}
	}

	// PASS 2 - decimating FIR over the mixed signal.
	//
	// Split into edge and interior so the interior taps need no bounds
	// check. The edges are the only places the filter window hangs off the
	// trace, and they are a handful of outputs out of ~134.
	taps := k.taps
	nt := len(taps)
	half := (nt - 1) / 2

	lo := (half + d - 1) / d      // first output whose window fits
	hi := (n - 1 - half) / d      // last output whose window fits
	if hi >= outN {
		hi = outN - 1
	}
	if lo > outN {
		lo = outN
	}

	edge := func(o int) {
		c := o*d - half
		var accI, accQ float64
		for t := 0; t < nt; t++ {
			idx := c + t
			if idx < 0 || idx >= n {
				continue // zero-extend
			}
			h := taps[t]
			accI += mi[idx] * h
			accQ += mq[idx] * h
		}
		dst.I[o] = 2 * accI
		dst.Q[o] = 2 * accQ
	}
	for o := 0; o < lo && o < outN; o++ {
		edge(o)
	}
	for o := hi + 1; o < outN; o++ {
		edge(o)
	}

	for o := lo; o <= hi; o++ {
		base := o*d - half
		si := mi[base : base+nt]
		sq := mq[base : base+nt]
		var accI, accQ float64
		for t, h := range taps {
			accI += si[t] * h
			accQ += sq[t] * h
		}
		// x2 restores the amplitude halved by real-signal mixing. See the
		// product-to-sum identity in the Demodulate doc comment.
		dst.I[o] = 2 * accI
		dst.Q[o] = 2 * accQ
	}
}

// Quantise runs stage 3: pack float baseband into Bits-wide integers.
//
// The ADC delivers 16 bits, but the measured dynamic range does not fill
// them. Peak sample is 11426 counts against an electronic noise floor of 5.5,
// which is 11.0 real bits of information stored in 16. Discarding the bottom
// bits therefore discards noise, and 16-, 12- and 10-bit variants produce
// IDENTICAL reconstruction error in testing. 12-bit is the production choice;
// 10-bit is available if the link ever tightens.
//
// Scale is per-trace so the full code range is used regardless of gain
// settings, and is stored in the frame header for reconstruction.
func (k *Kernel) Quantise(b *Baseband, out *Quantised) {
	n := len(b.I)
	out.N = n
	out.Bits = k.P.Bits
	out.I = growI32(out.I, n)
	out.Q = growI32(out.Q, n)

	peak := 0.0
	for i := 0; i < n; i++ {
		peak = math.Max(peak, math.Max(math.Abs(b.I[i]), math.Abs(b.Q[i])))
	}
	full := float64(int32(1)<<(k.P.Bits-1) - 1)
	if peak <= 0 {
		out.Scale = 1
		for i := 0; i < n; i++ {
			out.I[i], out.Q[i] = 0, 0
		}
		return
	}
	out.Scale = peak / full
	inv := full / peak
	for i := 0; i < n; i++ {
		out.I[i] = int32(math.Round(b.I[i] * inv))
		out.Q[i] = int32(math.Round(b.Q[i] * inv))
	}
}

// Quantised is one trace after stage 3.
type Quantised struct {
	I     []int32
	Q     []int32
	N     int
	Bits  int
	Scale float64 // multiply codes by this to recover baseband amplitude
}

// PackBits writes the quantised codes as a dense Bits-wide bitstream. This is
// where the 1.33x (12-bit) or 1.6x (10-bit) reduction is actually realised;
// leaving them in int32 would waste the whole stage.
func PackBits(q *Quantised, dst []byte) []byte {
	bits := q.Bits
	total := q.N * 2 * bits
	need := (total + 7) / 8
	dst = growB(dst, need)
	for i := range dst {
		dst[i] = 0
	}
	off := 0
	put := func(v int32) {
		u := uint32(v) & (1<<uint(bits) - 1) // two's complement, truncated
		for b := bits - 1; b >= 0; b-- {
			if u&(1<<uint(b)) != 0 {
				dst[off>>3] |= 0x80 >> uint(off&7)
			}
			off++
		}
	}
	for i := 0; i < q.N; i++ {
		put(q.I[i])
	}
	for i := 0; i < q.N; i++ {
		put(q.Q[i])
	}
	return dst
}

// ---------------------------------------------------------------------------
// RECONSTRUCTION SIDE
//
// These live here so the forward and inverse stay in one file and cannot drift
// apart. They run on the Windows compute server, not the SBC.
// ---------------------------------------------------------------------------

// Interp selects the reconstruction interpolator.
type Interp int

const (
	// InterpCubic is the only correct choice. See ReconstructionNote.
	InterpCubic Interp = iota
	// InterpLinear exists to reproduce the historical measurement and must
	// not be used in production.
	InterpLinear
)

// ReconstructionNote records the finding that changed the decimation choice,
// because it is the kind of thing that gets rediscovered expensively.
//
// Measured reconstruction error, flat machined plate:
//
//	D    linear rebuild   cubic rebuild
//	4        5.78%           4.66%
//	6        8.28%           4.85%
//	8       12.20%           5.67%
//
// The error at high D was dominated by the INTERPOLATOR, not by the
// compression. With cubic reconstruction D=6 is indistinguishable from D=4,
// which is where the extra 1.5x of the production ratio comes from. D=6 was
// rejected once purely because it had been judged against a linear rebuild.
//
// Never evaluate a decimation factor with linear interpolation.
const ReconstructionNote = "cubic required; linear rebuild misattributes interpolation error to compression"