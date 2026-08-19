//go:build linux

package spike

import "fmt"

// ============================================================================
// PA-mode frame layout (manual 2.8.6.19.1). Word order within a frame:
//
//	Metadata  56 words  (gate positions/widths/levels, DAC curve, probe/ascan ID)
//	Header    16 words  (64-bit µs timer, 4x 32-bit encoders, reserved)
//	A-scan    size_of_digitalization words
//	Footer    16 words  (per-gate amplitudes and times of flight)
//
// Which blocks are present depends on the frame_metadata / frame_header /
// frame_ascan / frame_footer indicators. FrameLayout captures that so parsing
// stays correct if you later collect C-scan-only (footer) frames.
// ============================================================================

// FrameLayout describes which blocks the device includes in each frame.
type FrameLayout struct {
	Metadata  bool
	Header    bool
	Ascan     bool
	Footer    bool
	AscanSize int // words, = size_of_digitalization
	FrameSize int // total words, = dev#1.grp#1.frame_size
}

// ReadFrameLayout queries the device for the current frame composition of a
// group (1-based index).
func ReadFrameLayout(group int) (FrameLayout, error) {
	var l FrameLayout
	p := func(name string) string { return fmt.Sprintf("dev#1.grp#%d.%s", group, name) }

	m, err := GetInt(p("frame_metadata"), UnitNone)
	if err != nil {
		return l, err
	}
	h, err := GetInt(p("frame_header"), UnitNone)
	if err != nil {
		return l, err
	}
	a, err := GetInt(p("frame_ascan"), UnitNone)
	if err != nil {
		return l, err
	}
	f, err := GetInt(p("frame_footer"), UnitNone)
	if err != nil {
		return l, err
	}
	size, err := GetInt(p("size_of_digitalization"), UnitNone)
	if err != nil {
		return l, err
	}
	fs, err := GetInt(p("frame_size"), UnitNone)
	if err != nil {
		return l, err
	}

	l = FrameLayout{
		Metadata:  m != 0,
		Header:    h != 0,
		Ascan:     a != 0,
		Footer:    f != 0,
		AscanSize: size,
		FrameSize: fs,
	}
	return l, nil
}
