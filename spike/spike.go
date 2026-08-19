//go:build linux

// Package spike provides Go bindings for the Socomate SPIKE SI5G library.
//
// Linux only. The shared object is located via SPIKE_LIB, or by searching
// $ROOT_SPIKE/{Libraries,lib,bin} for libsi5g.so, or by falling back to the
// dynamic loader's own path. Open() reads config-devices.ini from the SPIKE
// Config directory; the device mode (PA / FMC / FMC_TDM) is set there, not at
// runtime.
//
// Requires CGO_ENABLED=1 and a C toolchain. Export
// LD_LIBRARY_PATH=$ROOT_SPIKE/Libraries before running: libsi5g.so pulls in
// its own siblings (libusb, plugin .so files) and the dynamic loader reads
// that variable at process start, not from inside the program.
//
// Concurrency: SI5G is not documented as thread safe, and this server calls it
// from an acquisition goroutine and HTTP handlers at the same time, so every
// exported function here serializes on a package mutex.
package spike

import (
	"fmt"
	"sync"
)

// ============================================================================
// Unit codes (manual 5.1)
// ============================================================================

const (
	UnitNone       = -1
	UnitUs         = 0
	UnitMm         = 1
	UnitIn         = 2
	UnitNs         = 100
	UnitMs         = 101
	UnitS          = 102
	UnitUm         = 200
	UnitM          = 201
	UnitHz         = 300
	UnitKHz        = 301
	UnitMHz        = 302
	UnitMps        = 400
	UnitInPerUs    = 401
	UnitDeg        = 500
	UnitRad        = 501
	UnitCelsius    = 600
	UnitFahrenheit = 601
)

// Device modes (dev#1.mode)
const (
	ModePA     = 0
	ModeFMC    = 1
	ModeFMCTDM = 2
)

// ============================================================================
// Error codes (manual 5.3)
// ============================================================================

var errorNames = map[int]string{
	0:     "no error",
	-1:    "an exception occurred",
	-2:    "open file error",
	-10:   "null pointer",
	-11:   "invalid handle",
	-12:   "memory allocation failed",
	-100:  "access not opened",
	-101:  "access already opened",
	-102:  "device not found",
	-103:  "device not opened",
	-104:  "unknown model",
	-105:  "opening plugin failed",
	-106:  "license expired",
	-107:  "model error",
	-110:  "invalid argument",
	-111:  "unknown parameter",
	-112:  "invalid device index",
	-113:  "invalid script",
	-114:  "incompatible setting",
	-115:  "invalid element index",
	-116:  "incompatible mode",
	-117:  "invalid plugin syntax",
	-119:  "setting undefined",
	-120:  "invalid group index",
	-121:  "invalid sequence index",
	-122:  "invalid gate index",
	-130:  "not permitted to change the setting during firing",
	-131:  "not permitted to flush data when acquisition is running",
	-132:  "buffer is too small to receive data",
	-133:  "overflow error",
	-200:  "access timeout",
	-201:  "access failed",
	-300:  "invalid Synapse file",
	-301:  "unable to add a new group",
	-302:  "unable to delete the last group",
	-1000: "invalid USB device",
	-1001: "PLL initialization failed",
	-1002: "downloading firmware failed",
	-1003: "firmware initialization failed",
	-1004: "device initialization failed",
}

// Error wraps a nonzero SI5G return code with the parameter that caused it.
type Error struct {
	Code  int
	Param string
	Op    string
}

func (e *Error) Error() string {
	name, ok := errorNames[e.Code]
	if !ok {
		name = "unknown error"
	}
	if e.Param != "" {
		return fmt.Sprintf("spike: %s %q failed: %s (%d)", e.Op, e.Param, name, e.Code)
	}
	return fmt.Sprintf("spike: %s failed: %s (%d)", e.Op, name, e.Code)
}

// ErrorName returns the manual's text for a status code, for surfacing raw
// codes in the UI without a second lookup table on the frontend.
func ErrorName(code int) string {
	if n, ok := errorNames[code]; ok {
		return n
	}
	return "unknown error"
}

func check(op, param string, code int32) error {
	if code == 0 {
		return nil
	}
	return &Error{Code: int(code), Param: param, Op: op}
}

// ============================================================================
// Library lifecycle
// ============================================================================

var (
	mu       sync.Mutex
	loadOnce sync.Once
	loadErr  error
	opened   bool
)

// Open loads libsi5g.so (once) and opens access to the device(s) declared in
// config-devices.ini.
func Open() error {
	mu.Lock()
	defer mu.Unlock()

	loadOnce.Do(func() { loadErr = loadLibrary() })
	if loadErr != nil {
		return loadErr
	}
	if err := check("Open", "", libOpen()); err != nil {
		return err
	}
	opened = true
	return nil
}

// Close releases the library access. Call on shutdown. Safe to call when the
// device was never opened.
func Close() error {
	mu.Lock()
	defer mu.Unlock()

	if lib == nil || !opened {
		return nil
	}
	opened = false
	return check("Close", "", libClose())
}

// IsOpen reports whether Open() has succeeded and Close() has not run.
func IsOpen() bool {
	mu.Lock()
	defer mu.Unlock()
	return opened
}

func ensure() error {
	if lib == nil {
		return fmt.Errorf("spike: library not loaded (call Open first)")
	}
	return nil
}

// ============================================================================
// Typed Get / Set
// ============================================================================

// GetInt reads an integer parameter.
func GetInt(param string, unit int) (int, error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return 0, err
	}
	var v int32
	code := libGetInt(param, unit, &v)
	return int(v), check("GetInt", param, code)
}

// GetDbl reads a double parameter.
func GetDbl(param string, unit int) (float64, error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return 0, err
	}
	var v float64
	code := libGetDbl(param, unit, &v)
	return v, check("GetDbl", param, code)
}

// GetStr reads a string parameter using the two-step size query (manual 3.2).
func GetStr(param string, unit int) (string, error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return "", err
	}

	var dim int32
	if err := check("GetStr", param, libGetStr(param, unit, &dim, nil)); err != nil {
		return "", err
	}
	if dim <= 0 {
		return "", nil
	}
	buf := make([]byte, dim+1)
	if err := check("GetStr", param, libGetStr(param, unit, &dim, buf)); err != nil {
		return "", err
	}
	// Trim at NUL if present.
	n := 0
	for n < len(buf) && buf[n] != 0 {
		n++
	}
	return string(buf[:n]), nil
}

// GetDblArray reads a double array parameter of known length.
func GetDblArray(param string, unit int, n int) ([]float64, error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return nil, err
	}
	dim := int32(n)
	buf := make([]float64, n)
	code := libGet1DDbl(param, unit, &dim, buf)
	if err := check("Get1DDbl", param, code); err != nil {
		return nil, err
	}
	if int(dim) < n {
		buf = buf[:dim]
	}
	return buf, nil
}

// SetInt writes an integer parameter. Returns the value actually applied
// (the library clips out-of-range values) and whether clipping occurred.
func SetInt(param string, unit, value int) (applied int, clipped bool, err error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return 0, false, err
	}
	v := int32(value)
	var clip int32
	code := libSetInt(param, unit, &v, &clip)
	return int(v), clip != 0, check("SetInt", param, code)
}

// SetDbl writes a double parameter.
func SetDbl(param string, unit int, value float64) (applied float64, clipped bool, err error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return 0, false, err
	}
	v := value
	var clip int32
	code := libSetDbl(param, unit, &v, &clip)
	return v, clip != 0, check("SetDbl", param, code)
}

// SetStr writes a string parameter (e.g. the firing script on dev#1.settings).
// Pass "" to clear.
func SetStr(param string, unit int, value string) error {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return err
	}
	dim := int32(len(value))
	b := append([]byte(value), 0)
	var clip int32
	return check("SetStr", param, libSetStr(param, unit, &dim, b, &clip))
}

// Set1DInt writes an int array parameter (e.g. receiver_analog_filters,
// ascans_selection).
func Set1DInt(param string, unit int, values []int32) error {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return err
	}
	if len(values) == 0 {
		return fmt.Errorf("spike: Set1DInt %q: empty slice", param)
	}
	dim := int32(len(values))
	var clip int32
	return check("Set1DInt", param, libSet1DInt(param, unit, &dim, values, &clip))
}

// Set1DDbl writes a double array parameter (e.g. band pass filter, DAC points).
func Set1DDbl(param string, unit int, values []float64) error {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return err
	}
	if len(values) == 0 {
		return fmt.Errorf("spike: Set1DDbl %q: empty slice", param)
	}
	dim := int32(len(values))
	var clip int32
	return check("Set1DDbl", param, libSet1DDbl(param, unit, &dim, values, &clip))
}

// Set1DDblDim writes a double array with an explicit dim that may be smaller
// than the backing slice — including dim=0, which the manual requires for
// clearing DAC points ("the value pointer must NOT be NULL", gotcha 2.8.3.6):
// the slice supplies a valid pointer while dim tells the library "no points".
func Set1DDblDim(param string, unit int, dim int, values []float64) error {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return err
	}
	if len(values) == 0 {
		return fmt.Errorf("spike: Set1DDblDim %q: backing slice must be non-empty", param)
	}
	d := int32(dim)
	var clip int32
	return check("Set1DDbl", param, libSet1DDbl(param, unit, &d, values, &clip))
}

// GetData reads acquisition data (16-bit words) into buf and returns the
// number of words written. Used for dev#1.data and dev#1.ascans_on_demand.
//
// Note: dev#1.data blocks until the requested amount is available when a
// transfer has been initialized (FMC path). In the polled PA path pass the
// size reported by dev#1.available_data so this returns immediately, otherwise
// it will hold the package mutex and stall every config read.
func GetData(param string, dim int, buf []uint16) (int, error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return 0, err
	}
	if dim > len(buf) {
		return 0, fmt.Errorf("spike: GetData %q: buffer holds %d words, asked for %d", param, len(buf), dim)
	}
	if dim <= 0 {
		return 0, nil
	}
	d := int32(dim)
	code := libGet1DInt(param, UnitNone, &d, buf)
	return int(d), check("Get1DInt", param, code)
}

// ExecSync executes a named task (plugins, e.g. "focaltools.Calculate")
// and waits for completion.
func ExecSync(action string) error {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return err
	}
	return check("ExecSync", action, libExecSync(action))
}

// ExecAsync starts a named task without waiting.
func ExecAsync(action string) error {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return err
	}
	return check("ExecAsync", action, libExecAsync(action))
}
