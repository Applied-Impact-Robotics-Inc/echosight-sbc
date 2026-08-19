//go:build linux

package spike

import (
	"strings"
)

// Load resolves and dlopens libsi5g.so without opening access to any device.
//
// This exists because of manual 2.8.9.2: number_of_devices read BEFORE opening
// access returns the number of DETECTED devices, and only after opening does it
// return the number selected in config-devices.ini. The same holds for
// device_serial_numbers (2.8.9.4) and device_models (2.8.9.5).
//
// That gives the supervisor a cheap, side-effect-free presence check, so it
// never has to blind-retry Open() against an absent or wedged board. Open()
// downloads firmware and has no documented timeout, so guessing is expensive.
func Load() error {
	mu.Lock()
	defer mu.Unlock()
	loadOnce.Do(func() { loadErr = loadLibrary() })
	return loadErr
}

// Loaded reports whether the shared object is resolved.
func Loaded() bool {
	mu.Lock()
	defer mu.Unlock()
	return lib != nil
}

// DeviceCount returns number_of_devices. Before Open() this is the count of
// physically detected boards; after Open() it is the count declared in
// config-devices.ini.
func DeviceCount() (int, error) {
	return GetInt("number_of_devices", UnitNone)
}

// DeviceSerials returns device_serial_numbers with the same before/after
// semantics as DeviceCount. n is a hint for the buffer size.
func DeviceSerials(n int) ([]int32, error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return nil, err
	}
	if n <= 0 {
		n = 8
	}
	dim := int32(n)
	buf := make([]uint16, 2*n) // Get1DInt writes 32-bit ints; 2 words each
	code := libGet1DInt("device_serial_numbers", UnitNone, &dim, buf)
	if err := check("Get1DInt", "device_serial_numbers", code); err != nil {
		return nil, err
	}
	if int(dim) > n {
		dim = int32(n)
	}
	out := make([]int32, dim)
	for i := range out {
		out[i] = int32(uint32(buf[2*i]) | uint32(buf[2*i+1])<<16)
	}
	return out, nil
}

// DeviceModels returns the device_models JSON string, e.g. {"Models":["32x64"]}.
func DeviceModels() (string, error) {
	return GetStr("device_models", UnitNone)
}

// LibraryVersion returns the library_version string.
func LibraryVersion() (string, error) {
	return GetStr("library_version", UnitNone)
}

// Notification return values (manual 2.7).
const (
	NotifyNone            = 0
	NotifyParamFromOther  = 3 // another registered application changed a parameter
	NotifyParamFromThis   = 4 // we changed a parameter
)

// Notification polls SI_Notification. The library is shared and thread safe:
// several applications can drive the same board at once, and every registered
// application is notified when one of them changes a parameter (manual 2.2).
//
// A return of 3 means Synapse, or another echosight instance, has moved a
// setting out from under us and the staged config is stale.
func Notification() (int, error) {
	mu.Lock()
	defer mu.Unlock()
	if err := ensure(); err != nil {
		return 0, err
	}
	if !hasNotification() {
		return NotifyNone, nil
	}
	return int(libNotification()), nil
}

// IsDeviceGone classifies an error as "the board went away" rather than "we
// asked for something invalid". Only the former should tear down the session.
//
// Deliberately excluded: -110 invalid argument, -111 unknown parameter,
// -114 incompatible setting, -130 not permitted during firing, -132 buffer too
// small. Those are our bugs, and treating them as disconnects would drop a live
// scan every time a config write was malformed.
func IsDeviceGone(err error) bool {
	if err == nil {
		return false
	}
	var e *Error
	if !asError(err, &e) {
		return false
	}
	switch e.Code {
	case -100, // access not opened
		-102,  // device not found
		-103,  // device not opened
		-200,  // access timeout
		-201,  // access failed
		-1000, // invalid USB device
		-1001, // PLL init failed
		-1002, // downloading firmware failed
		-1003, // firmware init failed
		-1004: // device init failed
		return true
	}
	return false
}

// IsAlreadyOpen reports -101, which means a previous process left a stale
// registration in the library's internal database. Recover with Close+Open.
func IsAlreadyOpen(err error) bool {
	var e *Error
	return asError(err, &e) && e.Code == -101
}

func asError(err error, out **Error) bool {
	if e, ok := err.(*Error); ok {
		*out = e
		return true
	}
	return false
}

// Describe renders a load failure with the Linux-specific hints that actually
// resolve it, so the TUI can show something actionable.
func Describe(err error) string {
	if err == nil {
		return ""
	}
	var e *Error
	if !asError(err, &e) {
		return err.Error()
	}
	hint := ""
	switch e.Code {
	case -2:
		hint = "config-devices.ini not found or unreadable; it must live in the SPIKE Config directory"
	case -102:
		hint = "serial in config-devices.ini does not match the board, or no udev rule grants access. Setting SerialNumber to \"any\" removes the first case"
	case -106:
		hint = "license expired"
	case -1000, -1001, -1002, -1003, -1004:
		hint = "USB or firmware init failed; power cycle the board, check it enumerated at SuperSpeed"
	}
	if hint == "" {
		return e.Error()
	}
	return strings.Join([]string{e.Error(), hint}, " — ")
}
