//go:build linux

package spike

/*
#cgo LDFLAGS: -ldl
#include <dlfcn.h>
#include <stdint.h>
#include <stdlib.h>

// SI5G exported function signatures. The library is C ABI (SysV cdecl on
// Linux); `unit` is a plain int, every scalar is passed by pointer so the
// library can write back the clipped/applied value, and every function
// returns an int32 status code (0 = ok, negative = error).
typedef int32_t (*si_void_fn)(void);
typedef int32_t (*si_get_int_fn)(const char*, int, int32_t*);
typedef int32_t (*si_get_dbl_fn)(const char*, int, double*);
typedef int32_t (*si_get_str_fn)(const char*, int, int32_t*, char*);
typedef int32_t (*si_get_1di_fn)(const char*, int, int32_t*, void*);
typedef int32_t (*si_get_1dd_fn)(const char*, int, int32_t*, double*);
typedef int32_t (*si_set_int_fn)(const char*, int, int32_t*, int32_t*);
typedef int32_t (*si_set_dbl_fn)(const char*, int, double*, int32_t*);
typedef int32_t (*si_set_str_fn)(const char*, int, int32_t*, char*, int32_t*);
typedef int32_t (*si_set_1di_fn)(const char*, int, int32_t*, int32_t*, int32_t*);
typedef int32_t (*si_set_1dd_fn)(const char*, int, int32_t*, double*, int32_t*);
typedef int32_t (*si_exec_fn)(const char*);

// Go cannot call a raw C function pointer, so each signature gets a shim.
static int32_t si_call_void(void *f) {
	return ((si_void_fn)f)();
}
static int32_t si_call_get_int(void *f, const char *p, int u, int32_t *v) {
	return ((si_get_int_fn)f)(p, u, v);
}
static int32_t si_call_get_dbl(void *f, const char *p, int u, double *v) {
	return ((si_get_dbl_fn)f)(p, u, v);
}
static int32_t si_call_get_str(void *f, const char *p, int u, int32_t *dim, char *buf) {
	return ((si_get_str_fn)f)(p, u, dim, buf);
}
static int32_t si_call_get_1di(void *f, const char *p, int u, int32_t *dim, void *buf) {
	return ((si_get_1di_fn)f)(p, u, dim, buf);
}
static int32_t si_call_get_1dd(void *f, const char *p, int u, int32_t *dim, double *buf) {
	return ((si_get_1dd_fn)f)(p, u, dim, buf);
}
static int32_t si_call_set_int(void *f, const char *p, int u, int32_t *v, int32_t *clip) {
	return ((si_set_int_fn)f)(p, u, v, clip);
}
static int32_t si_call_set_dbl(void *f, const char *p, int u, double *v, int32_t *clip) {
	return ((si_set_dbl_fn)f)(p, u, v, clip);
}
static int32_t si_call_set_str(void *f, const char *p, int u, int32_t *dim, char *buf, int32_t *clip) {
	return ((si_set_str_fn)f)(p, u, dim, buf, clip);
}
static int32_t si_call_set_1di(void *f, const char *p, int u, int32_t *dim, int32_t *v, int32_t *clip) {
	return ((si_set_1di_fn)f)(p, u, dim, v, clip);
}
static int32_t si_call_set_1dd(void *f, const char *p, int u, int32_t *dim, double *v, int32_t *clip) {
	return ((si_set_1dd_fn)f)(p, u, dim, v, clip);
}
static int32_t si_call_exec(void *f, const char *a) {
	return ((si_exec_fn)f)(a);
}
*/
import "C"

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"unsafe"
)

// Candidate shared-object names. Linux is case sensitive; Windows was not, so
// the SDK's naming is not guaranteed to match what the docs show.
var libNames = []string{
	"libsi5g.so",
	"libSI5G.so",
	"si5g.so",
	"libsi5g.so.1",
}

// Directories under $ROOT_SPIKE to search, in order.
var libDirs = []string{"Libraries", "libraries", "lib", "Lib", "bin", "."}

type library struct {
	handle unsafe.Pointer
	path   string

	open  unsafe.Pointer
	close unsafe.Pointer

	getInt   unsafe.Pointer
	getDbl   unsafe.Pointer
	getStr   unsafe.Pointer
	get1DInt unsafe.Pointer
	get1DDbl unsafe.Pointer

	setInt   unsafe.Pointer
	setDbl   unsafe.Pointer
	setStr   unsafe.Pointer
	set1DInt unsafe.Pointer
	set1DDbl unsafe.Pointer

	execSync  unsafe.Pointer
	execAsync unsafe.Pointer
}

var lib *library

// LibPath reports the shared object that was actually loaded. Empty before
// Open() succeeds. Worth logging at startup: the failure mode where a stale
// copy of the SDK is picked up off LD_LIBRARY_PATH is otherwise invisible.
func LibPath() string {
	if lib == nil {
		return ""
	}
	return lib.path
}

// resolveLibPath finds libsi5g.so. Precedence:
//  1. $SPIKE_LIB, if set, used verbatim
//  2. $ROOT_SPIKE/<dir>/<name> for each candidate
//  3. bare name, letting the dynamic loader use LD_LIBRARY_PATH / ldconfig
func resolveLibPath() []string {
	if p := os.Getenv("SPIKE_LIB"); p != "" {
		return []string{p}
	}
	var out []string
	if root := os.Getenv("ROOT_SPIKE"); root != "" {
		for _, d := range libDirs {
			for _, n := range libNames {
				p := filepath.Join(root, d, n)
				if _, err := os.Stat(p); err == nil {
					out = append(out, p)
				}
			}
		}
	}
	// Fall through to the loader's own search path.
	out = append(out, libNames...)
	return out
}

func dlerror() string {
	if e := C.dlerror(); e != nil {
		return C.GoString(e)
	}
	return "unknown dlopen error"
}

func loadLibrary() error {
	candidates := resolveLibPath()
	var attempts []string

	var handle unsafe.Pointer
	var loaded string
	for _, p := range candidates {
		cpath := C.CString(p)
		C.dlerror() // clear any stale error
		// RTLD_LOCAL, not RTLD_GLOBAL. This was found the hard way: under
		// RTLD_GLOBAL the library's constructor-time USB attach silently
		// fails — number_of_devices stays 0 and Open() returns -102 with a
		// board plugged in and working — while under RTLD_LOCAL the same
		// process detects and opens the board immediately. RTLD_LOCAL is
		// also exactly what Python's ctypes.CDLL uses, which is why the SDK
		// samples worked while this server did not. The earlier assumption
		// that the plugin .so files need our symbols exported globally was
		// wrong: libpath.so, libfocalTools.so and libplate.so all load fine
		// under RTLD_LOCAL (each dlopen'd library still sees its own
		// dependency chain; RTLD_GLOBAL only adds cross-library symbol
		// interposition, which is precisely what broke the USB attach).
		h := C.dlopen(cpath, C.RTLD_NOW|C.RTLD_LOCAL)
		C.free(unsafe.Pointer(cpath))
		if h != nil {
			handle = unsafe.Pointer(h)
			loaded = p
			break
		}
		attempts = append(attempts, fmt.Sprintf("  %s: %s", p, dlerror()))
	}

	if handle == nil {
		root := os.Getenv("ROOT_SPIKE")
		hint := "ROOT_SPIKE is not set"
		if root != "" {
			hint = "ROOT_SPIKE=" + root
		}
		return fmt.Errorf(
			"spike: could not load the SI5G shared library (%s)\n"+
				"tried:\n%s\n"+
				"check: find $ROOT_SPIKE -name '*si5g*'\n"+
				"       ldd <path>/libsi5g.so | grep 'not found'\n"+
				"and export LD_LIBRARY_PATH=$ROOT_SPIKE/Libraries:$LD_LIBRARY_PATH before running",
			hint, strings.Join(attempts, "\n"))
	}

	l := &library{handle: handle, path: loaded}

	var missing []string
	resolve := func(names ...string) unsafe.Pointer {
		for _, n := range names {
			cn := C.CString(n)
			C.dlerror()
			sym := C.dlsym(handle, cn)
			C.free(unsafe.Pointer(cn))
			if sym != nil {
				return unsafe.Pointer(sym)
			}
		}
		missing = append(missing, names[0])
		return nil
	}

	// The Windows DLL exported undecorated names. If the Linux build differs,
	// the SI_-prefixed fallbacks cover the common variant; anything C++
	// mangled (_ZN...) needs the real name adding here.
	l.open = resolve("Open", "SI_Open", "si5g_Open")
	l.close = resolve("Close", "SI_Close", "si5g_Close")
	l.getInt = resolve("GetInt", "SI_GetInt")
	l.getDbl = resolve("GetDbl", "SI_GetDbl")
	l.getStr = resolve("GetStr", "SI_GetStr")
	l.get1DInt = resolve("Get1DInt", "SI_Get1DInt")
	l.get1DDbl = resolve("Get1DDbl", "SI_Get1DDbl")
	l.setInt = resolve("SetInt", "SI_SetInt")
	l.setDbl = resolve("SetDbl", "SI_SetDbl")
	l.setStr = resolve("SetStr", "SI_SetStr")
	l.set1DInt = resolve("Set1DInt", "SI_Set1DInt")
	l.set1DDbl = resolve("Set1DDbl", "SI_Set1DDbl")
	l.execSync = resolve("ExecSync", "SI_ExecSync")
	l.execAsync = resolve("ExecAsync", "SI_ExecAsync")

	if len(missing) > 0 {
		return fmt.Errorf(
			"spike: loaded %s but these symbols are missing: %s\n"+
				"list what it actually exports with:\n"+
				"  nm -D --defined-only %s | grep ' T '\n"+
				"if the names are C++ mangled the candidate lists in lib_linux.go need updating",
			loaded, strings.Join(missing, ", "), loaded)
	}

	lib = l
	return nil
}

// ---------------------------------------------------------------------------
// Raw call shims. These do no locking and no error mapping; spike.go owns both.
// ---------------------------------------------------------------------------

func libOpen() int32  { return int32(C.si_call_void(lib.open)) }
func libClose() int32 { return int32(C.si_call_void(lib.close)) }

func libGetInt(param string, unit int, v *int32) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	return int32(C.si_call_get_int(lib.getInt, p, C.int(unit), (*C.int32_t)(v)))
}

func libGetDbl(param string, unit int, v *float64) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	return int32(C.si_call_get_dbl(lib.getDbl, p, C.int(unit), (*C.double)(v)))
}

// buf may be nil for the size-query pass.
func libGetStr(param string, unit int, dim *int32, buf []byte) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	var b *C.char
	if len(buf) > 0 {
		b = (*C.char)(unsafe.Pointer(&buf[0]))
	}
	return int32(C.si_call_get_str(lib.getStr, p, C.int(unit), (*C.int32_t)(dim), b))
}

func libGet1DInt(param string, unit int, dim *int32, buf []uint16) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	var b unsafe.Pointer
	if len(buf) > 0 {
		b = unsafe.Pointer(&buf[0])
	}
	return int32(C.si_call_get_1di(lib.get1DInt, p, C.int(unit), (*C.int32_t)(dim), b))
}

func libGet1DDbl(param string, unit int, dim *int32, buf []float64) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	var b *C.double
	if len(buf) > 0 {
		b = (*C.double)(unsafe.Pointer(&buf[0]))
	}
	return int32(C.si_call_get_1dd(lib.get1DDbl, p, C.int(unit), (*C.int32_t)(dim), b))
}

func libSetInt(param string, unit int, v *int32, clip *int32) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	return int32(C.si_call_set_int(lib.setInt, p, C.int(unit), (*C.int32_t)(v), (*C.int32_t)(clip)))
}

func libSetDbl(param string, unit int, v *float64, clip *int32) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	return int32(C.si_call_set_dbl(lib.setDbl, p, C.int(unit), (*C.double)(v), (*C.int32_t)(clip)))
}

func libSetStr(param string, unit int, dim *int32, val []byte, clip *int32) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	var b *C.char
	if len(val) > 0 {
		b = (*C.char)(unsafe.Pointer(&val[0]))
	}
	return int32(C.si_call_set_str(lib.setStr, p, C.int(unit), (*C.int32_t)(dim), b, (*C.int32_t)(clip)))
}

func libSet1DInt(param string, unit int, dim *int32, vals []int32, clip *int32) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	var b *C.int32_t
	if len(vals) > 0 {
		b = (*C.int32_t)(unsafe.Pointer(&vals[0]))
	}
	return int32(C.si_call_set_1di(lib.set1DInt, p, C.int(unit), (*C.int32_t)(dim), b, (*C.int32_t)(clip)))
}

func libSet1DDbl(param string, unit int, dim *int32, vals []float64, clip *int32) int32 {
	p := C.CString(param)
	defer C.free(unsafe.Pointer(p))
	var b *C.double
	if len(vals) > 0 {
		b = (*C.double)(unsafe.Pointer(&vals[0]))
	}
	return int32(C.si_call_set_1dd(lib.set1DDbl, p, C.int(unit), (*C.int32_t)(dim), b, (*C.int32_t)(clip)))
}

func libExecSync(action string) int32 {
	a := C.CString(action)
	defer C.free(unsafe.Pointer(a))
	return int32(C.si_call_exec(lib.execSync, a))
}

func libExecAsync(action string) int32 {
	a := C.CString(action)
	defer C.free(unsafe.Pointer(a))
	return int32(C.si_call_exec(lib.execAsync, a))
}
