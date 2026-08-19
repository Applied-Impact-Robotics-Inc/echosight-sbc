//go:build linux

package spike

/*
#cgo LDFLAGS: -ldl
#include <dlfcn.h>
#include <stdint.h>
#include <stdlib.h>

typedef int32_t (*si_notify_fn)(void);
static int32_t si_call_notify(void *f) { return ((si_notify_fn)f)(); }
*/
import "C"

import (
	"sync"
	"unsafe"
)

// SI_Notification takes no arguments and returns a status int (manual 2.7), so
// it needs no //export callback plumbing. It is resolved separately from the
// main symbol table because it is optional: an older library that lacks it
// should not fail the whole load.
var (
	notifyOnce sync.Once
	notifySym  unsafe.Pointer
)

func resolveNotification() {
	if lib == nil || lib.handle == nil {
		return
	}
	for _, name := range []string{"SI_Notification", "Notification"} {
		cn := C.CString(name)
		C.dlerror()
		sym := C.dlsym(lib.handle, cn)
		C.free(unsafe.Pointer(cn))
		if sym != nil {
			notifySym = unsafe.Pointer(sym)
			return
		}
	}
}

func hasNotification() bool {
	notifyOnce.Do(resolveNotification)
	return notifySym != nil
}

func libNotification() int32 {
	return int32(C.si_call_notify(notifySym))
}
