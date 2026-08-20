// Package device owns the SI5G board: the connection state machine, the
// acquisition loop and the thickness measurement.
package device

import (
	"sync/atomic"
	"context"
	"encoding/json"
	"fmt"
	"sync"
	"time"

	"echosight/internal/config"
	"echosight/internal/state"
	"echosight/internal/wire"
	"echosight/spike"
)

type Options struct {
	Store  *state.Store
	Broker *state.Broker

	Group  int // SI5G group index, 1-based
	Serial int // expected board serial; 0 accepts whatever is detected

	OpenTimeout time.Duration // deadline on Open(), which the manual does not bound
	OnHang      func()        // called when the deadline blows; normally os.Exit(1)

	// AutoResumeFiring restarts acquisition after a reconnect without waiting
	// for an operator. Correct for an unattended robot inside a tank; the ARMED
	// state must stay loud in the UI either way.
	AutoResumeFiring bool

	Simulated bool // dev#1.enable_pattern was requested at startup

	CaptureDir string // where VSA snapshot files are written; "" = ./captures

	// UplinkAddr is host:port of the compute server's frame receiver.
	// Empty runs the compression chain but discards the output, which is
	// the correct bench mode: it exercises and times the DSP without
	// needing the far end to exist.
	UplinkAddr string

	// InitialConfig seeds staged/applied at startup instead of Default().
	// Without it every server restart silently reverts the operator's whole
	// working point — HV off, points 2048, sync 1000 — which reads in the UI
	// as "the board stopped working" and cost real field time.
	InitialConfig *wire.Config

	// OnApplied persists a successfully applied config so the next start
	// can restore it. Called off the supervisor goroutine.
	OnApplied func(wire.Config)
}

type Supervisor struct {
	opt Options

	cmd  chan func()
	once sync.Once

	mu      sync.RWMutex
	staged  wire.Config
	applied wire.Config
	// hasApplied is false until a configuration has been APPLIED to the board.
	//
	// It deliberately does NOT track staging. Staging writes s.staged and
	// leaves s.applied at its zero value, so a flag set by Stage made the
	// reconnect path believe there was something to reapply and it reapplied
	// the zero config: "fmc probe elements and aperture must be > 0", from a
	// configuration nobody ever chose.
	//
	// While false nothing is reapplied and acquisition is refused.
	hasApplied bool

	acq       *acquisition
	wasFiring bool

	openBackoff time.Duration
	nextOpenAt  time.Time
	openAttempt int

	frame     spike.FrameLayout
	fwMode    int // dev#1.mode at last readDeviceInfo; supervisor goroutine only
	numShots  int
	syncMinUs float64

	// needsReopen latches when a read loop failed to drain and was leaked.
	// The board may still have a transfer in flight on a pinned OS thread,
	// so every path that would reconfigure it refuses until Reconnect clears
	// this. Supervisor goroutine only.
	needsReopen bool
	// bundleSeq is persistent across acquisition restarts. Restarting it at
	// 1 per acquisition made the client's gap detector see a huge negative
	// delta on the second start and report ~2^33 "gaps".
	bundleSeq uint32
	// FMC capture machinery (fmc_capture.go): geometry published by the read
	// loop after calibration; an armed capture the loop appends into; the
	// last result.
	fmcGeom atomic.Pointer[fmcGeometry]
	fmcCap  atomic.Pointer[fmcCaptureState]
	fmcLast fmcCaptureResult

	// pipe is the compression + uplink worker (pipeline.go).
	pipe pipeState

	// previewBudgetBytes is the FMC preview traffic budget in bytes/s,
	// adjustable live via SetPreviewBudget (no device interaction, so it is
	// safe during collection). Zero means the default.
	previewBudgetBytes atomic.Int64

}

const (
	backoffMin = 1 * time.Second
	backoffMax = 10 * time.Second
)

func New(opt Options) *Supervisor {
	if opt.Group == 0 {
		opt.Group = 1
	}
	if opt.OpenTimeout == 0 {
		opt.OpenTimeout = 20 * time.Second
	}
	// NO FALLBACK to config.Default().
	//
	// Default() used to seed this, and that was correct when the machine
	// persisted its own configuration. It is not now: a board that comes up
	// running a configuration nobody selected is exactly the failure the pull
	// design removes, and it is invisible — the board streams, the UI looks
	// healthy, and the geometry is whatever the built-in default happened to
	// be. So with no config the supervisor opens the device, reports what it
	// found, and REFUSES TO ARM until one arrives.
	var c wire.Config
	// An initial config has been PULLED but not yet applied. It seeds staged
	// so the first apply has something to write, and leaves applied zero,
	// which is the truth: nothing is on the board yet.
	if opt.InitialConfig != nil {
		c = *opt.InitialConfig
	}
	s := &Supervisor{
		opt:         opt,
		cmd:         make(chan func(), 16),
		staged:      c,
		applied:     c,
		openBackoff: backoffMin,
	}
	s.opt.Store.Update(func(sn *state.Snapshot) { sn.Status.Simulated = opt.Simulated })
	s.pipeInit()
	return s
}

func (s *Supervisor) logf(level, format string, a ...any) {
	s.opt.Store.Logf(level, fmt.Sprintf(format, a...))
}

// do runs fn on the supervisor goroutine so every compound device operation is
// serialized against the state machine.
func (s *Supervisor) do(ctx context.Context, fn func() error) error {
	done := make(chan error, 1)
	select {
	case s.cmd <- func() { done <- fn() }:
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(5 * time.Second):
		return fmt.Errorf("supervisor busy")
	}
	select {
	case err := <-done:
		return err
	case <-ctx.Done():
		return ctx.Err()
	}
}

// Run drives the state machine until ctx is cancelled.
func (s *Supervisor) Run(ctx context.Context) {
	tick := time.NewTicker(500 * time.Millisecond)
	defer tick.Stop()
	status := time.NewTicker(1 * time.Second)
	defer status.Stop()

	for {
		select {
		case <-ctx.Done():
			s.teardown()
			return
		case fn := <-s.cmd:
			fn()
		case <-tick.C:
			s.step(ctx)
		case <-status.C:
			s.publishStatus()
		}
	}
}

func (s *Supervisor) phase() state.Phase { return s.opt.Store.Snapshot().Phase }

func (s *Supervisor) setErr(format string, a ...any) {
	msg := fmt.Sprintf(format, a...)
	s.opt.Store.Update(func(sn *state.Snapshot) { sn.LastError = msg })
	s.logf("error", "%s", msg)
}

func (s *Supervisor) step(ctx context.Context) {
	switch s.phase() {
	case state.PhaseAbsent:
		s.stepAbsent()
	case state.PhaseOpening:
		s.stepOpening()
	case state.PhaseConfiguring:
		s.stepConfiguring()
	case state.PhaseRunning:
		s.stepRunning()
	case state.PhaseDegraded:
		s.stepDegraded()
	case state.PhaseClosing:
		s.stepClosing()
	case state.PhaseHung:
		// Nothing recovers this in-process. OnHang has already fired.
	}
}

// stepAbsent polls for physical presence without opening anything.
//
// number_of_devices read before Open returns the count of DETECTED devices
// (manual 2.8.9.2), so this costs one library call and cannot wedge the USB
// controller. Open() by contrast downloads firmware and has no documented
// timeout, which is exactly the call that has hung on this hardware before.
func (s *Supervisor) stepAbsent() {
	if !spike.Loaded() {
		if err := spike.Load(); err != nil {
			s.setErr("%s", spike.Describe(err))
			return
		}
		s.opt.Store.Update(func(sn *state.Snapshot) { sn.LibPath = spike.LibPath() })
		s.logf("info", "loaded %s", spike.LibPath())
	}

	n, err := spike.DeviceCount()
	if err != nil {
		s.setErr("number_of_devices: %v", err)
		return
	}
	s.opt.Store.Update(func(sn *state.Snapshot) { sn.Detected = n })
	if n < 1 {
		return
	}

	if s.opt.Serial != 0 {
		serials, err := spike.DeviceSerials(n)
		if err == nil {
			found := false
			for _, ser := range serials {
				if int(ser) == s.opt.Serial {
					found = true
				}
			}
			if !found {
				s.setErr("detected %d board(s) but none with serial %d", n, s.opt.Serial)
				return
			}
		}
	}

	if time.Now().Before(s.nextOpenAt) {
		return
	}
	s.opt.Store.SetPhase(state.PhaseOpening)
}

func (s *Supervisor) stepOpening() {
	s.logf("info", "opening device (attempt %d)", s.openAttempt+1)

	done := make(chan error, 1)
	go func() { done <- spike.Open() }()

	select {
	case err := <-done:
		if err != nil && spike.IsAlreadyOpen(err) {
			// A previous process left a stale registration in the library's
			// internal application database. Close ours and retry once.
			s.logf("warn", "access already open, clearing stale registration")
			_ = spike.Close()
			err = spike.Open()
		}
		if err != nil {
			s.setErr("%s", spike.Describe(err))
			s.failOpen()
			return
		}
	case <-time.After(s.opt.OpenTimeout):
		// The goroutine is stuck inside a blocking cgo call. It cannot be
		// cancelled and its OS thread is gone for the life of the process.
		s.opt.Store.SetPhase(state.PhaseHung)
		s.setErr("Open() exceeded %s; the USB controller is wedged and the process cannot recover in place", s.opt.OpenTimeout)
		if s.opt.OnHang != nil {
			s.opt.OnHang()
		}
		return
	}

	s.openAttempt = 0
	s.openBackoff = backoffMin
	s.readDeviceInfo()
	s.opt.Store.Update(func(sn *state.Snapshot) {
		sn.State.DeviceOpen = true
		sn.Reconnects++
	})
	s.logf("info", "device open")
	s.opt.Store.SetPhase(state.PhaseConfiguring)
}

func (s *Supervisor) failOpen() {
	s.openAttempt++
	s.openBackoff *= 2
	if s.openBackoff > backoffMax {
		s.openBackoff = backoffMax
	}
	s.nextOpenAt = time.Now().Add(s.openBackoff)
	s.opt.Store.SetPhase(state.PhaseAbsent)
}

// stepConfiguring replays the whole applied config. Everything on the board is
// back at defaults after a power cycle, so a partial reapply is not an option.
func (s *Supervisor) stepConfiguring() {
	s.mu.RLock()
	cfg := s.applied
	have := s.hasApplied
	s.mu.RUnlock()

	// With no configuration there is nothing to write. The device stays open
	// and interrogable so the operator can see the serial, the firmware mode
	// and any fault, which is most of what they need while they choose a tank.
	if !have {
		s.logf("warn", "device open but UNCONFIGURED: waiting for a configuration from the compute server")
		s.opt.Store.SetPhase(state.PhaseRunning)
		return
	}

	if _, err := s.applyLocked(cfg); err != nil {
		s.setErr("reapply after reconnect failed: %v", err)
		if spike.IsDeviceGone(err) {
			s.opt.Store.SetPhase(state.PhaseDegraded)
			return
		}
		// A -114 here means the saved config is invalid against whatever
		// firmware came up. Stay open so the operator can see and fix it.
	}
	s.opt.Store.SetPhase(state.PhaseRunning)

	if s.wasFiring && s.opt.AutoResumeFiring {
		s.logf("warn", "auto-resuming acquisition after reconnect")
		if err := s.startAcqLocked("continuous", 0); err != nil {
			s.setErr("auto-resume failed: %v", err)
		}
	}
}

// quietDevice reports whether background device interrogation must be
// suppressed. During FMC collection the manual's transfer method (3.5) owns
// the link: reads block until a packet fills, and ANY interleaved parameter
// query — temperature, available_data, inputs, notifications, device_info —
// wedges the readout engine (dev#1.data then returns nbBytes=0 forever). This
// was the last difference between the server and the vendor-faithful
// fmcprobe, which polls nothing and never wedges. PA acquisition is
// unaffected: its documented flow is poll-then-read.
func (s *Supervisor) quietDevice() bool {
	return s.acq != nil && (s.fwMode == spike.ModeFMC || s.fwMode == spike.ModeFMCTDM)
}

// errQuiet is returned by every operator-facing operation that would talk to
// the board mid-FMC-collection. Under the manual 3.5 transfer method the
// stream owns the link: one interleaved parameter query or naked dev#1.data
// read leaves the readout engine returning nbBytes=0 until the device is
// reopened. Refusing with an explanation beats silently killing an
// inspection run — stop acquisition first, then retry.
func errQuiet(op string) error {
	return fmt.Errorf("%s is blocked during FMC collection: the transfer method owns the link "+
		"(any interleaved device call wedges the stream). Stop acquisition first", op)
}

// stepRunning is the health probe. temperature_status is a cheap read that also
// feeds the status strip, so it does double duty.
func (s *Supervisor) stepRunning() {
	if s.quietDevice() {
		// Streaming FMC: report the last known health rather than querying.
		return
	}
	temp, err := spike.GetDbl("dev#1.temperature_status", spike.UnitCelsius)
	if err != nil {
		if spike.IsDeviceGone(err) {
			s.setErr("device lost: %v", err)
			s.opt.Store.SetPhase(state.PhaseDegraded)
			return
		}
		s.setErr("temperature_status: %v", err)
		return
	}

	backlog, _ := spike.GetInt("dev#1.available_data", spike.UnitNone)
	inputs, _ := spike.GetInt("dev#1.inputs", spike.UnitNone)

	// The library is shared: Synapse or a second echosight can be driving the
	// same board. A notification of 3 means somebody else moved a parameter and
	// the staged config no longer describes the hardware.
	if n, err := spike.Notification(); err == nil && n == spike.NotifyParamFromOther {
		s.logf("warn", "another application changed a parameter; staged config may be stale")
	}

	s.opt.Store.Update(func(sn *state.Snapshot) {
		sn.Status.TempC = temp
		sn.Status.BacklogWords = backlog
		sn.State.IoInputs = inputs
		sn.State.SyncPeriodMinUs = s.syncMinUs
		sn.LastError = ""
	})
}

func (s *Supervisor) stepDegraded() {
	s.stopAcqLocked()
	s.opt.Store.SetPhase(state.PhaseClosing)
}

// stepClosing brings the board down safely. Close itself only unregisters this
// application and frees its resources (manual 2.3), so it is cheap and cannot
// block on vanished hardware.
func (s *Supervisor) stepClosing() {
	_, _, _ = spike.SetInt("dev#1.enable_high_voltage", spike.UnitNone, 0)
	_, _, _ = spike.SetInt("dev#1.enable_prf", spike.UnitNone, 0)
	_, _, _ = spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 0)
	if err := spike.Close(); err != nil {
		s.logf("warn", "Close: %v", err)
	}
	s.opt.Store.Update(func(sn *state.Snapshot) {
		sn.State.DeviceOpen = false
		sn.Status.Acq = "idle"
		sn.Status.HvOn = false
	})
	s.nextOpenAt = time.Now().Add(s.openBackoff)
	s.opt.Store.SetPhase(state.PhaseAbsent)
	s.logf("info", "device closed, retrying in %s", s.openBackoff)
}

func (s *Supervisor) teardown() {
	s.once.Do(func() {
		s.stopAcqLocked()
		if s.opt.Store.Snapshot().State.DeviceOpen {
			_, _, _ = spike.SetInt("dev#1.enable_high_voltage", spike.UnitNone, 0)
			_, _, _ = spike.SetInt("dev#1.enable_prf", spike.UnitNone, 0)
			_, _, _ = spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 0)
			_ = spike.Close()
		}
		s.logf("info", "shutdown complete")
	})
}

func (s *Supervisor) readDeviceInfo() {
	if s.quietDevice() {
		return // see quietDevice: no interrogation during FMC collection
	}
	info, _ := spike.GetStr("dev#1.device_info", spike.UnitNone)
	libv, _ := spike.GetStr("library_version", spike.UnitNone)
	serial, _ := spike.GetInt("dev#1.serial_number", spike.UnitNone)
	mode, _ := spike.GetInt("dev#1.mode", spike.UnitNone)
	usb3, _ := spike.GetInt("dev#1.is_usb3", spike.UnitNone)
	eth, _ := spike.GetInt("dev#1.is_ethernet", spike.UnitNone)

	model := ""
	var parsed struct {
		Model      string `json:"Model"`
		Connection string `json:"Connection"`
	}
	if json.Unmarshal([]byte(info), &parsed) == nil {
		model = parsed.Model
	}

	conn := "usb2"
	switch {
	case eth == 1:
		conn = "ethernet"
	case usb3 == 1:
		conn = "usb3"
	}
	modeStr := map[int]string{spike.ModePA: "PA", spike.ModeFMC: "FMC", spike.ModeFMCTDM: "FMC_TDM"}[mode]
	// Cached for the acquisition branch: FMC firmware frames have a different
	// layout and read path than PA frames. Written and read only on the
	// supervisor goroutine.
	s.fwMode = mode
	fwOK := mode == spike.ModeFMC || mode == spike.ModeFMCTDM
	if !fwOK {
		// Not recoverable from here: dev#1.mode is never switched at runtime
		// (invariant #1), and an FMC scan under PA firmware silently
		// BEAMFORMS — plausible images, wrong numbers. Surface it so the app
		// can block rather than let an operator inspect a plate with it.
		s.logf("error", "board booted in %s firmware; this server is FMC-only. Fix Mode in config-devices.ini and power-cycle", modeStr)
	}

	s.opt.Store.Update(func(sn *state.Snapshot) {
		sn.State.FirmwareOK = fwOK
		sn.Device = wire.DeviceInfo{
			Model:          model,
			Serial:         fmt.Sprintf("%d", serial),
			Connection:     conn,
			FirmwareMode:   modeStr,
			LibraryVersion: libv,
			DeviceInfo:     info,
		}
	})
}

func (s *Supervisor) publishStatus() {
	sn := s.opt.Store.Snapshot()
	st := sn.Status
	st.UptimeS = int(time.Since(sn.StartedAt).Seconds())
	s.opt.Store.Update(func(x *state.Snapshot) { x.Status.UptimeS = st.UptimeS })
	s.opt.Broker.PublishJSON(wire.StatusMsg{T: "status", AppStatus: st})
}

// ---------------------------------------------------------------------------
// Public API used by the HTTP layer
// ---------------------------------------------------------------------------

func (s *Supervisor) State() wire.AppState {
	sn := s.opt.Store.Snapshot()
	st := sn.State
	st.Frame = wire.FrameLayoutInfo{
		Metadata:  s.frame.Metadata,
		Header:    s.frame.Header,
		Ascan:     s.frame.Ascan,
		Footer:    s.frame.Footer,
		SizeWords: s.frame.FrameSize,
	}
	status := sn.Status
	st.Status = &status
	st.Phase = string(sn.Phase)
	st.LastError = sn.LastError
	st.Detected = sn.Detected
	st.LibPath = sn.LibPath
	return st
}

func (s *Supervisor) DeviceInfo() wire.DeviceInfo { return s.opt.Store.Snapshot().Device }

func (s *Supervisor) Config() wire.Config {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.staged
}

// ConfigEnvelope returns both halves and whether they differ. The dirty flag is
// computed the same way Apply decides whether to touch the board at all, so the
// UI and the device can never disagree about it.
func (s *Supervisor) ConfigEnvelope() wire.ConfigEnvelope {
	s.mu.RLock()
	staged, applied := s.staged, s.applied
	s.mu.RUnlock()
	same, err := configEqual(staged, applied)
	return wire.ConfigEnvelope{Staged: staged, Applied: applied, Dirty: err != nil || !same}
}

// Stage validates and stores a config without touching the device.
func (s *Supervisor) Stage(c wire.Config) wire.ValidateResult {
	c.V = wire.ConfigVersion
	res := config.Validate(c)
	if res.Valid {
		s.mu.Lock()
		s.staged = c
		s.mu.Unlock()
	}
	return res
}

// Script returns the firing scenario JSON for the staged config.
func (s *Supervisor) Script() (wire.ScriptResult, error) {
	s.mu.RLock()
	cfg := s.staged
	s.mu.RUnlock()
	js, err := config.ScriptJSON(cfg)
	if err != nil {
		return wire.ScriptResult{}, err
	}
	return wire.ScriptResult{JSON: js, SequenceMap: config.SequenceMap(cfg)}, nil
}

// ValidateScript round-trips a scenario through the device when it is open,
// which is the only way to get the library's own -113 / -115 verdict.
func (s *Supervisor) ValidateScript(ctx context.Context, js string) wire.ValidateResult {
	res := config.ValidateScriptJSON(js)
	if !res.Valid || !s.opt.Store.Snapshot().State.DeviceOpen {
		return res
	}
	_ = s.do(ctx, func() error {
		if s.acq != nil {
			return nil // never disturb a live scan just to lint a script
		}
		if err := spike.SetStr("dev#1.settings", spike.UnitNone, js); err != nil {
			res.Valid = false
			res.Problems = append(res.Problems, wire.Problem{Path: "device", Msg: err.Error()})
		}
		// Put the real scenario back.
		s.mu.RLock()
		cfg := s.applied
		s.mu.RUnlock()
		if good, err := config.ScriptJSON(cfg); err == nil {
			_ = spike.SetStr("dev#1.settings", spike.UnitNone, good)
		}
		return nil
	})
	return res
}
// Flush drains whatever the board still holds after firing stopped (2.8.6.12).
func (s *Supervisor) Flush(ctx context.Context) (int, error) {
	var drained int
	err := s.do(ctx, func() error {
		if s.quietDevice() {
			return errQuiet("flush")
		}
		remaining, err := spike.GetInt("dev#1.flush_data", spike.UnitNone)
		if err != nil {
			return err
		}
		for remaining > 0 {
			avail, err := spike.GetInt("dev#1.available_data", spike.UnitNone)
			if err != nil || avail <= 0 {
				break
			}
			buf := make([]uint16, avail)
			n, err := spike.GetData("dev#1.data", avail, buf)
			if err != nil {
				return err
			}
			drained += n
			remaining, err = spike.GetInt("dev#1.flush_data", spike.UnitNone)
			if err != nil {
				break
			}
		}
		return nil
	})
	return drained, err
}

// SetMode is intentionally NOT a runtime switch any more. The board's mode
// is set at boot from config-devices.ini and left alone: every wedged FMC
// session in the field followed a runtime dev#1.mode switch (the readout
// engine returns nbBytes=0 forever afterwards), while the same parameters,
// scenario and collection method run indefinitely on a board that BOOTED in
// FMC. The library reports the switch as successful either way, so nothing
// on the host can detect or repair it — the only reliable fix is to not
// switch. Raw dev#1.mode remains reachable through the expert console for
// deliberate experiments.
func (s *Supervisor) SetMode(ctx context.Context, mode string) error {
	cur := s.opt.Store.Snapshot().Device.FirmwareMode
	if mode == cur {
		return nil
	}
	return fmt.Errorf("board mode is set at boot, not at runtime: edit Mode in config-devices.ini "+
		"(currently %s, wanted %s) and restart the board and this server", cur, mode)
}

// Expert is the raw parameter console behind /api/expert.
func (s *Supervisor) Expert(ctx context.Context, op, param string, unit int, value any) (any, bool, error) {
	var out any
	var clipped bool
	err := s.do(ctx, func() error {
		if s.quietDevice() {
			return errQuiet("the expert console")
		}
		switch op {
		case "getInt":
			v, err := spike.GetInt(param, unit)
			out = v
			return err
		case "getDbl":
			v, err := spike.GetDbl(param, unit)
			out = v
			return err
		case "getStr":
			v, err := spike.GetStr(param, unit)
			out = v
			return err
		case "setInt":
			f, ok := value.(float64)
			if !ok {
				return fmt.Errorf("setInt needs a numeric value")
			}
			v, c, err := spike.SetInt(param, unit, int(f))
			out, clipped = v, c
			return err
		case "setDbl":
			f, ok := value.(float64)
			if !ok {
				return fmt.Errorf("setDbl needs a numeric value")
			}
			v, c, err := spike.SetDbl(param, unit, f)
			out, clipped = v, c
			return err
		case "setStr":
			str, ok := value.(string)
			if !ok {
				return fmt.Errorf("setStr needs a string value")
			}
			return spike.SetStr(param, unit, str)
		case "exec":
			return spike.ExecSync(param)
		default:
			return fmt.Errorf("unknown op %q", op)
		}
	})
	return out, clipped, err
}

// CalibrateVelocity solves for the material velocity that makes the currently
// measured echo spacing equal a known thickness.
func (s *Supervisor) CalibrateVelocity(trueMm float64) (float64, error) {
	return 0, fmt.Errorf("velocity calibration lives on the reconstruction server: the SBC no longer measures thickness (no TFM, no gate engine), so it has nothing to calibrate against. Set Processing.VelocityMps through the normal config path")
}

// bumpGeom marks the acquisition geometry as changed. The compression worker
// re-reads geometry into its frame envelope on the next cycle, so downstream
// reconstruction always sees the geometry the samples were taken under.
func (s *Supervisor) bumpGeom() { s.pipe.geomGen.Add(1) }

// Reconnect forces a full close and reopen.
func (s *Supervisor) Reconnect(ctx context.Context) error {
	// Close/reopen is the only known-good recovery from a leaked read loop.
	s.needsReopen = false
	s.opt.Store.Update(func(sn *state.Snapshot) { sn.State.NeedsReopen = false })
	return s.do(ctx, func() error {
		s.openBackoff = backoffMin
		s.nextOpenAt = time.Time{}
		if s.opt.Store.Snapshot().State.DeviceOpen {
			s.opt.Store.SetPhase(state.PhaseDegraded)
		} else {
			s.opt.Store.SetPhase(state.PhaseAbsent)
		}
		return nil
	})
}

func (s *Supervisor) Shutdown() {
	s.pipeShutdown()
	s.teardown()
}

func (s *Supervisor) SequenceCount() int { return s.numShots }

// PreviewBudgetBytes reports the live FMC preview budget in bytes/s.
func (s *Supervisor) PreviewBudgetBytes() int {
	if v := s.previewBudgetBytes.Load(); v > 0 {
		return int(v)
	}
	return fmcPreviewBudgetBytes
}

// SetPreviewBudget adjusts the preview budget (MB/s), clamped to 0.5..64.
// Takes effect within a second; never touches the device.
func (s *Supervisor) SetPreviewBudget(mbs float64) float64 {
	if mbs < 0.5 {
		mbs = 0.5
	}
	if mbs > 64 {
		mbs = 64
	}
	s.previewBudgetBytes.Store(int64(mbs * (1 << 20)))
	return mbs
}
