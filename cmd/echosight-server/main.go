// Command echosight-sbc runs FMC acquisition, compression and uplink on the
// robot SBC (PICO-RAP4). It owns the SI5G board, compresses every acquired
// cycle, and streams the result to the compute server over the fibre
// umbilical. It does NOT reconstruct: TFM, measurement and imaging all live
// on the compute server now, so nothing here interprets the samples.
//
// The REST + WebSocket surface remains for control, configuration and bench
// diagnostics. It is not the data path.
//
//	echosight-server --listen 0.0.0.0:8975
//	echosight-server --headless --log-file /var/log/echosight.log
//
// Binding to 0.0.0.0 is the default because the client is never on this
// machine. Binding to loopback would make the app unreachable, which is the
// first thing to check when the connect screen sits on "unreachable".
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"echosight/internal/api"
	"echosight/internal/config"
	"echosight/internal/device"
	"echosight/internal/pose"
	"echosight/internal/presets"
	"echosight/internal/state"
	"echosight/internal/tui"
	"echosight/internal/wire"
)

func main() {
	var (
		listen     = flag.String("listen", "0.0.0.0:8975", "HTTP listen address")
		presetDir  = flag.String("presets", defaultPresetDir(), "directory for saved configurations")
		captureDir = flag.String("captures", defaultCaptureDir(), "directory for .fmc capture files and last-config.json")
		group      = flag.Int("group", 1, "SI5G group index")
		serial     = flag.Int("serial", 0, "expected board serial; 0 accepts any detected board")
		poseAddr   = flag.String("pose", "", "UDP address to receive robot pose on, e.g. 127.0.0.1:9100")
		openTO     = flag.Duration("open-timeout", 20*time.Second, "deadline on the SI5G Open() call")
		autoResume = flag.Bool("auto-resume", true, "restart firing automatically after a reconnect")
		headless   = flag.Bool("headless", false, "no TUI; log to stderr (use under systemd)")
		uplinkTo   = flag.String("uplink", "", "compute server frame receiver, host:port. Empty runs compression and discards output (bench mode)")
		logFile    = flag.String("log-file", "", "also append logs to this file")
		exitOnHang = flag.Bool("exit-on-hang", true, "exit(1) when Open() blows its deadline so a supervisor can restart us")
	)
	flag.Parse()

	store := state.NewStore()
	broker := state.NewBroker()

	log, closeLog := newLogger(store, *headless, *logFile)
	defer closeLog()

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	ps, err := presets.New(*presetDir)
	if err != nil {
		fmt.Fprintf(os.Stderr, "preset directory %s: %v\n", *presetDir, err)
		os.Exit(1)
	}

	var poseSrc *pose.Source
	if *poseAddr != "" {
		poseSrc, err = pose.NewUDP(*poseAddr)
		if err != nil {
			fmt.Fprintf(os.Stderr, "pose listener %s: %v\n", *poseAddr, err)
			os.Exit(1)
		}
		go poseSrc.Run(ctx)
		store.Update(func(sn *state.Snapshot) { sn.PoseSource = *poseAddr })
		log.Info("pose source listening", "addr", *poseAddr)
	}

	// Restore the last applied config so a restart resumes the operator's
	// working point (HV, gains, geometry) instead of silently reverting to
	// factory defaults.
	lastCfgPath := filepath.Join(*captureDir, "last-config.json")
	var initialCfg *wire.Config
	if c, err := config.Load(lastCfgPath); err == nil {
		initialCfg = &c
		log.Info("restored last applied config", "path", lastCfgPath)
	} else if ver := (*config.ErrWrongVersion)(nil); errors.As(err, &ver) {
		// Deliberately NOT migrated. Every pre-v2 file describes a
		// phased-array scan this server cannot run, and Default() is now the
		// working FMC point — booting to it beats booting to a translation
		// nobody has ever run against the board. The old file is kept as
		// .v1.bak, not deleted.
		log.Warn("ignoring pre-FMC config; booting to FMC defaults", "err", err)
	} else if !os.IsNotExist(err) {
		log.Warn("could not read last config; booting to FMC defaults", "err", err)
	}

	sup := device.New(device.Options{
		Store:            store,
		Broker:           broker,
		Pose:             poseSrc,
		Group:            *group,
		Serial:           *serial,
		OpenTimeout:      *openTO,
		AutoResumeFiring: *autoResume,
		CaptureDir:       *captureDir,
		UplinkAddr:       *uplinkTo,
		InitialConfig:    initialCfg,
		OnApplied: func(c wire.Config) {
			if err := config.Save(lastCfgPath, c); err != nil {
				log.Warn("could not persist last config", "err", err)
			}
		},
		OnHang: func() {
			if !*exitOnHang {
				return
			}
			// The Open() goroutine is stuck in a blocking cgo call: it cannot
			// be cancelled and its OS thread is gone. Process restart is the
			// only recovery, so hand that job to systemd.
			log.Error("Open() deadline exceeded, exiting for supervisor restart")
			closeLog()
			os.Exit(1)
		},
	})
	go sup.Run(ctx)

	srv := api.New(sup, store, broker, ps, stop)
	httpSrv := &http.Server{
		Addr:              *listen,
		Handler:           srv.Handler(),
		ReadHeaderTimeout: 10 * time.Second,
		// No WriteTimeout: the WebSocket is a long-lived connection and any
		// value here would guillotine it mid-scan.
	}

	go func() {
		log.Info("listening", "addr", *listen)
		store.Logf("info", "listening on "+*listen)
		if err := httpSrv.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
			log.Error("http server stopped", "err", err)
			store.Logf("error", "http server stopped: "+err.Error())
			stop()
		}
	}()

	if !*headless {
		bg := context.Background()
		m := tui.New(store, sup, *listen, tui.Actions{
			Reconnect: func() { _ = sup.Reconnect(bg) },
			Start:     func() { _ = sup.StartAcq(bg, "continuous", 0) },
			Stop:      func() { _ = sup.StopAcq(bg) },
		})
		if err := tui.Run(ctx, m); err != nil && !errors.Is(err, context.Canceled) {
			fmt.Fprintln(os.Stderr, "tui:", err)
		}
		// Cancel the app context only AFTER Bubble Tea has returned; the
		// program was created with tea.WithContext(ctx) and cancelling it
		// while the event loop is mid-Update deadlocks it (see tui.go).
		// This also stops the supervisor's step loop before the teardown
		// below closes the device, so the state machine cannot race a
		// re-open against process exit.
		stop()
	} else {
		<-ctx.Done()
	}

	shutdownCtx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_ = httpSrv.Shutdown(shutdownCtx)
	sup.Shutdown()
	log.Info("stopped")
}

func defaultPresetDir() string {
	if d, err := os.UserConfigDir(); err == nil {
		return filepath.Join(d, "echosight", "presets")
	}
	return "./presets"
}

func defaultCaptureDir() string {
	if d, err := os.UserConfigDir(); err == nil {
		return filepath.Join(d, "echosight", "captures")
	}
	return "./captures"
}

// storeHandler mirrors every slog record into the in-memory ring the TUI shows,
// so operators see the same events whether or not they have a terminal.
type storeHandler struct {
	inner slog.Handler // may be nil in TUI mode with no log file
	store *state.Store
}

func (h storeHandler) Enabled(ctx context.Context, l slog.Level) bool { return l >= slog.LevelInfo }

func (h storeHandler) WithAttrs(as []slog.Attr) slog.Handler {
	if h.inner != nil {
		h.inner = h.inner.WithAttrs(as)
	}
	return h
}

func (h storeHandler) WithGroup(name string) slog.Handler {
	if h.inner != nil {
		h.inner = h.inner.WithGroup(name)
	}
	return h
}

func (h storeHandler) Handle(ctx context.Context, r slog.Record) error {
	level := "info"
	switch {
	case r.Level >= slog.LevelError:
		level = "error"
	case r.Level >= slog.LevelWarn:
		level = "warn"
	}
	msg := r.Message
	r.Attrs(func(a slog.Attr) bool {
		msg += " " + a.Key + "=" + a.Value.String()
		return true
	})
	h.store.Logf(level, msg)
	if h.inner == nil {
		return nil
	}
	return h.inner.Handle(ctx, r)
}

func newLogger(store *state.Store, headless bool, path string) (*slog.Logger, func()) {
	var inner slog.Handler
	closer := func() {}

	// Bubble Tea owns the terminal. Writing to stderr underneath it corrupts
	// the frame, so in TUI mode the only sinks are the ring and the log file.
	//
	// In headless mode, mirror the ENTIRE log ring to stderr rather than
	// attaching a stderr handler to slog. The supervisor logs through
	// store.Logf directly (not slog), so a slog-only stderr sink silently
	// swallows every connection error — "could not load libsi5g.so",
	// "number_of_devices: ...", Open() failures — which is exactly what an
	// operator under systemd needs to see. Every line already lands in the
	// ring (slog lines via storeHandler, supervisor lines directly), so the
	// mirror is the single stderr path and nothing prints twice.
	if headless {
		store.SetMirror(func(level, msg string) {
			fmt.Fprintf(os.Stderr, "%s %-5s %s\n",
				time.Now().Format("15:04:05.000"), level, msg)
		})
	}
	if path != "" {
		f, err := os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o644)
		if err == nil {
			fh := slog.NewJSONHandler(f, &slog.HandlerOptions{Level: slog.LevelDebug})
			if inner == nil {
				inner = fh
			} else {
				inner = multiHandler{inner, fh}
			}
			closer = func() { _ = f.Close() }
		}
	}
	return slog.New(storeHandler{inner: inner, store: store}), closer
}

type multiHandler []slog.Handler

func (m multiHandler) Enabled(ctx context.Context, l slog.Level) bool {
	for _, h := range m {
		if h.Enabled(ctx, l) {
			return true
		}
	}
	return false
}

func (m multiHandler) Handle(ctx context.Context, r slog.Record) error {
	for _, h := range m {
		if h.Enabled(ctx, r.Level) {
			_ = h.Handle(ctx, r.Clone())
		}
	}
	return nil
}

func (m multiHandler) WithAttrs(as []slog.Attr) slog.Handler {
	out := make(multiHandler, len(m))
	for i, h := range m {
		out[i] = h.WithAttrs(as)
	}
	return out
}

func (m multiHandler) WithGroup(name string) slog.Handler {
	out := make(multiHandler, len(m))
	for i, h := range m {
		out[i] = h.WithGroup(name)
	}
	return out
}
