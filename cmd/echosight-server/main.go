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
//	echosight-server --log-file /var/log/echosight.log
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

	"github.com/joho/godotenv"

	"echosight/internal/api"
	"echosight/internal/compute"
	"echosight/internal/device"
	"echosight/internal/state"
	"echosight/internal/wire"
)

// env reads a value with a fallback, for flag defaults.
//
// Precedence is flag > environment > built-in default, which is the same
// ordering fmcwriter and the compute server use. The .env file is for the
// values that differ between machines; a flag is for a one-off run.
func env(key, fallback string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return fallback
}

func main() {
	// Load .env before flags so it can supply their defaults. Missing is
	// normal under systemd, where EnvironmentFile does the same job.
	if err := godotenv.Load(); err != nil {
		fmt.Fprintln(os.Stderr, "no .env file, using the environment")
	}

	var (
		listen     = flag.String("listen", env("LISTEN_ADDR", "0.0.0.0:8975"), "HTTP listen address")
		captureDir = flag.String("captures", env("CAPTURE_DIR", defaultCaptureDir()), "directory for .fmc capture files")
		computeTo  = flag.String("compute", env("COMPUTE_ADDR", ""), "compute server HTTP address, host:port (COMPUTE_ADDR). This machine pulls its configuration from there on boot and on reconnect; empty means no configuration and the board will not arm")
		group      = flag.Int("group", 1, "SI5G group index")
		serial     = flag.Int("serial", 0, "expected board serial; 0 accepts any detected board")
		openTO     = flag.Duration("open-timeout", 20*time.Second, "deadline on the SI5G Open() call")
		autoResume = flag.Bool("auto-resume", true, "restart firing automatically after a reconnect")
		uplinkTo   = flag.String("uplink", env("UPLINK_ADDR", ""), "compute server frame receiver, host:port (UPLINK_ADDR). Empty runs compression and discards output (bench mode)")
		logFile    = flag.String("log-file", "", "also append logs to this file")
		exitOnHang = flag.Bool("exit-on-hang", true, "exit(1) when Open() blows its deadline so a supervisor can restart us")
	)
	flag.Parse()

	store := state.NewStore()
	broker := state.NewBroker()

	log, closeLog := newLogger(store, *logFile)
	defer closeLog()

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	// Pull the configuration for the ACTIVE SESSION from the compute server.
	//
	// This replaces last-config.json. That file existed to resume the
	// operator's working point across a restart, which is the right goal, but
	// it made this machine a second place a configuration could live and the
	// two drifted: a stale file holding points 750 and rxCount 30 produced a
	// diagonal crawl that looked like a live display for a whole session.
	//
	// Note this asks for the CURRENT SESSION, not for a default preset. A
	// reboot mid-scan must restore the geometry the operator chose; pulling a
	// default would silently revert step 1 to step 2 and everything downstream
	// would look healthy while being reconstructed against the wrong aperture.
	//
	// With no compute server, or no active tank, initialCfg stays nil and the
	// supervisor refuses to arm. There is deliberately no fallback: a default
	// this machine could apply on its own is the second source of truth we are
	// removing.
	var (
		initialCfg *wire.Config
		initialGen string
		cc         *compute.Client
	)
	if *computeTo != "" {
		cc = compute.New(*computeTo)
		pullCtx, cancelPull := context.WithTimeout(ctx, 60*time.Second)
		sess, err := cc.PullWithRetry(pullCtx, 5*time.Second, func(err error) {
			if errors.Is(err, compute.ErrNoSession) {
				log.Warn("no active tank on the compute server; waiting", "compute", *computeTo)
			} else {
				log.Warn("cannot reach the compute server; retrying", "compute", *computeTo, "err", err)
			}
		})
		cancelPull()
		if err != nil {
			log.Error("no configuration pulled; the board will not arm until one arrives",
				"compute", *computeTo, "err", err)
		} else {
			initialCfg = &sess.Config
			initialGen = sess.Generation
			log.Info("pulled configuration", "tank", sess.TankID, "generation", sess.Generation)
		}
	} else {
		log.Error("no COMPUTE_ADDR set; this machine has no configuration source and will not arm")
	}

	// Declared ahead of the supervisor because OnApplied closes over them.
	var srv *api.Server
	appliedGen := initialGen

	sup := device.New(device.Options{
		Store:            store,
		Broker:           broker,
		Group:            *group,
		Serial:           *serial,
		OpenTimeout:      *openTO,
		AutoResumeFiring: *autoResume,
		CaptureDir:       *captureDir,
		UplinkAddr:       *uplinkTo,
		InitialConfig: initialCfg,
		// Nothing is written to disk. The generation is recorded in memory and
		// reported in GET /api/state, which is how the compute server notices
		// this process restarted and pushes again. An applied config that this
		// machine cannot name is one the compute server cannot verify.
		OnApplied: func(c wire.Config) {
			if srv != nil {
				srv.SetConfigGeneration(appliedGen)
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

	srv = api.New(sup, store, broker, stop)
	srv.SetConfigGeneration(appliedGen)
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

	// Wait for a signal. There is no TUI: this process runs under systemd on a
	// machine inside a tank, and the operator interface is Tank Sight talking
	// to the compute server. A terminal UI on the robot was something only a
	// person standing next to the robot could read, and by the time the robot
	// is somewhere it matters, nobody is.
	<-ctx.Done()

	shutdownCtx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_ = httpSrv.Shutdown(shutdownCtx)
	sup.Shutdown()
	log.Info("stopped")
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

func newLogger(store *state.Store, path string) (*slog.Logger, func()) {
	var inner slog.Handler
	closer := func() {}

	// Mirror the ENTIRE log ring to stderr rather than attaching a stderr
	// handler to slog.
	//
	// The supervisor logs through store.Logf directly, not slog, so a
	// slog-only stderr sink silently swallows every connection error:
	// "could not load libsi5g.so", "number_of_devices: ...", Open() failures.
	// Those are exactly what an operator under systemd needs to see. Every
	// line already lands in the ring, slog lines via storeHandler and
	// supervisor lines directly, so the mirror is the single stderr path and
	// nothing prints twice.
	store.SetMirror(func(level, msg string) {
		fmt.Fprintf(os.Stderr, "%s %-5s %s\n",
			time.Now().Format("15:04:05.000"), level, msg)
	})

	if path != "" {
		f, err := os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o644)
		if err == nil {
			inner = slog.NewJSONHandler(f, &slog.HandlerOptions{Level: slog.LevelDebug})
			closer = func() { _ = f.Close() }
		}
	}
	return slog.New(storeHandler{inner: inner, store: store}), closer
}

