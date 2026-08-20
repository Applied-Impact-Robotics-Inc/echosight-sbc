// Package api exposes the REST and WebSocket contract the Electron app talks to.
// Route names and JSON shapes must match src/renderer/src/api/client.ts exactly.
package api

import (
	"strings"
	"sync/atomic"
	"os"
	"encoding/base64"
	"bytes"
	"path/filepath"
	"encoding/json"
	"fmt"
	"net/http"
	"time"

	"echosight/internal/device"
	"echosight/internal/state"
	"echosight/internal/wire"
)

type Server struct {
	sup    *device.Supervisor
	store  *state.Store
	broker *state.Broker
	onQuit func()

	// configGen is the generation hash of the configuration currently applied
	// to the board, as handed down by the compute server.
	//
	// It is EMPTY until something is applied, and that empty value is load
	// bearing: it is what a freshly restarted SBC reports, and it is how the
	// compute server notices it needs to push again. Without it a rebooted
	// board sits unarmed while every readout upstream looks healthy.
	configGen atomic.Value // string
}

func New(sup *device.Supervisor, store *state.Store, broker *state.Broker, onQuit func()) *Server {
	s := &Server{sup: sup, store: store, broker: broker, onQuit: onQuit}
	s.configGen.Store("")
	return s
}

// SetConfigGeneration records what the board is now running. Called from the
// supervisor's OnApplied hook.
func (s *Server) SetConfigGeneration(gen string) { s.configGen.Store(gen) }

// ConfigGeneration is what the compute server compares against.
func (s *Server) ConfigGeneration() string {
	v, _ := s.configGen.Load().(string)
	return v
}

func statusEnvelope(s wire.AppStatus) wire.StatusMsg {
	return wire.StatusMsg{T: "status", AppStatus: s}
}

func (s *Server) Handler() http.Handler {
	mux := http.NewServeMux()

	mux.HandleFunc("GET /api/state", s.getState)
	mux.HandleFunc("GET /api/device", s.getDevice)
	mux.HandleFunc("POST /api/shutdown", s.shutdown)

	mux.HandleFunc("GET /api/config", s.getConfig)
	mux.HandleFunc("PUT /api/config", s.putConfig)
	mux.HandleFunc("POST /api/config/apply", s.applyConfig)
	mux.HandleFunc("GET /api/config/script", s.getScript)
	mux.HandleFunc("POST /api/config/script/validate", s.validateScript)

	mux.HandleFunc("POST /api/acquisition/start", s.startAcq)
	mux.HandleFunc("POST /api/acquisition/stop", s.stopAcq)
	mux.HandleFunc("POST /api/acquisition/flush", s.flush)
	mux.HandleFunc("POST /api/preview", s.setPreview)
	mux.HandleFunc("POST /api/fmc/capture", s.fmcCapture)
	mux.HandleFunc("GET /api/fmc/last", s.fmcLast)
	mux.HandleFunc("GET /api/fmc/last/digest", s.fmcDigest)
	mux.HandleFunc("GET /api/fmc/last/file", s.fmcFile)
	mux.HandleFunc("GET /api/fmc/last/raw", s.fmcRaw)
	mux.HandleFunc("GET /api/fmc/last/matrix", s.fmcMatrix)
	mux.HandleFunc("GET /api/uplink", s.uplinkStats)

	mux.HandleFunc("POST /api/expert", s.expert)
	mux.HandleFunc("POST /api/calibrate/velocity", s.calibrate)


	mux.HandleFunc("/ws", s.handleWS)

	return cors(mux)
}

// cors permits the Electron renderer, whose Origin is literally "null" once the
// app is packaged and loading from file://.
func cors(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		origin := r.Header.Get("Origin")
		if origin == "" {
			origin = "*"
		}
		w.Header().Set("Access-Control-Allow-Origin", origin)
		w.Header().Set("Vary", "Origin")
		w.Header().Set("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
		w.Header().Set("Access-Control-Allow-Headers", "content-type")
		w.Header().Set("Access-Control-Max-Age", "600")
		if r.Method == http.MethodOptions {
			w.WriteHeader(http.StatusNoContent)
			return
		}
		next.ServeHTTP(w, r)
	})
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

func writeJSON(w http.ResponseWriter, code int, v any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(code)
	if v != nil {
		_ = json.NewEncoder(w).Encode(v)
	}
}

// fail renders the error model the frontend expects, so si5g entries can be
// shown verbatim on the offending control.
func fail(w http.ResponseWriter, code int, err error, si ...wire.Si5gError) {
	writeJSON(w, code, wire.APIError{Error: err.Error(), Si5g: si})
}

func readJSON(r *http.Request, v any) error {
	defer r.Body.Close()
	return json.NewDecoder(r.Body).Decode(v)
}

// ---------------------------------------------------------------------------
// device / state
// ---------------------------------------------------------------------------

// getState carries the applied config generation alongside the device state,
// so the compute server answers "is it alive" and "is it running what I think"
// in one call rather than racing two.
func (s *Server) getState(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, 200, struct {
		wire.AppState
		ConfigGen string `json:"configGen"`
	}{s.sup.State(), s.ConfigGeneration()})
}

func (s *Server) getDevice(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, 200, s.sup.DeviceInfo())
}

func (s *Server) shutdown(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, 204, nil)
	go func() {
		time.Sleep(100 * time.Millisecond)
		s.onQuit()
	}()
}

// ---------------------------------------------------------------------------
// config
// ---------------------------------------------------------------------------

// getConfig returns staged AND applied plus a dirty flag. One source of truth
// for "would Apply do anything": the frontend used to diff client-side, and a
// disagreement between its diff and the server's meant an operator pressing
// Apply on a board that was already streaming for no reason at all.
func (s *Server) getConfig(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, 200, s.sup.ConfigEnvelope())
}

func (s *Server) putConfig(w http.ResponseWriter, r *http.Request) {
	var c wire.Config
	if err := readJSON(r, &c); err != nil {
		fail(w, 400, err)
		return
	}
	writeJSON(w, 200, s.sup.Stage(c))
}

func (s *Server) applyConfig(w http.ResponseWriter, r *http.Request) {
	res, err := s.sup.Apply(r.Context())
	if err != nil {
		fail(w, 500, err)
		return
	}
	writeJSON(w, 200, res)
}

func (s *Server) getScript(w http.ResponseWriter, r *http.Request) {
	res, err := s.sup.Script()
	if err != nil {
		fail(w, 400, err)
		return
	}
	writeJSON(w, 200, res)
}

func (s *Server) validateScript(w http.ResponseWriter, r *http.Request) {
	var body struct {
		JSON string `json:"json"`
	}
	if err := readJSON(r, &body); err != nil {
		fail(w, 400, err)
		return
	}
	writeJSON(w, 200, s.sup.ValidateScript(r.Context(), body.JSON))
}

// ---------------------------------------------------------------------------
// acquisition
// ---------------------------------------------------------------------------

func (s *Server) startAcq(w http.ResponseWriter, r *http.Request) {
	var body struct {
		Mode   string `json:"mode"`
		Cycles int    `json:"cycles"`
	}
	if err := readJSON(r, &body); err != nil {
		fail(w, 400, err)
		return
	}
	if body.Mode == "" {
		body.Mode = "continuous"
	}
	if err := s.sup.StartAcq(r.Context(), body.Mode, body.Cycles); err != nil {
		fail(w, 500, err)
		return
	}
	writeJSON(w, 204, nil)
}

func (s *Server) stopAcq(w http.ResponseWriter, r *http.Request) {
	if err := s.sup.StopAcq(r.Context()); err != nil {
		fail(w, 500, err)
		return
	}
	writeJSON(w, 204, nil)
}

func (s *Server) flush(w http.ResponseWriter, r *http.Request) {
	n, err := s.sup.Flush(r.Context())
	if err != nil {
		fail(w, 500, err)
		return
	}
	writeJSON(w, 200, map[string]int{"drainedWords": n})
}

// setPreview adjusts the FMC preview traffic budget. No device interaction:
// safe at any time, including during collection.
func (s *Server) setPreview(w http.ResponseWriter, r *http.Request) {
	var req struct {
		BudgetMBs float64 `json:"budgetMBs"`
	}
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	applied := s.sup.SetPreviewBudget(req.BudgetMBs)
	writeJSON(w, 200, map[string]float64{"budgetMBs": applied})
}

func (s *Server) fmcCapture(w http.ResponseWriter, r *http.Request) {
	var req struct {
		Cycles int `json:"cycles"`
	}
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	if err := s.sup.StartFMCCapture(req.Cycles); err != nil {
		fail(w, 409, err)
		return
	}
	writeJSON(w, 200, s.sup.LastFMCCapture())
}

func (s *Server) fmcLast(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, 200, s.sup.LastFMCCapture())
}

func (s *Server) fmcDigest(w http.ResponseWriter, r *http.Request) {
	txt, err := s.sup.FMCDigest()
	if err != nil {
		fail(w, 409, err)
		return
	}
	w.Header().Set("Content-Type", "text/plain; charset=utf-8")
	_, _ = w.Write([]byte(txt))
}

func (s *Server) fmcFile(w http.ResponseWriter, r *http.Request) {
	path, err := s.sup.FMCFilePath()
	if err != nil {
		fail(w, 409, err)
		return
	}
	w.Header().Set("Content-Disposition", "attachment; filename=\""+filepath.Base(path)+"\"")
	http.ServeFile(w, r, path)
}

// uplinkStats reports how the compression + transmit path is doing. This is
// the SBC's only "output" endpoint now: it produces compressed frames, not
// images, so there is nothing to render or export here.
//
// Dropped is the number that matters. It must stay at zero: a non-zero value
// means the ring overflowed and FMC frames were destroyed, which is silent
// data loss no downstream stage can detect or recover.
func (s *Server) uplinkStats(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, 200, s.sup.UplinkStats())
}

// fmcRaw serves the last capture as a self-describing TEXT file: a banner,
// the JSON meta line from the .fmc, then the raw little-endian int16 samples
// as one base64 line. Exists because moving binaries off the SBC is painful:
// this downloads through the browser and uploads into a chat, and any tool
// can reconstruct the full matrix from it (b64decode -> int16 LE ->
// [cycle][frame][sample][channel] per the meta geometry).
func (s *Server) fmcRaw(w http.ResponseWriter, r *http.Request) {
	path, err := s.sup.FMCFilePath()
	if err != nil {
		fail(w, 409, err)
		return
	}
	b, err := os.ReadFile(path)
	if err != nil {
		fail(w, 500, err)
		return
	}
	nl := bytes.IndexByte(b, '\n')
	if nl < 0 {
		fail(w, 500, fmt.Errorf("malformed capture file"))
		return
	}
	meta, raw := b[:nl], b[nl+1:]
	w.Header().Set("Content-Type", "text/plain; charset=utf-8")
	w.Header().Set("Content-Disposition",
		"attachment; filename=\""+strings.TrimSuffix(filepath.Base(path), ".fmc")+".txt\"")
	_, _ = w.Write([]byte("ECHOSIGHT FMC RAW v1\n"))
	_, _ = w.Write(meta)
	_, _ = w.Write([]byte("\n"))
	enc := base64.NewEncoder(base64.StdEncoding, w)
	_, _ = enc.Write(raw)
	_ = enc.Close()
	_, _ = w.Write([]byte("\n"))
}

// ---------------------------------------------------------------------------
// expert / calibration
// ---------------------------------------------------------------------------

var unitCodes = map[string]int{
	"none": -1, "us": 0, "mm": 1, "in": 2, "ns": 100, "ms": 101, "s": 102,
	"um": 200, "m": 201, "Hz": 300, "kHz": 301, "MHz": 302,
	"mps": 400, "inpus": 401, "deg": 500, "rad": 501,
	"Celsius": 600, "Fahrenheit": 601,
}

func (s *Server) expert(w http.ResponseWriter, r *http.Request) {
	var body struct {
		Op    string `json:"op"`
		Param string `json:"param"`
		Unit  string `json:"unit"`
		Value any    `json:"value"`
	}
	if err := readJSON(r, &body); err != nil {
		fail(w, 400, err)
		return
	}
	unit, ok := unitCodes[body.Unit]
	if !ok {
		unit = -1
	}
	res, clipped, err := s.sup.Expert(r.Context(), body.Op, body.Param, unit, body.Value)
	out := map[string]any{}
	if err != nil {
		out["error"] = err.Error()
	} else {
		out["result"] = res
		if clipped {
			out["clip"] = true
		}
	}
	writeJSON(w, 200, out)
}

func (s *Server) calibrate(w http.ResponseWriter, r *http.Request) {
	var body struct {
		TrueThicknessMm float64 `json:"trueThicknessMm"`
	}
	if err := readJSON(r, &body); err != nil {
		fail(w, 400, err)
		return
	}
	// Always fails on the SBC build: no thickness measurement here.
	v, err := s.sup.CalibrateVelocity(body.TrueThicknessMm)
	if err != nil {
		fail(w, 400, err)
		return
	}
	writeJSON(w, 200, map[string]float64{"velocityMps": v})
}

