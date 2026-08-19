// Package wire holds every type that crosses the network boundary. It is a
// 1:1 mirror of the Electron frontend's src/renderer/src/api/types.ts. Keep the
// two in sync; nothing else in this server defines JSON shapes.
//
// FMC-ONLY. The phased-array (focal-law) era is gone: there are no scan kinds,
// no gates, no DAC, no VSA, no per-sequence gate measurements. The board boots
// in FMC firmware from config-devices.ini and the only imaging path is TFM.
// Nothing here is backwards compatible with a v1 config; see config.Load.
package wire

// ConfigVersion is written into every persisted config and preset. A file
// carrying any other value is refused outright rather than migrated: every v1
// file is a phased-array config, and Default() now boots to a working FMC
// setup, which is strictly better than anything a migration could produce.
const ConfigVersion = 2

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

type DeviceConfig struct {
	HighVoltage   int  `json:"highVoltage"` // 10..50 V   dev#1.high_voltage        LOCK
	HVEnable      bool `json:"hvEnable"`    //            dev#1.enable_high_voltage
	Preamp        int  `json:"preamp"`      // 0..2 G1/G2/G3  dev#1.receiver_preampli  LOCK
	AnalogFilters struct {
		HP int `json:"hp"` // 0:300kHz 1:600kHz
		LP int `json:"lp"` // 0:10MHz 1:15MHz 2:25MHz
	} `json:"analogFilters"` // dev#1.receiver_analog_filters  LOCK
	DeviceName string `json:"deviceName"` // dev#1.device_name
}

type IoConfig struct {
	IO5V    bool `json:"io5v"`    // dev#1.IO_5V
	Output3 bool `json:"output3"` // dev#1.output_3
	Output4 bool `json:"output4"` // dev#1.output_4
}

// PulserConfig no longer carries a pulse shape or a burst count. The shape is
// hardcoded negative in the apply plan (the only shape this probe/plate
// combination was ever characterised on) and burst is always 1: raising
// transmit energy is not a knob an operator should reach for when interface
// clipping at +-32k is by design (invariant #4).
type PulserConfig struct {
	LengthNs  float64 `json:"lengthNs"`  // 25..500  LOCK
	BankIndex int     `json:"bankIndex"` // 1..8     LOCK
}

// ReceiverConfig keeps only the analog band. Analog and digital gain are
// written as a hard 0/0 by the apply plan and are deliberately NOT fields:
// invariant #4 says the interface clips by design at HV 40 V with gains zeroed,
// and every "fix the clipping" regression started with a gain field existing.
type ReceiverConfig struct {
	DigitalHpf  int        `json:"digitalHpf"`  // 0 none / 1 3480 / 2 1860 / 3 900 kHz
	BandpassKHz [2]float64 `json:"bandpassKHz"` // 0..25000
}

// DigitizationConfig has no sampling code. FMC firmware digitizes at 50 MHz
// regardless of what frequency_sampling is set to (the PA code table does not
// apply), so the setter is hardcoded in the apply plan and the rate is a
// read-only 50 MHz everywhere in the UI.
type DigitizationConfig struct {
	Points        int     `json:"points"`        // 256..32768  LOCK
	DelayOffsetUs float64 `json:"delayOffsetUs"` // 0..2000
}

type TimingConfig struct {
	SyncPeriodUs    float64 `json:"syncPeriodUs"`    // >= syncPeriodMinUs
	SyncSource      int     `json:"syncSource"`      // 0..8
	InternalTimerUs float64 `json:"internalTimerUs"` // used when source == 7
}

type AcquisitionConfig struct {
	PreviewHz int `json:"previewHz"` // 0 = derive from the byte budget alone
}

type EncodersConfig struct {
	Units       []int     `json:"units"`
	Resolutions []float64 `json:"resolutions"`
	Invert      []bool    `json:"invert"`
}

type TestingConfig struct {
	PatternEnabled    bool   `json:"patternEnabled"`
	Pattern           string `json:"pattern"` // "echoTrain" | "sine" | "csv"
	EncoderSimEnabled bool   `json:"encoderSimEnabled"`
}

// ProcessingConfig is what the TFM engine and the steel-profile estimator
// need, and nothing else. StandoffMm survives as the SEED for the interface
// fitter only — it is not operator-facing; the app displays the fitted water
// path in the status strip instead.
type ProcessingConfig struct {
	VelocityMps float64 `json:"velocityMps"`
	NominalMm   float64 `json:"nominalMm"`
	ToleranceMm float64 `json:"toleranceMm"`
	StandoffMm  float64 `json:"standoffMm"`

	// SmoothingMm is the operator's lateral median window over BOTH measured
	// layers, in mm of plate. 0 disables it and the pipeline is bit-identical
	// to having no such setting.
	//
	// It exists because the per-column residual on corroded plate is CLUTTER,
	// not topography: measured across six laterally nudged captures of one
	// spot, the per-column profile decorrelated completely (mean correlation
	// +0.07) while the frame medians agreed to 0.030 mm. Averaging laterally
	// therefore buys real accuracy rather than hiding real detail —
	// 0.217 mm per column falls to 0.121 at a 4 mm window and 0.080 at 8 mm,
	// and the worst single column falls from 1.68 mm to 0.13.
	//
	// What it costs is set by the median's own behaviour, which is a step and
	// not a rolloff: a feature WIDER than about half the window survives at
	// essentially full depth, and one narrower vanishes outright. Measured on
	// drilled holes — at an 8 mm window, 5.0/3.2/3.0 mm holes kept 98/98/96%
	// of their depth while 2.4/1.0/0.8 mm holes kept 0/0/1%. So the setting
	// trades a known minimum feature width for accuracy, and nothing else.
	//
	// OFF for machined plate and for anything where isolated small pitting is
	// the target. ON for area wastage on corroded plate, which is what a tank
	// floor actually presents. It is deliberately manual: a single frame
	// cannot choose this for itself, because the clutter is correlated over
	// ~3 mm and that is the same scale as the features worth keeping, so
	// nothing in one frame separates them. Under a moving probe successive
	// frames give independent clutter realisations of overlapping metal and
	// the choice could be made from data — that is the later change.
	SmoothingMm float64 `json:"smoothingMm"`
}

// ScanConfig describes the FMC firing geometry. There is no Kind: FMC is the
// only scan this server builds, and a discriminant with one legal value is
// just a branch waiting to rot.
//
// Working point (proven on board SN 223, and the geometry the entire profile
// estimator was calibrated against): aperture 2, step 2 -> 32 tx positions
// across a 64-element probe, 32 parallel receive channels.
type ScanConfig struct {
	ProbeElements int     `json:"probeElements"` // 64
	PitchMm       float64 `json:"pitchMm"`       // 0.6
	ProbeFreqMHz  float64 `json:"probeFreqMHz"`  // 5
	Aperture      int     `json:"aperture"`      // elements per tx sub-aperture
	Step          int     `json:"step"`          // elements between tx positions
	RxCount       int     `json:"rxCount"`       // parallel receive channels per shot
}

// BoardReceivers is how many parallel receive channels the hardware has. The
// SPIKE 32x64 has 32 for 64 elements, which is why FMC windows the receive
// aperture into banks at all.
const BoardReceivers = 32

// FiringsPerTx is how many board firings it takes to collect one transmit
// position's full receive aperture.
//
// 1 in plain FMC, where a single firing captures RxCount <= 32 channels.
// 2 in FMC_TDM with RxCount 64: the board time-division-multiplexes its 32
// receivers across all 64 elements, so a transmit is fired twice and the
// halves are combined. That doubles cycle time but leaves the BYTE rate
// unchanged — twice the channels arriving half as often — which is why every
// rate below has to divide by this rather than assume one firing per frame.
// Getting this wrong makes a legal TDM config look like 217 MB/s and be
// refused at arm time for a bus overrun that is not happening.
func (s ScanConfig) FiringsPerTx() int {
	if s.RxCount <= BoardReceivers {
		return 1
	}
	return (s.RxCount + BoardReceivers - 1) / BoardReceivers
}

// TxPositions is the number of firings per cycle implied by the geometry. It
// is the single most load-bearing number in the system: cycle time is
// TxPositions * syncPeriodUs, which sets the cycle rate, the produced byte
// rate, and the coherent block size K in the imaging worker.
func (s ScanConfig) TxPositions() int {
	if s.Step <= 0 || s.Aperture <= 0 || s.ProbeElements <= 0 {
		return 0
	}
	n := (s.ProbeElements-s.Aperture)/s.Step + 1
	if n < 1 {
		return 0
	}
	return n
}

type Config struct {
	V            int                `json:"v"` // ConfigVersion
	Device       DeviceConfig       `json:"device"`
	IO           IoConfig           `json:"io"`
	Pulser       PulserConfig       `json:"pulser"`
	Receiver     ReceiverConfig     `json:"receiver"`
	Digitization DigitizationConfig `json:"digitization"`
	Timing       TimingConfig       `json:"timing"`
	Scan         ScanConfig         `json:"scan"`
	Acquisition  AcquisitionConfig  `json:"acquisition"`
	Encoders     EncodersConfig     `json:"encoders"`
	Testing      TestingConfig      `json:"testing"`
	Processing   ProcessingConfig   `json:"processing"`
}

// ---------------------------------------------------------------------------
// State / status
// ---------------------------------------------------------------------------

type AppStatus struct {
	Acq            string  `json:"acq"` // idle|configuring|armed|firing|draining
	PrfActualHz    float64 `json:"prfActualHz"`
	PrfRequestedHz float64 `json:"prfRequestedHz"`
	DataRateMBs    float64 `json:"dataRateMBs"`
	FramesPerSec   int     `json:"framesPerSec"`
	CyclesPerSec   float64 `json:"cyclesPerSec"` // FMC frames/s ÷ tx positions
	BacklogWords   int     `json:"backlogWords"`
	PreviewHz      int     `json:"previewHz"`     // effective FMC preview publish rate
	PreviewMBs     float64 `json:"previewMBs"`    // bytes actually published to clients, MB/s
	PreviewBudget  float64 `json:"previewBudget"` // configured budget, MB/s
	DroppedFrames  int     `json:"droppedFrames"`
	TempC          float64 `json:"tempC"`
	UptimeS        int     `json:"uptimeS"`
	HvOn           bool    `json:"hvOn"`
	Simulated      bool    `json:"simulated"`
}

type DeviceInfo struct {
	Model          string `json:"model"`
	Serial         string `json:"serial"`
	Connection     string `json:"connection"` // usb3|usb2|ethernet
	FirmwareMode   string `json:"firmwareMode"`
	LibraryVersion string `json:"libraryVersion"`
	DeviceInfo     string `json:"deviceInfo"`
}

type FrameLayoutInfo struct {
	Metadata  bool `json:"metadata"`
	Header    bool `json:"header"`
	Ascan     bool `json:"ascan"`
	Footer    bool `json:"footer"`
	SizeWords int  `json:"sizeWords"`
}

type AppState struct {
	DeviceOpen      bool            `json:"deviceOpen"`
	SyncPeriodMinUs float64         `json:"syncPeriodMinUs"`
	IoInputs        int             `json:"ioInputs"`
	Frame           FrameLayoutInfo `json:"frame"`
	Status          *AppStatus      `json:"status,omitempty"`

	// Diagnostics. Without these, a server stuck in `absent` because the
	// SI5G library failed to load (or number_of_devices errored) is
	// indistinguishable over HTTP from "no board plugged in": deviceOpen is
	// false and everything else is zero. Mirror these into the frontend's
	// types.ts in the same commit (repo rule; there is no codegen).
	Phase     string `json:"phase"`               // absent|opening|configuring|running|degraded|closing|hung
	LastError string `json:"lastError,omitempty"` // last transport or SI5G error text
	Detected  int    `json:"detected"`            // number_of_devices before Open
	LibPath   string `json:"libPath,omitempty"`   // shared object actually loaded

	// FirmwareOK is false when the board did not boot into FMC firmware.
	// That is not a mode to switch out of (invariant #1: never touch
	// dev#1.mode at runtime) — it is a config-devices.ini misconfiguration,
	// and acquisition refuses to start until it is fixed and the board
	// power-cycled. The app surfaces this as a blocking banner.
	FirmwareOK bool `json:"firmwareOK"`

	// NeedsReopen is set when a read loop failed to drain within its join
	// deadline and was leaked. The board's readout engine may still have a
	// transfer in flight on a pinned OS thread, so reconfiguring it would
	// wedge it (invariant #2). Close/reopen is the only known-good recovery.
	NeedsReopen bool `json:"needsReopen"`
}

type SequenceMapEntry struct {
	FiringIndex   int    `json:"firingIndex"`
	SpatialIndex  int    `json:"spatialIndex"`
	CenterElement int    `json:"centerElement"`
	Label         string `json:"label"`
}

// ---------------------------------------------------------------------------
// WebSocket messages
// ---------------------------------------------------------------------------

type ApplyProgressMsg struct {
	T      string `json:"t"` // "applyProgress"
	Step   string `json:"step"`
	Detail string `json:"detail,omitempty"`
}

// StatusMsg is AppStatus with the WS discriminator attached.
type StatusMsg struct {
	T string `json:"t"` // "status"
	AppStatus
}

// ---------------------------------------------------------------------------
// REST error / result model
// ---------------------------------------------------------------------------

type Si5gError struct {
	Op    string `json:"op"`
	Param string `json:"param"`
	Code  int    `json:"code"`
	Name  string `json:"name"`
}

type APIError struct {
	Error string      `json:"error"`
	Si5g  []Si5gError `json:"si5g,omitempty"`
}

type Problem struct {
	Path string `json:"path"`
	Msg  string `json:"msg"`
}

type ValidateResult struct {
	Valid    bool      `json:"valid"`
	Problems []Problem `json:"problems"`
}

type Clip struct {
	Path      string  `json:"path"`
	Requested float64 `json:"requested"`
	Applied   float64 `json:"applied"`
}

type ApplyResult struct {
	Applied []string `json:"applied"`
	Clipped []Clip   `json:"clipped"`
	Errors  []string `json:"errors"`
	// Skipped is set (and Applied empty) when staged == applied and the
	// board was left completely untouched. See device.Apply: a no-op apply
	// against a streaming board is the difference between a harmless click
	// and a wedged readout engine.
	Skipped string `json:"skipped,omitempty"`
}

// ConfigEnvelope is the GET /api/config body: both halves plus the dirty flag,
// so the app never has to diff client-side to know whether Apply would do
// anything.
type ConfigEnvelope struct {
	Staged  Config `json:"staged"`
	Applied Config `json:"applied"`
	Dirty   bool   `json:"dirty"`
}

type ScriptResult struct {
	JSON        string             `json:"json"`
	SequenceMap []SequenceMapEntry `json:"sequenceMap"`
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Uplink
// ---------------------------------------------------------------------------

// UplinkStats is published over the WebSocket (JSON, t:"uplink") so the bench
// UI and the TUI can see whether the SBC is keeping up. This replaces TFMLive:
// the SBC no longer produces images, only compressed frames.
type UplinkStats struct {
	Connected  bool    `json:"connected"`
	Target     string  `json:"target"`
	FramesSent uint64  `json:"framesSent"`
	BytesIn    uint64  `json:"bytesIn"`  // pre-compression
	BytesOut   uint64  `json:"bytesOut"` // post-compression, on the wire
	Ratio      float64 `json:"ratio"`    // BytesIn / BytesOut
	MBps       float64 `json:"mbps"`     // wire rate, 1s EWMA
	Queued     int     `json:"queued"`   // ring occupancy
	Dropped    uint64  `json:"dropped"`  // ring overflow — MUST stay 0
	CompressMs float64 `json:"compressMs"`
}

// Flaw is one detected mid-wall reflector run (tfm.MidWallFlaw on the wire).
type Flaw struct {
	X0Mm float64 `json:"x0Mm"`
	X1Mm float64 `json:"x1Mm"`
	// TRUE depth below the entry surface — topside floors arrive already
	// corrected for the water/steel velocity mapping (see tfm.MidWallFlaw).
	DepthMm float64 `json:"depthMm"`
	Score   float64 `json:"score"` // peak z-score over the run
	// "mid" (in-wall reflector) or "topside" (cavity floor). Empty on
	// servers predating the topside branch; consumers treat empty as "mid".
	Kind string `json:"kind,omitempty"`
}
