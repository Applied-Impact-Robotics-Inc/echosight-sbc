// Package tui renders the operator-facing status view on the robot console.
//
// Bubble Tea owns stdout once it starts, so nothing else in the process may
// write there. All logging goes to the state store's ring (shown here) and,
// when --log-file is set, to disk.
package tui

import (
	"context"
	"fmt"
	"strings"
	"time"

	tea "github.com/charmbracelet/bubbletea"
	"github.com/charmbracelet/lipgloss"

	"echosight/internal/device"
	"echosight/internal/state"
)

type Actions struct {
	Reconnect func()
	Start     func()
	Stop      func()
}

type Model struct {
	store  *state.Store
	sup    *device.Supervisor
	act    Actions
	listen string

	width  int
	height int
	quit   bool
}

func New(store *state.Store, sup *device.Supervisor, listen string, act Actions) Model {
	return Model{store: store, sup: sup, act: act, listen: listen, width: 100, height: 34}
}

type tickMsg time.Time

func tick() tea.Cmd {
	return tea.Tick(250*time.Millisecond, func(t time.Time) tea.Msg { return tickMsg(t) })
}

func (m Model) Init() tea.Cmd { return tick() }

func (m Model) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	switch msg := msg.(type) {
	case tea.WindowSizeMsg:
		m.width, m.height = msg.Width, msg.Height
	case tickMsg:
		return m, tick()
	case tea.KeyMsg:
		switch msg.String() {
		case "q", "ctrl+c":
			// Only return tea.Quit here. Calling a callback that cancels the
			// context passed to tea.WithContext from INSIDE Update deadlocks
			// the program: cancelling p.ctx makes handleCommands exit, and the
			// event loop then blocks forever on its unbuffered `cmds <- cmd`
			// send right after Update returns (observed hung at tea.go:445,
			// goroutine 1 in [chan send]). Main cancels the app context after
			// Run() has returned instead.
			m.quit = true
			return m, tea.Quit
		case "r":
			if m.act.Reconnect != nil {
				m.act.Reconnect()
			}
		case "s":
			sn := m.store.Snapshot()
			if sn.Status.Acq == "firing" {
				if m.act.Stop != nil {
					m.act.Stop()
				}
			} else if m.act.Start != nil {
				m.act.Start()
			}
		}
	}
	return m, nil
}

// ---------------------------------------------------------------------------
// styling
// ---------------------------------------------------------------------------

var (
	cAccent = lipgloss.Color("51")  // cyan
	cDim    = lipgloss.Color("244")
	cWarn   = lipgloss.Color("214")
	cErr    = lipgloss.Color("203")
	cOK     = lipgloss.Color("42")

	titleStyle = lipgloss.NewStyle().Bold(true).Foreground(cAccent)
	dimStyle   = lipgloss.NewStyle().Foreground(cDim)
	keyStyle   = lipgloss.NewStyle().Foreground(cDim)
	boxStyle   = lipgloss.NewStyle().
			Border(lipgloss.RoundedBorder()).
			BorderForeground(lipgloss.Color("238")).
			Padding(0, 1)
)

func chip(text string, fg lipgloss.Color) string {
	return lipgloss.NewStyle().Foreground(fg).Bold(true).Render("[" + text + "]")
}

func phaseColor(p state.Phase) lipgloss.Color {
	switch p {
	case state.PhaseRunning:
		return cOK
	case state.PhaseConfiguring, state.PhaseOpening:
		return cWarn
	case state.PhaseHung, state.PhaseDegraded:
		return cErr
	default:
		return cDim
	}
}

func kv(k, v string) string {
	return keyStyle.Render(fmt.Sprintf("%-14s", k)) + v
}

func (m Model) View() string {
	if m.quit {
		return "shutting down…\n"
	}
	sn := m.store.Snapshot()

	// --- header ---
	head := []string{
		titleStyle.Render("ECHOSIGHT"),
		dimStyle.Render(m.listen),
		chip(string(sn.Phase), phaseColor(sn.Phase)),
	}
	if sn.State.DeviceOpen {
		head = append(head, dimStyle.Render(fmt.Sprintf("%s SN %s %s",
			sn.Device.Model, sn.Device.Serial, sn.Device.Connection)))
	} else {
		head = append(head, dimStyle.Render(fmt.Sprintf("detected %d", sn.Detected)))
	}
	if sn.Status.HvOn {
		head = append(head, chip("ARMED HV", cErr))
	}
	if sn.Status.Simulated {
		head = append(head, chip("PATTERN", cWarn))
	}
	header := strings.Join(head, "  ")

	// --- device panel ---
	dev := strings.Join([]string{
		kv("phase", fmt.Sprintf("%s  %s", sn.Phase, dimStyle.Render(dur(time.Since(sn.PhaseSince))))),
		kv("model", orDash(sn.Device.Model)),
		kv("serial", orDash(sn.Device.Serial)),
		kv("firmware", warnIf(orDash(sn.Device.FirmwareMode), !sn.State.FirmwareOK)),
		kv("library", orDash(sn.Device.LibraryVersion)),
		kv("so", orDash(shorten(sn.LibPath, 34))),
		kv("reconnects", fmt.Sprintf("%d", sn.Reconnects)),
	}, "\n")

	// --- acquisition panel ---
	acq := strings.Join([]string{
		kv("acq", acqChip(sn.Status.Acq)),
		kv("prf", fmt.Sprintf("%.0f / %.0f Hz", sn.Status.PrfActualHz, sn.Status.PrfRequestedHz)),
		kv("data rate", fmt.Sprintf("%.1f MB/s", sn.Status.DataRateMBs)),
		kv("frames", fmt.Sprintf("%d /s", sn.Status.FramesPerSec)),
		// Cycles, not frames, is the rate everything downstream is derived
		// from: TFM image rate = cycles/s / K.
		kv("cycles", fmt.Sprintf("%.1f /s", sn.Status.CyclesPerSec)),
		kv("backlog", warnIf(fmt.Sprintf("%d w", sn.Status.BacklogWords), sn.Status.BacklogWords > 200000)),
		kv("dropped", warnIf(fmt.Sprintf("%d", sn.Status.DroppedFrames), sn.Status.DroppedFrames > 0)),
		kv("temp", fmt.Sprintf("%.1f °C", sn.Status.TempC)),
	}, "\n")

	// --- measurement panel ---
	meas := strings.Join([]string{
		kv("uplink", uplinkLine(sn)),
		kv("sweep", fmt.Sprintf("%d", sn.LastSweepID)),
		kv("sequences", fmt.Sprintf("%d", m.sup.SequenceCount())),
		kv("ws clients", fmt.Sprintf("%d", sn.Clients)),
		kv("client drops", warnIf(fmt.Sprintf("%d", sn.ClientDrops), sn.ClientDrops > 0)),
		kv("uptime", dur(time.Since(sn.StartedAt))),
	}, "\n")

	colW := (m.width - 12) / 3
	if colW < 26 {
		colW = 26
	}
	cols := lipgloss.JoinHorizontal(lipgloss.Top,
		boxStyle.Width(colW).Render(titleStyle.Render("Device")+"\n"+dev),
		boxStyle.Width(colW).Render(titleStyle.Render("Acquisition")+"\n"+acq),
		boxStyle.Width(colW).Render(titleStyle.Render("Measurement")+"\n"+meas),
	)

	// --- error banner ---
	banner := ""
	if sn.LastError != "" {
		banner = lipgloss.NewStyle().Foreground(cErr).
			Render("! "+wrap(sn.LastError, m.width-4)) + "\n"
	}
	if sn.Phase == state.PhaseHung {
		banner += lipgloss.NewStyle().Foreground(cErr).Bold(true).Render(
			"! Open() is wedged. Restarting the process is the only recovery; "+
				"power cycle the board or use uhubctl on its port.") + "\n"
	}

	// --- log tail ---
	logLines := m.height - 20
	if logLines < 3 {
		logLines = 3
	}
	var lb strings.Builder
	lb.WriteString(titleStyle.Render("Log") + "\n")
	for _, l := range m.store.Tail(logLines) {
		style := dimStyle
		switch l.Level {
		case "warn":
			style = lipgloss.NewStyle().Foreground(cWarn)
		case "error":
			style = lipgloss.NewStyle().Foreground(cErr)
		}
		lb.WriteString(dimStyle.Render(l.At.Format("15:04:05")) + " " +
			style.Render(shorten(l.Msg, m.width-14)) + "\n")
	}

	help := keyStyle.Render("q quit   r reconnect   s start/stop acquisition")

	return strings.Join([]string{header, "", cols, banner, boxStyle.Width(m.width - 4).Render(lb.String()), help, ""}, "\n")
}

func acqChip(s string) string {
	switch s {
	case "firing":
		return lipgloss.NewStyle().Foreground(cOK).Bold(true).Render(s)
	case "configuring", "draining", "armed":
		return lipgloss.NewStyle().Foreground(cWarn).Render(s)
	default:
		return dimStyle.Render(s)
	}
}

func uplinkLine(sn state.Snapshot) string {
	if !sn.UplinkConnected {
		return "down"
	}
	s := fmt.Sprintf("%.1f MB/s  %.2fx  q%d", sn.UplinkMBps, sn.UplinkRatio, sn.UplinkQueued)
	if sn.UplinkDropped > 0 {
		s += fmt.Sprintf("  DROPPED %d", sn.UplinkDropped)
	}
	return s
}


func warnIf(s string, bad bool) string {
	if bad {
		return lipgloss.NewStyle().Foreground(cWarn).Render(s)
	}
	return s
}

func orDash(s string) string {
	if s == "" {
		return dimStyle.Render("—")
	}
	return s
}

func shorten(s string, n int) string {
	if n < 8 || len(s) <= n {
		return s
	}
	return "…" + s[len(s)-n+1:]
}

func wrap(s string, n int) string {
	if n < 20 || len(s) <= n {
		return s
	}
	return s[:n-1] + "…"
}

func dur(d time.Duration) string {
	d = d.Round(time.Second)
	h := int(d.Hours())
	m := int(d.Minutes()) % 60
	sec := int(d.Seconds()) % 60
	if h > 0 {
		return fmt.Sprintf("%dh%02dm", h, m)
	}
	if m > 0 {
		return fmt.Sprintf("%dm%02ds", m, sec)
	}
	return fmt.Sprintf("%ds", sec)
}

// Run starts the TUI and blocks until the operator quits or ctx is cancelled.
func Run(ctx context.Context, m Model) error {
	p := tea.NewProgram(m, tea.WithAltScreen(), tea.WithContext(ctx))
	_, err := p.Run()
	return err
}
