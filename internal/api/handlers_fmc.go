package api

import (
	"bufio"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

// fmcMeta is the JSON header line of a .fmc capture file. Only the fields the
// matrix export needs are decoded; the file is self-describing on purpose so
// offline tools never have to be told the geometry out of band.
type fmcMeta struct {
	Kind     string `json:"kind"`
	Geometry struct {
		NumShots   int `json:"numShots"`
		Channels   int `json:"channels"`
		Points     int `json:"points"`
		WireFrame  int `json:"wireFrame"`
		FrameLead  int `json:"frameLead"`
		CycleWords int `json:"cycleWords"`
	} `json:"geometry"`
	Cycles int `json:"cycles"`
}

// fmcMatrix streams one firing cycle of the last capture as DECIMAL text:
// one block per transmit position, rows = samples, columns = receive channels,
// tab separated.
//
// This exists alongside /raw because /raw is base64 — fine for round-tripping
// a capture into a tool, useless for the thing captures are usually opened
// for, which is looking at the numbers. At 32 tx x 800 pts x 32 ch a cycle is
// about 5 MB of text, which every editor and every spreadsheet can open.
func (s *Server) fmcMatrix(w http.ResponseWriter, r *http.Request) {
	path, err := s.sup.FMCFilePath()
	if err != nil {
		fail(w, 409, err)
		return
	}
	cycle := 0
	if v := r.URL.Query().Get("cycle"); v != "" {
		if n, err := strconv.Atoi(v); err == nil && n >= 0 {
			cycle = n
		}
	}

	f, err := os.Open(path)
	if err != nil {
		fail(w, 500, err)
		return
	}
	defer f.Close()

	br := bufio.NewReaderSize(f, 1<<20)
	line, err := br.ReadBytes('\n')
	if err != nil {
		fail(w, 500, fmt.Errorf("malformed capture file: %w", err))
		return
	}
	var meta fmcMeta
	if err := json.Unmarshal(line, &meta); err != nil {
		fail(w, 500, fmt.Errorf("capture header: %w", err))
		return
	}
	g := meta.Geometry
	if g.CycleWords <= 0 || g.NumShots <= 0 || g.Channels <= 0 || g.Points <= 0 {
		fail(w, 500, fmt.Errorf("capture header has no usable geometry"))
		return
	}
	if cycle >= meta.Cycles {
		fail(w, 400, fmt.Errorf("capture holds %d cycles; asked for index %d", meta.Cycles, cycle))
		return
	}

	// Seek to the requested cycle rather than reading the whole file: a
	// 64-cycle capture is ~100 MB and the debugger usually wants cycle 0.
	if _, err := f.Seek(int64(len(line))+int64(cycle)*int64(g.CycleWords)*2, 0); err != nil {
		fail(w, 500, err)
		return
	}
	buf := make([]byte, g.CycleWords*2)
	if _, err := readFull(f, buf); err != nil {
		fail(w, 500, fmt.Errorf("short read on cycle %d: %w", cycle, err))
		return
	}

	name := strings.TrimSuffix(filepath.Base(path), ".fmc")
	w.Header().Set("Content-Type", "text/plain; charset=utf-8")
	w.Header().Set("Content-Disposition",
		fmt.Sprintf("attachment; filename=%q", fmt.Sprintf("%s-cycle%d.txt", name, cycle)))

	out := bufio.NewWriterSize(w, 1<<20)
	defer out.Flush()

	fmt.Fprintf(out, "# echosight fmc matrix v1\n")
	fmt.Fprintf(out, "# source %s  cycle %d of %d\n", filepath.Base(path), cycle, meta.Cycles)
	fmt.Fprintf(out, "# %d tx x %d samples x %d channels, int16, 50 MHz\n",
		g.NumShots, g.Points, g.Channels)
	fmt.Fprintf(out, "# blocks are per tx; rows are samples; columns are receive channels\n")
	// The capture's own JSON header, verbatim. Offline tools need the config
	// block (aperture, step, rxCount, syncPeriodUs, delayOffsetUs) to rebuild
	// the receive-window mapping, and a decimal dump that drops it is a dump
	// nothing can reconstruct geometry from.
	fmt.Fprintf(out, "# meta %s\n", strings.TrimSpace(string(line)))

	for tx := 0; tx < g.NumShots; tx++ {
		base := tx * g.WireFrame
		if base+g.WireFrame > g.CycleWords {
			break
		}
		// Header words carry the µs timer and encoder counts; print the timer
		// so a row in the text can be tied back to a moment in the scan.
		var timer uint64
		if g.FrameLead >= 4 {
			for i := 0; i < 4; i++ {
				timer |= uint64(binary.LittleEndian.Uint16(buf[(base+i)*2:])) << (16 * i)
			}
		}
		fmt.Fprintf(out, "\n# tx %d  timer %d us\n", tx, timer)
		data := base + g.FrameLead
		for smp := 0; smp < g.Points; smp++ {
			row := data + smp*g.Channels
			for ch := 0; ch < g.Channels; ch++ {
				if ch > 0 {
					out.WriteByte('\t')
				}
				v := int16(binary.LittleEndian.Uint16(buf[(row+ch)*2:]))
				out.WriteString(strconv.Itoa(int(v)))
			}
			out.WriteByte('\n')
		}
	}
}

func readFull(f *os.File, b []byte) (int, error) {
	n := 0
	for n < len(b) {
		m, err := f.Read(b[n:])
		n += m
		if err != nil {
			return n, err
		}
	}
	return n, nil
}
