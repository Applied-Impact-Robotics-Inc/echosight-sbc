// Command fmcprobe is a minimal, vendor-faithful FMC bring-up used to bisect
// the FMC readout wedge (blocking dev#1.data reads returning nbBytes=0 while
// available_data grows). It replicates Socomate's CollectFMCData.py sample
// EXACTLY — the same dozen parameters, nothing from the echosight config
// plan — then collects via the manual 3.5 transfer method (or the legacy
// poll+read flow with --legacy).
//
// Usage on the SBC (board already booted or runtime-switched to FMC mode):
//
//	./fmcprobe                      # sample-faithful: 1 seq, tx el 16, rx 1..32
//	./fmcprobe --sparse             # our 8x(ap8) sparse scenario instead
//	./fmcprobe --fs 0               # try frequency_sampling code 0 (default 1, per sample)
//	./fmcprobe --points 2048        # record length (default 1024, per sample)
//	./fmcprobe --legacy             # poll available_data + read, old style
//	./fmcprobe --seconds 10
//
// Interpretation:
//   - fmcprobe works, echosight-server wedges  -> a config-plan parameter is
//     the trigger; bisect by adding plan params to this probe in groups.
//   - fmcprobe ALSO gets zeros                 -> environment/library/firmware;
//     send this program + its output to Socomate as the repro.
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"echosight/spike"
)

func die(step string, err error) {
	if err != nil {
		fmt.Fprintf(os.Stderr, "FAIL %s: %v\n", step, err)
		_ = spike.Close()
		os.Exit(1)
	}
}

func setInt(param string, unit, v int) {
	got, clipped, err := spike.SetInt(param, unit, v)
	die("SetInt "+param, err)
	if clipped || got != v {
		fmt.Printf("  %s: requested %d, applied %d\n", param, v, got)
	}
}

func setDbl(param string, unit int, v float64) {
	got, clipped, err := spike.SetDbl(param, unit, v)
	die("SetDbl "+param, err)
	if clipped || got != v {
		fmt.Printf("  %s: requested %g, applied %g\n", param, v, got)
	}
}

// tolerant setters for the bisect stages: print failures, keep going — the
// server's Exec records per-setter errors the same way, and the wedge may be
// caused by a write that succeeds.
func try(step string, err error) {
	if err != nil {
		fmt.Printf("  EXTRA %s -> error: %v\n", step, err)
	}
}

func tryInt(param string, unit, v int) {
	_, _, err := spike.SetInt(param, unit, v)
	try(param, err)
}

func tryDbl(param string, unit int, v float64) {
	_, _, err := spike.SetDbl(param, unit, v)
	try(param, err)
}

// applyExtras mirrors internal/config/apply.go Plan writes the Socomate FMC
// sample does NOT make, in cumulative suspicion-ordered stages. numSeq sizes
// the per-sequence arrays exactly like the server does.
func applyExtras(stage, numSeq int) {
	if stage >= 1 {
		fmt.Println("EXTRA stage 1: type_of_acquisition")
		tryInt("dev#1.type_of_acquisition", spike.UnitNone, 1)
	}
	if stage >= 2 {
		fmt.Println("EXTRA stage 2: gate arrays + per-seq DAC clear")
		for gi := 1; gi <= 3; gi++ {
			gp := func(name string) string { return fmt.Sprintf("dev#1.grp#1.g#%d.%s", gi, name) }
			zi := make([]int32, numSeq)
			zd := make([]float64, numSeq)
			one := make([]float64, numSeq)
			for i := range one {
				one[i] = 1
			}
			try(gp("gate_triggers"), spike.Set1DInt(gp("gate_triggers"), spike.UnitNone, zi))
			try(gp("gate_phases"), spike.Set1DInt(gp("gate_phases"), spike.UnitNone, zi)) // 0 = inactive
			try(gp("gate_point_of_measures"), spike.Set1DInt(gp("gate_point_of_measures"), spike.UnitNone, zi))
			try(gp("gate_positions"), spike.Set1DDbl(gp("gate_positions"), spike.UnitUs, one))
			try(gp("gate_widths"), spike.Set1DDbl(gp("gate_widths"), spike.UnitUs, one))
			try(gp("gate_levels"), spike.Set1DDbl(gp("gate_levels"), spike.UnitNone, zd))
		}
		for seq := 1; seq <= numSeq; seq++ {
			p := fmt.Sprintf("dev#1.grp#1.seq#%d.receiver_dac", seq)
			try(p, spike.Set1DDblDim(p, spike.UnitUs, 0, []float64{0}))
		}
	}
	if stage >= 3 {
		fmt.Println("EXTRA stage 3: digital receiver path")
		tryDbl("dev#1.grp#1.receiver_digital_gain", spike.UnitNone, 24)
		try("band_pass", spike.Set1DDbl("dev#1.grp#1.receiver_digital_band_pass_filter", spike.UnitKHz, []float64{2000, 8000}))
		tryInt("dev#1.grp#1.receiver_dac_enable", spike.UnitNone, 0)
		tryInt("dev#1.grp#1.receiver_dac_trigger", spike.UnitNone, 0)
	}
	if stage >= 4 {
		fmt.Println("EXTRA stage 4: misc device params")
		try("device_name", spike.SetStr("dev#1.device_name", spike.UnitNone, "SPIKE-01"))
		tryInt("dev#1.receiver_preampli", spike.UnitNone, 1)
		tryInt("dev#1.IO_5V", spike.UnitNone, 0)
		tryInt("dev#1.output_3", spike.UnitNone, 0)
		tryInt("dev#1.output_4", spike.UnitNone, 0)
		tryInt("dev#1.synchronization_source", spike.UnitNone, 0)
		try("encoder_units", spike.Set1DInt("dev#1.encoder_units", spike.UnitNone, []int32{1, 1, 0, 0}))
		try("encoder_resolutions", spike.Set1DDbl("dev#1.encoder_resolutions", spike.UnitNone, []float64{1, 1, 1, 1}))
		tryInt("dev#1.encoder_directions", spike.UnitNone, 0)
		tryInt("dev#1.grp#1.pulser_index", spike.UnitNone, 1)
		tryDbl("dev#1.grp#1.delay_offset", spike.UnitUs, 0)
		tryInt("dev#1.enable_pattern", spike.UnitNone, 0)
		tryInt("dev#1.enable_encoders_simulation", spike.UnitNone, 0)
	}
}

func sampleScenario() string {
	// Verbatim structure of the 32x64 sample: single sequence, tx element
	// 16 aperture 1, rx 1..32 flat.
	rx := make([]int, 32)
	dl := make([]int, 32)
	for i := range rx {
		rx[i] = i + 1
	}
	b, _ := json.Marshal(map[string]any{
		"Group": 1, "Probe": 1,
		"Sequences": []map[string]any{{
			"Sequence":  1,
			"Emission":  map[string]any{"Aperture": 1, "Elements": []int{16}, "Delays": []int{0}},
			"Reception": map[string]any{"Aperture": 32, "Elements": rx, "Delays": dl},
		}},
	})
	return string(b)
}

func sparseScenario() string {
	sc, err := spike.BuildFMCScan(spike.FMCScanConfig{ProbeElements: 64, Aperture: 8, Step: 8, RxCount: 32})
	die("BuildFMCScan", err)
	js, err := sc.JSON()
	die("scenario JSON", err)
	return js
}

func main() {
	var (
		sparse   = flag.Bool("sparse", false, "use our 8-tx sparse scenario instead of the sample's single sequence")
		fs       = flag.Int("fs", 1, "frequency_sampling code (sample uses 1 = 50 MHz in FMC mode)")
		points   = flag.Int("points", 1024, "size_of_digitalization")
		hv       = flag.Int("hv", 50, "high voltage (V); 0 skips enabling HV")
		syncUs   = flag.Float64("sync", 5000, "sync period (µs); sample uses 5000")
		seconds  = flag.Int("seconds", 8, "how long to collect")
		legacy   = flag.Bool("legacy", false, "poll available_data + read (old method) instead of transfer method")
		xferMB   = flag.Int("xfer-mb", 32, "transfer method: MB per initialize_data_transfer")
		packetKW = flag.Int("packet-kw", 512, "transfer method: read packet size in kilowords (<= 2048 = 4 MB)")
		extra    = flag.Int("extra", 0, "add echosight plan parameter groups cumulatively: 1=acquisition_type, 2=+gates/DAC arrays, 3=+digital rx path, 4=+misc device params")
		reapply  = flag.Bool("reapply", false, "after 3s of collection, stop, rewrite ALL settings (like a server apply), restart — reproduces the re-apply wedge")
	)
	flag.Parse()

	die("Load", spike.Load())
	fmt.Printf("library: %s\n", spike.LibPath())
	n, err := spike.DeviceCount()
	die("DeviceCount", err)
	fmt.Printf("devices detected: %d\n", n)
	die("Open", spike.Open())
	defer func() { _ = spike.Close() }()

	mode, err := spike.GetInt("dev#1.mode", spike.UnitNone)
	die("mode", err)
	fmt.Printf("dev#1.mode = %d (%s)\n", mode,
		map[int]string{0: "PA", 1: "FMC", 2: "FMC_TDM"}[mode])
	if mode != spike.ModeFMC && mode != spike.ModeFMCTDM {
		fmt.Println("switching to FMC via dev#1.mode (runtime switch)")
		setInt("dev#1.mode", spike.UnitNone, spike.ModeFMC)
	}

	// --- parameters: the sample's set, nothing more ---
	fmt.Println("applying the sample's parameter set:")
	die("clear settings", spike.SetStr("dev#1.settings", spike.UnitNone, ""))
	setInt("dev#1.grp#1.size_of_digitalization", spike.UnitNone, *points)
	setInt("dev#1.grp#1.pulser_type", spike.UnitNone, 1)
	setDbl("dev#1.grp#1.pulser_length", spike.UnitNs, 100)
	setDbl("dev#1.grp#1.sync_period", spike.UnitUs, *syncUs)
	setInt("dev#1.grp#1.frequency_sampling", spike.UnitNone, *fs)
	die("analog filters", spike.Set1DInt("dev#1.receiver_analog_filters", spike.UnitNone, []int32{1, 0}))
	setInt("dev#1.receiver_digital_high_pass_filter", spike.UnitNone, 1)
	setDbl("dev#1.grp#1.receiver_analog_gain", spike.UnitNone, 30)

	js := sampleScenario()
	if *sparse {
		js = sparseScenario()
	}
	die("settings", spike.SetStr("dev#1.settings", spike.UnitNone, js))

	if *hv > 0 {
		setInt("dev#1.high_voltage", spike.UnitNone, *hv)
		setInt("dev#1.enable_high_voltage", spike.UnitNone, 1)
	}
	setInt("dev#1.grp#1.scope_video_mode", spike.UnitNone, 0)

	shots, err := spike.GetInt("dev#1.grp#1.number_of_shots", spike.UnitNone)
	if err != nil {
		shots = 1
	}
	if *extra > 0 {
		applyExtras(*extra, shots)
	}
	frame, err := spike.GetInt("dev#1.grp#1.frame_size", spike.UnitNone)
	die("frame_size", err)
	fmt.Printf("number_of_shots %d, frame_size %d words (%d bytes/cycle)\n",
		shots, frame, frame*shots*2)

	// clean exit on ^C
	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)
	done := time.After(time.Duration(*seconds) * time.Second)

	setInt("dev#1.enable_acquisition", spike.UnitNone, 1)
	setInt("dev#1.enable_prf", spike.UnitNone, 1)
	defer func() {
		_, _, _ = spike.SetInt("dev#1.enable_prf", spike.UnitNone, 0)
		_, _, _ = spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 0)
	}()

	var (
		totalWords int
		reads      int
		zeros      int
		firstDump  = true
		t0         = time.Now()
	)
	report := func() {
		el := time.Since(t0).Seconds()
		fmt.Printf("\n=== %0.1fs: %d reads, %d zero/failed, %.1f MB total, %.1f MB/s ===\n",
			el, reads, zeros, float64(totalWords)*2/(1<<20), float64(totalWords)*2/(1<<20)/el)
	}
	dump := func(buf []uint16, n int) {
		if !firstDump || n < 16 {
			return
		}
		firstDump = false
		var sb strings.Builder
		for i := 0; i < 16; i++ {
			fmt.Fprintf(&sb, "%04x ", buf[i])
		}
		timer := uint64(buf[0]) | uint64(buf[1])<<16 | uint64(buf[2])<<32 | uint64(buf[3])<<48
		fmt.Printf("first words: %s\n(header timer would be %d µs)\n", sb.String(), timer)
	}

	if *legacy {
		fmt.Println("collecting: LEGACY poll+read")
		buf := make([]uint16, frame*shots)
		for {
			select {
			case <-stop:
				report()
				return
			case <-done:
				report()
				return
			default:
			}
			avail, err := spike.GetInt("dev#1.available_data", spike.UnitNone)
			if err != nil {
				fmt.Printf("available_data: %v\n", err)
				time.Sleep(50 * time.Millisecond)
				continue
			}
			if avail < len(buf) {
				time.Sleep(2 * time.Millisecond)
				continue
			}
			n, err := spike.GetData("dev#1.data", len(buf), buf)
			reads++
			if err != nil || n < len(buf) {
				zeros++
				if zeros <= 5 || zeros%100 == 0 {
					fmt.Printf("read #%d: n=%d err=%v (avail was %d)\n", reads, n, err, avail)
				}
				continue
			}
			totalWords += n
			dump(buf, n)
		}
	}

	reconfigure := func() {
		fmt.Println("\n--- REAPPLY: stop, rewrite everything, restart (server-apply style) ---")
		_, _, _ = spike.SetInt("dev#1.enable_prf", spike.UnitNone, 0)
		_, _, _ = spike.SetInt("dev#1.enable_acquisition", spike.UnitNone, 0)
		die("clear settings", spike.SetStr("dev#1.settings", spike.UnitNone, ""))
		setInt("dev#1.grp#1.size_of_digitalization", spike.UnitNone, *points)
		setDbl("dev#1.grp#1.sync_period", spike.UnitUs, *syncUs)
		setInt("dev#1.grp#1.frequency_sampling", spike.UnitNone, *fs)
		die("settings", spike.SetStr("dev#1.settings", spike.UnitNone, js))
		if *extra > 0 {
			applyExtras(*extra, shots)
		}
		setInt("dev#1.enable_acquisition", spike.UnitNone, 1)
		setInt("dev#1.enable_prf", spike.UnitNone, 1)
		fmt.Println("--- REAPPLY done; resuming collection ---")
	}
	reapplied := false

	fmt.Printf("collecting: TRANSFER method (%d MB transfers, %d KW packets)\n", *xferMB, *packetKW)
	packet := *packetKW * 1024
	buf := make([]uint16, packet)
	transferKB := *xferMB * 1024
	packetsPerTransfer := (*xferMB * 1024 * 1024) / (packet * 2)
	for {
		if _, _, err := spike.SetInt("dev#1.initialize_data_transfer", spike.UnitNone, transferKB); err != nil {
			fmt.Printf("initialize_data_transfer: %v\n", err)
			time.Sleep(200 * time.Millisecond)
			continue
		}
		for p := 0; p < packetsPerTransfer; p++ {
			select {
			case <-stop:
				report()
				return
			case <-done:
				report()
				return
			default:
			}
			if *reapply && !reapplied && time.Since(t0) > 3*time.Second {
				reapplied = true
				reconfigure()
				break // fresh initialize_data_transfer after the reconfigure
			}
			n, err := spike.GetData("dev#1.data", packet, buf) // blocks until full
			reads++
			if err != nil || n < packet {
				zeros++
				if zeros <= 5 || zeros%100 == 0 {
					fmt.Printf("read #%d: n=%d err=%v\n", reads, n, err)
				}
				time.Sleep(10 * time.Millisecond)
				continue
			}
			totalWords += n
			dump(buf, n)
		}
	}
}
