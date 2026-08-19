// Package presets persists whole configurations as JSON files on the robot.
package presets

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	"echosight/internal/config"
	"echosight/internal/wire"
)

type Entry struct {
	Name    string    `json:"name"`
	SavedAt time.Time `json:"savedAt"`
}

type Store struct{ dir string }

func New(dir string) (*Store, error) {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return nil, err
	}
	return &Store{dir: dir}, nil
}

// safeName refuses anything that could escape the preset directory. Names come
// straight off the network.
func safeName(name string) (string, error) {
	if name == "" {
		return "", fmt.Errorf("preset name is empty")
	}
	if strings.ContainsAny(name, `/\..`) || strings.ContainsRune(name, os.PathSeparator) {
		return "", fmt.Errorf("preset name may not contain path separators or dots")
	}
	return name + ".json", nil
}

func (s *Store) List() ([]Entry, error) {
	ents, err := os.ReadDir(s.dir)
	if err != nil {
		return nil, err
	}
	var out []Entry
	for _, e := range ents {
		if e.IsDir() || !strings.HasSuffix(e.Name(), ".json") {
			continue
		}
		info, err := e.Info()
		if err != nil {
			continue
		}
		out = append(out, Entry{
			Name:    strings.TrimSuffix(e.Name(), ".json"),
			SavedAt: info.ModTime(),
		})
	}
	sort.Slice(out, func(i, j int) bool { return out[i].Name < out[j].Name })
	if out == nil {
		out = []Entry{}
	}
	return out, nil
}

func (s *Store) Get(name string) (wire.Config, error) {
	var c wire.Config
	fn, err := safeName(name)
	if err != nil {
		return c, err
	}
	// Presets go through the same version gate as last-config.json: a
	// phased-array preset describes a scan this server cannot run, and
	// silently loading one would stage a config whose Apply is guaranteed to
	// fail somewhere less obvious.
	return config.Load(filepath.Join(s.dir, fn))
}

// Save writes atomically so a crash mid-write cannot leave a truncated preset
// that fails to parse on the next boot.
func (s *Store) Save(name string, c wire.Config) error {
	fn, err := safeName(name)
	if err != nil {
		return err
	}
	return config.Save(filepath.Join(s.dir, fn), c)
}

func (s *Store) Delete(name string) error {
	fn, err := safeName(name)
	if err != nil {
		return err
	}
	return os.Remove(filepath.Join(s.dir, fn))
}
