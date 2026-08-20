// Package compute pulls the acquisition configuration from the compute server.
//
// The SBC is DISKLESS with respect to configuration. It holds one in RAM for
// the life of a process and persists nothing, because both configuration
// incidents in this project were two-sources-of-truth failures: a stale
// last-config.json holding points 750 and rxCount 30 produced a diagonal crawl
// that looked like a live display for a whole session.
//
// The pull direction is deliberate. This machine already opens the uplink to
// the compute server, so it knows that address; the compute server does not
// need to know where the robot is. Asking is also what makes a restart
// self-healing.
//
// What is pulled is the config of the CURRENT SESSION, not a default preset.
// That distinction is the whole design: a reboot mid-scan must restore the
// geometry the operator chose. Pulling "the default" would silently revert
// step 1 to step 2 and every reading downstream would look perfectly healthy
// while being reconstructed against the wrong aperture.
package compute

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"time"

	"echosight/internal/wire"
)

// ErrNoSession means the compute server has no active tank.
//
// The correct response is to sit UNARMED and say so. There is deliberately no
// fallback: a default this machine could apply on its own is exactly the
// second source of truth being removed.
var ErrNoSession = errors.New("compute server has no active tank; nothing to configure from")

type Client struct {
	base string
	http *http.Client
}

func New(addr string) *Client {
	if addr == "" {
		return nil
	}
	return &Client{
		base: "http://" + addr,
		http: &http.Client{Timeout: 10 * time.Second},
	}
}

// Session is what the compute server hands down.
type Session struct {
	TankID     string      `json:"tankId"`
	Generation string      `json:"generation"`
	Config     wire.Config `json:"config"`
}

// Pull fetches the configuration for the active tank.
func (c *Client) Pull(ctx context.Context) (*Session, error) {
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, c.base+"/api/sbc/config", nil)
	if err != nil {
		return nil, err
	}
	res, err := c.http.Do(req)
	if err != nil {
		return nil, fmt.Errorf("pulling config: %w", err)
	}
	defer res.Body.Close()

	if res.StatusCode == http.StatusPreconditionFailed {
		return nil, ErrNoSession
	}
	if res.StatusCode >= 300 {
		msg, _ := io.ReadAll(io.LimitReader(res.Body, 2048))
		return nil, fmt.Errorf("pulling config: %s: %s", res.Status, msg)
	}

	var s Session
	if err := json.NewDecoder(res.Body).Decode(&s); err != nil {
		return nil, fmt.Errorf("decoding config: %w", err)
	}
	if s.Generation == "" {
		return nil, errors.New("compute server returned a config with no generation")
	}
	return &s, nil
}

// PullWithRetry keeps asking until it succeeds or the context ends.
//
// Boot order between the robot and the compute server is not guaranteed, and a
// tether takes time to come up. Retrying is the difference between "powered on
// in the wrong order" being a non-event and being a site visit.
//
// ErrNoSession is NOT retried away silently: it is reported to the caller each
// time, because it means a human has to choose a tank and the operator should
// see that rather than watching a spinner.
func (c *Client) PullWithRetry(ctx context.Context, every time.Duration, onWait func(error)) (*Session, error) {
	for {
		s, err := c.Pull(ctx)
		if err == nil {
			return s, nil
		}
		if onWait != nil {
			onWait(err)
		}
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		case <-time.After(every):
		}
	}
}
