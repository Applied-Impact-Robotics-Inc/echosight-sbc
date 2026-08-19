package api

import (
	"encoding/json"
	"net/http"
	"time"

	"github.com/gorilla/websocket"

	"echosight/internal/state"
)

// The renderer's origin is "null" when the packaged Electron app loads from
// file://, and http://localhost:5173 in dev. Gorilla rejects both by default,
// which is the single most common reason this refuses to connect. There is no
// browser-enforced boundary worth defending here: the server sits on an
// isolated robot network behind the tether.
var upgrader = websocket.Upgrader{
	ReadBufferSize:  4096,
	WriteBufferSize: 1 << 20,
	CheckOrigin:     func(r *http.Request) bool { return true },
}

const (
	writeWait      = 5 * time.Second
	pongWait       = 30 * time.Second
	subBuffer      = 16 // sweeps queued per client before drop-oldest kicks in
	maxMessageSize = 4096
)

func (s *Server) handleWS(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		s.store.Logf("warn", "ws upgrade failed: "+err.Error())
		return
	}
	sub := s.broker.Subscribe(subBuffer)
	s.store.Update(func(sn *state.Snapshot) { sn.Clients = s.broker.Count() })
	s.store.Logf("info", "ws client connected: "+conn.RemoteAddr().String())

	done := make(chan struct{})
	// gorilla permits exactly one concurrent writer, so the reader never writes.
	// Pong replies are handed to the write loop instead.
	pongs := make(chan uint32, 4)

	// Reader: application-level ping from the frontend, plus the close signal.
	// A tethered TCP socket can half-open without ever firing onclose, so the
	// frontend heartbeats and we answer.
	go func() {
		defer close(done)
		conn.SetReadLimit(maxMessageSize)
		_ = conn.SetReadDeadline(time.Now().Add(pongWait))
		conn.SetPongHandler(func(string) error {
			return conn.SetReadDeadline(time.Now().Add(pongWait))
		})
		for {
			_, msg, err := conn.ReadMessage()
			if err != nil {
				return
			}
			_ = conn.SetReadDeadline(time.Now().Add(pongWait))
			var m struct {
				T  string `json:"t"`
				ID uint32 `json:"id"`
			}
			if json.Unmarshal(msg, &m) != nil {
				continue
			}
			if m.T == "ping" {
				select {
				case pongs <- m.ID:
				default: // heartbeat backlog means the writer is already wedged
				}
			}
		}
	}()

	// Send the current status immediately so the UI is populated before the
	// next 1 Hz tick.
	s.sendSnapshot(conn)

	for {
		select {
		case <-done:
			s.broker.Unsubscribe(sub)
			_ = conn.Close()
			s.store.Update(func(sn *state.Snapshot) { sn.Clients = s.broker.Count() })
			s.store.Logf("info", "ws client gone: "+conn.RemoteAddr().String())
			return

		case id := <-pongs:
			reply, _ := json.Marshal(map[string]any{"t": "pong", "id": id})
			_ = conn.SetWriteDeadline(time.Now().Add(writeWait))
			if conn.WriteMessage(websocket.TextMessage, reply) != nil {
				s.broker.Unsubscribe(sub)
				_ = conn.Close()
				return
			}

		case f := <-sub.C:
			typ := websocket.TextMessage
			if f.Binary {
				typ = websocket.BinaryMessage
			}
			_ = conn.SetWriteDeadline(time.Now().Add(writeWait))
			if err := conn.WriteMessage(typ, f.Data); err != nil {
				s.broker.Unsubscribe(sub)
				_ = conn.Close()
				return
			}
		}
	}
}

func (s *Server) sendSnapshot(conn *websocket.Conn) {
	st := s.sup.State()
	if st.Status == nil {
		return
	}
	b, err := json.Marshal(statusEnvelope(*st.Status))
	if err != nil {
		return
	}
	_ = conn.SetWriteDeadline(time.Now().Add(writeWait))
	_ = conn.WriteMessage(websocket.TextMessage, b)
}
