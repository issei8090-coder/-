package main

import (
	"crypto/sha1"
	"encoding/base64"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net/http"
	"sync"
	"time"
)

// ================================================================================
// データ構造定義（バックエンド・物理マスター）
// ================================================================================

type SignalAspect string

const (
	SignalR  SignalAspect = "R"
	SignalYY SignalAspect = "YY"
	SignalY  SignalAspect = "Y"
	SignalYG SignalAspect = "YG"
	SignalG  SignalAspect = "G"
)

type GateStatus string

const (
	GateOpen   GateStatus = "OPEN"
	GateClosed GateStatus = "CLOSED"
)

type TrainState string

const (
	StateReady         TrainState = "出庫待機"
	StateRunning       TrainState = "本線走行中"
	StateStationStop   TrainState = "駅停車中"
	StateDeadheadToYrd TrainState = "留置線回送中"
	StateYardStop      TrainState = "留置線エンド交換中"
	StateDeadheadToPlt TrainState = "ホーム据付中"
	StateFinished      TrainState = "運用終了"
)

type Train struct {
	ID           string     `json:"id"`
	Type         string     `json:"type"`
	Cars         int        `json:"cars"`
	Length       float64    `json:"length"`
	MaxSpeed     float64    `json:"max_speed"`
	Acceleration float64    `json:"acceleration"`
	Deceleration float64    `json:"deceleration"`
	Position     float64    `json:"position"`
	CurrentSpeed float64    `json:"speed"`
	TargetSpeed  float64    `json:"target_speed"`
	CurrentNotch string     `json:"notch"`
	State        TrainState `json:"state"`
	StateTimer   int        `json:"state_timer"`
	TargetTrack  int        `json:"target_track"`
	IsShuttle    bool       `json:"is_shuttle"`
	IsDeadNotch  bool       `json:"is_dead_notch"`
}

func NewKeikyuTrain(id, tType string, cars int, maxSp, acc, dec, pos float64, startTime int, track int) *Train {
	length := (18.5 * 2.0) + (18.0 * float64(cars-2)) // 先頭18.5m/中間18.0m
	return &Train{
		ID: id, Type: tType, Cars: cars, Length: length, MaxSpeed: maxSp,
		Acceleration: acc, Deceleration: dec, Position: pos, State: StateReady,
		StateTimer: startTime, TargetTrack: track,
	}
}

type Simulation struct {
	TimeSec             int
	CrossingGate        GateStatus
	TrackShinagawaPlat1 string
	TrackShinagawaPlat2 string
	TrackShinagawaPlat3 string
	TrackYardA          string
	TurnoutSwitchLock   string
	Signal2ndHome       SignalAspect
	Signal3rdHome       SignalAspect
	Signal1stHome       SignalAspect
}

type ClientPacket struct {
	Time         string `json:"time"`
	CrossingGate string `json:"crossing_gate"`
	TurnoutLock  string `json:"turnout_lock"`
	Signals      struct {
		Home3rd string `json:"home_3rd"`
		Home2nd string `json:"home_2nd"`
		Home1st string `json:"home_1st"`
	} `json:"signals"`
	Trains []*Train `json:"trains"`
}

// ================================================================================
// 物理演算 ＆ C-ATS ＆ 勾配抵抗数理ロジック
// ================================================================================

func kmhToMs(kmh float64) float64 { return kmh / 3.6 }
func msToKmh(ms float64) float64  { return ms * 3.6 }

func (t *Train) CalculateTargetSpeed(sim *Simulation) {
	pos := t.Position
	limitSpeed := t.MaxSpeed

	// 12両編成有効長チェック（panicを避けログ出力と停止制御に変更）
	if t.Cars == 12 && t.TargetTrack == 3 {
		log.Printf("[WARN] 列車 %s: 12両編成有効長不足（Track 3）のため停止指示", t.ID)
		t.TargetSpeed = 0
		return
	}

	if t.State == StateDeadheadToYrd || t.State == StateDeadheadToPlt {
		t.TargetSpeed = kmhToMs(15.0) // 15km/h制限
		return
	}

	// 25‰勾配区間における制動延伸計算
	baseDecMs2 := kmhToMs(t.Deceleration)
	effectiveDecMs2 := baseDecMs2
	if pos >= 2300 && pos <= 3700 {
		effectiveDecMs2 = baseDecMs2 - 0.245
	}

	if pos >= 1150 && pos < 1620 {
		if limitSpeed > 40 {
			limitSpeed = 40
		}
	}
	if pos >= 1620 && pos < 1800 {
		if limitSpeed > 25 {
			limitSpeed = 25
		}
	}

	// 第3場内引き付け
	if pos < 1480 {
		var sigLimit float64 = 105
		switch sim.Signal3rdHome {
		case SignalR:
			sigLimit = 0
		case SignalYY:
			sigLimit = 25
		case SignalY:
			sigLimit = 45
		case SignalYG:
			sigLimit = 70
		case SignalG:
			sigLimit = t.MaxSpeed
		}
		dist := 1480.0 - pos
		if dist > 0 {
			allowed := math.Sqrt(math.Pow(kmhToMs(sigLimit), 2) + 2*effectiveDecMs2*dist)
			if sigLimit == 0 || sigLimit == 25 {
				t.IsDeadNotch = dist > 150.0
			} else {
				t.IsDeadNotch = false
			}
			if limitSpeed > msToKmh(allowed) {
				limitSpeed = msToKmh(allowed)
			}
		}
	}

	// 第2場内引き付け
	if pos >= 1480 && pos < 1620 {
		var sigLimit float64 = 105
		switch sim.Signal2ndHome {
		case SignalR:
			sigLimit = 0
		case SignalYY:
			sigLimit = 25
		case SignalY:
			sigLimit = 45
		case SignalG:
			sigLimit = t.MaxSpeed
		}
		isBlocked := false
		if t.TargetTrack == 2 && sim.TrackShinagawaPlat2 != "" && sim.TrackShinagawaPlat2 != t.ID {
			isBlocked = true
		}
		if sim.TurnoutSwitchLock != "" && sim.TurnoutSwitchLock != t.ID {
			isBlocked = true
		}
		distToStop := 1800.0 - pos
		if isBlocked && distToStop > 0 {
			allowed := math.Sqrt(2 * effectiveDecMs2 * distToStop)
			if limitSpeed > msToKmh(allowed) {
				limitSpeed = msToKmh(allowed)
			}
		} else {
			distToSignal := 1620.0 - pos
			if distToSignal > 0 {
				allowed := math.Sqrt(math.Pow(kmhToMs(sigLimit), 2) + 2*effectiveDecMs2*distToSignal)
				if limitSpeed > msToKmh(allowed) {
					limitSpeed = msToKmh(allowed)
				}
			}
		}
	}
	t.TargetSpeed = kmhToMs(limitSpeed)
}

func (t *Train) UpdatePhysics() {
	if t.State == StateStationStop || t.State == StateYardStop || t.State == StateFinished || t.State == StateReady {
		t.CurrentSpeed = 0
		t.CurrentNotch = "常用 (B8)"
		return
	}
	oldSpeed := t.CurrentSpeed
	accMps2 := kmhToMs(t.Acceleration)
	decMps2 := kmhToMs(t.Deceleration)
	gravityAcc := 0.0
	if t.Position >= 2300 && t.Position <= 3700 {
		gravityAcc = 0.245
	}

	if t.IsDeadNotch {
		t.CurrentNotch = "見越し惰行 (DN)"
		t.CurrentSpeed += gravityAcc
	} else if t.CurrentSpeed < t.TargetSpeed-0.2 {
		t.CurrentSpeed += (accMps2 + gravityAcc)
		if t.CurrentSpeed > t.TargetSpeed {
			t.CurrentSpeed = t.TargetSpeed
		}
		t.CurrentNotch = "力行 (P5)"
	} else if t.CurrentSpeed > t.TargetSpeed+0.2 {
		t.CurrentSpeed -= (decMps2 - gravityAcc)
		if t.CurrentSpeed < t.TargetSpeed {
			t.CurrentSpeed = t.TargetSpeed
		}
		t.CurrentNotch = "制動 (B5)"
	} else {
		t.CurrentNotch = "惰行 (N)"
		t.CurrentSpeed += gravityAcc
		if t.CurrentSpeed > t.TargetSpeed {
			t.CurrentSpeed = t.TargetSpeed
		}
	}
	if t.CurrentSpeed < 0 {
		t.CurrentSpeed = 0
	}
	avgSpeed := (oldSpeed + t.CurrentSpeed) / 2.0
	if t.State == StateDeadheadToPlt {
		t.Position -= avgSpeed
	} else {
		t.Position += avgSpeed
	}
}

func (sim *Simulation) UpdateSignals() {
	if sim.TrackShinagawaPlat2 != "" || (sim.TurnoutSwitchLock != "" && sim.TurnoutSwitchLock != sim.TrackShinagawaPlat2) {
		sim.Signal2ndHome = SignalR
	} else {
		sim.Signal2ndHome = SignalG
	}

	if sim.CrossingGate == GateOpen {
		if sim.Signal2ndHome == SignalR {
			sim.Signal3rdHome = SignalYY
		} else {
			sim.Signal3rdHome = SignalY
		}
	} else if sim.Signal2ndHome == SignalR {
		sim.Signal3rdHome = SignalYY
	} else {
		if sim.TrackYardA != "" {
			sim.Signal3rdHome = SignalYG
		} else {
			sim.Signal3rdHome = SignalG
		}
	}

	if sim.TrackYardA != "" {
		sim.Signal1stHome = SignalY
	} else {
		sim.Signal1stHome = SignalG
	}
}

// ================================================================================
// WebSocket通信・サーバー配信エンジン
// ================================================================================

var (
	clients   = make(map[chan []byte]bool)
	clientsMu sync.Mutex
)

// 標準WebSocketハンドシェイクキー生成
func computeAcceptKey(key string) string {
	h := sha1.New()
	h.Write([]byte(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"))
	return base64.StdEncoding.EncodeToString(h.Sum(nil))
}

func wsHandler(w http.ResponseWriter, r *http.Request) {
	hj, ok := w.(http.Hijacker)
	if !ok {
		return
	}
	conn, bufrw, err := hj.Hijack()
	if err != nil {
		return
	}
	defer conn.Close()

	key := r.Header.Get("Sec-WebSocket-Key")
	if key == "" {
		return
	}

	acceptKey := computeAcceptKey(key)
	bufrw.WriteString("HTTP/1.1 101 Switching Protocols\r\n" +
		"Upgrade: websocket\r\n" +
		"Connection: Upgrade\r\n" +
		"Sec-WebSocket-Accept: " + acceptKey + "\r\n\r\n")
	bufrw.Flush()

	ch := make(chan []byte, 10)
	clientsMu.Lock()
	clients[ch] = true
	clientsMu.Unlock()
	defer func() {
		clientsMu.Lock()
		delete(clients, ch)
		clientsMu.Unlock()
	}()

	for msg := range ch {
		bufrw.Write([]byte{0x81, byte(len(msg))})
		bufrw.Write(msg)
		if err := bufrw.Flush(); err != nil {
			return
		}
	}
}

func formatTime(totalSec int) string {
	return fmt.Sprintf("%02d:%02d:%02d", totalSec/3600, (totalSec%3600)/60, totalSec%60)
}

func main() {
	sim := &Simulation{TimeSec: 28800, CrossingGate: GateClosed, Signal1stHome: SignalG}
	trains := []*Train{
		NewKeikyuTrain("TRAIN_01", "普通(703B)", 8, 105, 3.5, 4.0, 0, 120, 2),
		NewKeikyuTrain("TRAIN_02", "急行(711T)", 8, 110, 3.3, 3.5, -200, 240, 2),
		NewKeikyuTrain("TRAIN_03", "MW2号", 12, 120, 3.5, 4.0, -400, 360, 2),
	}

	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		http.ServeFile(w, r, "index.html")
	})
	http.HandleFunc("/ws", wsHandler)

	go func() {
		for {
			time.Sleep(1 * time.Second)
			sim.TimeSec++
			if sim.TimeSec >= 28980 && sim.TimeSec <= 29010 {
				sim.CrossingGate = GateOpen
			} else {
				sim.CrossingGate = GateClosed
			}
			sim.UpdateSignals()

			for _, t := range trains {
				switch t.State {
				case StateReady:
					t.StateTimer--
					if t.StateTimer <= 0 {
						t.State = StateRunning
					}
				case StateRunning:
					if t.Position >= 1800 && t.Position < 2040 {
						t.Position = 1850
						t.State = StateStationStop
						if t.IsShuttle {
							t.StateTimer = 180
							sim.TrackShinagawaPlat3 = t.ID
						} else {
							t.StateTimer = 60
							sim.TrackShinagawaPlat2 = t.ID
						}
					} else if t.Position >= 2500 {
						t.State = StateFinished
					}
				case StateStationStop:
					t.StateTimer--
					if t.ID == "TRAIN_01" && sim.TimeSec == 29070 && !t.IsShuttle {
						t.StateTimer += 30
					}
					if t.StateTimer <= 0 {
						if t.IsShuttle {
							t.State = StateFinished
							sim.TrackShinagawaPlat3 = ""
						} else if t.ID == "TRAIN_01" {
							t.State = StateDeadheadToYrd
							sim.TrackShinagawaPlat2 = ""
							sim.TurnoutSwitchLock = t.ID
						} else {
							t.State = StateRunning
							sim.TrackShinagawaPlat2 = ""
							t.Position = 2050
						}
					}
				case StateDeadheadToYrd:
					if t.Position < 1800.0+50.0+t.Length {
						t.CurrentSpeed = kmhToMs(15.0)
					} else {
						sim.TurnoutSwitchLock = ""
						t.State = StateYardStop
						t.StateTimer = 60
						sim.TrackYardA = t.ID
					}
				case StateYardStop:
					t.StateTimer--
					if t.StateTimer <= 0 {
						t.State = StateDeadheadToPlt
						sim.TrackYardA = ""
						sim.TurnoutSwitchLock = t.ID
					}
				case StateDeadheadToPlt:
					if t.Position > 1850.0 {
						t.CurrentSpeed = kmhToMs(15.0)
					} else {
						sim.TurnoutSwitchLock = ""
						t.State = StateFinished
						sim.TrackShinagawaPlat1 = t.ID
					}
				}

				if t.State == StateRunning || t.State == StateDeadheadToYrd || t.State == StateDeadheadToPlt {
					t.CalculateTargetSpeed(sim)
					t.UpdatePhysics()
				}
			}

			var packet ClientPacket
			packet.Time = formatTime(sim.TimeSec)
			packet.CrossingGate = string(sim.CrossingGate)
			packet.TurnoutLock = sim.TurnoutSwitchLock
			packet.Signals.Home3rd = string(sim.Signal3rdHome)
			packet.Signals.Home2nd = string(sim.Signal2ndHome)
			packet.Signals.Home1st = string(sim.Signal1stHome)
			packet.Trains = trains

			if msg, err := json.Marshal(packet); err == nil {
				clientsMu.Lock()
				for ch := range clients {
					select {
					case ch <- msg:
					default:
					}
				}
				clientsMu.Unlock()
			}
		}
	}()

	fmt.Println("📢 指令室サーバーが起動しました。[http://localhost:8080] をブラウザで開いてください。")
	log.Fatal(http.ListenAndServe(":8080", nil))
}
