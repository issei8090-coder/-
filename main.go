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
	length := (18.5 * 2.0) + (18.0 * float64(cars-2))
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

	// 有効長オーバーの安全制御
	if t.Cars == 12 && t.TargetTrack == 3 {
		t.TargetSpeed = 0
		return
	}

	if t.State == StateDeadheadToYrd || t.State == StateDeadheadToPlt {
		t.TargetSpeed = kmhToMs(15.0)
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

func computeAcceptKey(key string) string {
	h := sha1.New()
	h.Write([]byte(key + "258EAFA5-E914-47
