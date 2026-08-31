# S2M-CommRelay

**Scout2Map** — 다중 센서 기반 환경 적응형 정찰 UGV의 통신 중계 레이어다. `S2M-Event-Engine`이 발행하는 `/events`를 Web-Monitoring 쪽으로 넘기되, **네트워크가 끊겼다가 다시 붙어도 그 사이에 놓친 이벤트가 유실되지 않도록** 버퍼링하는 게 이 레포의 핵심 역할이다. 여기에 더해 SLAM 지도, 로봇 위치(pose), 배터리/하드웨어 텔레메트리를 같은 연결로 함께 relay하고, 웹에서 들어오는 드라이브·미션 제어 명령을 ROS2로 중계한다.

rosbridge_suite를 쓰지 않는다. rosbridge는 pub/sub을 WebSocket으로 중계만 할 뿐 끊긴 동안의 메시지를 들고 있다가 재전송하는 기능이 없어서, 이벤트처럼 "놓치면 안 되는" 데이터에는 맞지 않다. 또한 원격 관제에서 로봇을 직접 제어(E-Stop, 미션 시작/종료)하려면 클라이언트 → 서버 방향 명령 채널도 필요한데 이 역시 rosbridge 범용 프로토콜보다는 커스텀 레이어가 다루기 쉽다. 대신 `comm_relay_node`가 자체 WebSocket 프로토콜로 버퍼링·재전송·연결 상태 보고·명령 처리까지 직접 처리한다.

`/map`(nav_msgs/OccupancyGrid)도 같은 노드, 같은 WebSocket 연결로 함께 relay한다. 다만 지도는 이벤트와 성격이 달라서(최신 값만 중요하고 끊겨도 다시 받으면 그만) **버퍼링은 하지 않는다.** 대신 대역폭을 아끼려고 zlib 압축 + 주기 제한(throttle)을 건다.

로봇 위치(TF 기반 pose)와 배터리/하드웨어 텔레메트리(`/drive/battery`, `/drive/status`) 역시 최신 값만 중요한 best-effort 데이터라 버퍼링하지 않고 주기적으로만 broadcast한다. raw 센서 스트림은 아직 스코프 밖이다.

## 아키텍처

```
S2M-Event-Engine                 S2M-CommRelay                  Web-Monitoring
  /events (String, JSON)  ──►  comm_relay_node
                                  ├─ 클라이언트 연결됨 → 즉시 전달
                                  ├─ 클라이언트 없음   → 버퍼에 적재
                                  └─ 재연결 시         → 버퍼 순서대로 재전송
SLAM/AMCL                                │
  /map (OccupancyGrid)   ──►            │  (버퍼링 없음, latest-wins,
                                         │   zlib 압축 + 주기 제한)
TF (map → base_link)     ──►            │  (버퍼링 없음, pose_relay_period_s 주기)
                                         │
  /drive/battery (BatteryState) ──►     │  (버퍼링 없음, telemetry_relay_period_s
  /drive/status (DriveStatus)   ──►     │   주기, 클라이언트 연결 시에만)
                                       ▼
                                ws://<sbc-ip>:9091  ◄──────────►  new WebSocket(...)
                                /relay/link_status (String, JSON, 로컬 ROS 헤르트비트)
                                /control/heartbeat (Empty, 웹 클라이언트 연결 시에만
                                 heartbeat_period_s 주기로 발행 → event_engine, return_home)

                                  ▲ command 프레임 (estop / clear_fault / reset_odom /
                                  │  launch_mission / stop_mission)
                                  │
                    /drive/estop, /drive/clear_fault,     explore_lite            /return_home/trigger,
                    /drive/reset_odom (Trigger 서비스)     (ros2 launch subprocess) /return_home/arm
                                                                                    (Trigger/SetBool 서비스,
                                                                                    return_home_node는 별도
                                                                                    launch로 이미 떠 있음)
```

## 왜 커스텀 프로토콜인가

roslibjs/rosbridge 방식이 아니라 브라우저 기본 `WebSocket` API만으로 붙을 수 있는 얕은 JSON 프로토콜을 직접 정의했다. 이유:

- 재연결 시 "얼마나 끊겨 있었는지", "몇 개가 밀렸는지", "몇 개가 버려졌는지"를 클라이언트 접속 시점에 알려줘야 하는데, 이건 rosbridge 프로토콜에 없는 개념이라 어차피 얹으려면 커스텀 레이어가 필요하다.
- 웹에서 E-Stop·미션 제어처럼 로봇에 직접 작용하는 명령을 보내야 하는데, rosbridge의 임의 서비스 호출 노출면(화이트리스트 설정, 인증 부재)을 그대로 열어두는 것보다 필요한 명령만 좁게 받는 커스텀 채널이 안전하다.
- `/events` 하나만 다루던 초기 단계에서는 rosbridge의 범용성(임의 토픽/서비스 노출)이 필요 없었고, 지금도 노출 대상을 명시적으로 제한하고 싶다는 이유는 그대로 유효하다.

## 메시지 프로토콜

**서버 → 클라이언트**는 status/event/map/pose/telemetry를 계속 push한다. **클라이언트 → 서버**는 처음에는 무시했지만, 지금은 드라이브·미션 제어 명령(`kind: "command"`)을 받아서 처리한다 — 더 이상 완전한 단방향 프로토콜이 아니다.

### 서버 → 클라이언트

**접속 직후, 상태 안내 1회:**
```json
{
  "kind": "status",
  "state": "ok",
  "reconnected": true,
  "offline_s": 42.3,
  "buffered_count": 7,
  "dropped_count": 0
}
```
`reconnected`가 `false`면 이 세션이 이 서버에 처음 붙은 경우(재연결 아님), `offline_s`는 그때 `null`이다.

**이벤트 (버퍼에 있던 것 재전송 + 실시간 둘 다 같은 포맷):**
```json
{
  "kind": "event",
  "seq": 118,
  "replay": true,
  "relayed_at": 1755999999.12,
  "data": { "...": "S2M-Event-Engine의 /events payload 원본" }
}
```
`replay: true`면 끊긴 동안 쌓였다가 재전송된 것, `false`면 실시간으로 온 것.

**지도 (JSON 메타 프레임 + 곧바로 이어지는 바이너리 프레임, 총 2개가 한 쌍):**
```json
{
  "kind": "map_meta",
  "seq": 42,
  "encoding": "zlib-int8-rowmajor-bottomleft",
  "resolution": 0.05,
  "width": 400,
  "height": 400,
  "origin": { "x": -10.0, "y": -10.0, "yaw": 0.0 },
  "raw_bytes": 160000,
  "compressed_bytes": 3421
}
```
이 프레임 바로 다음에 오는 바이너리 프레임이 실제 그리드 데이터다: `nav_msgs/OccupancyGrid.data`(int8, 값은 `-1`/`0`..`100`)를 부호 없는 바이트로 그대로 담고(`byte = value & 0xFF`, 즉 `-1` → `255`) zlib(RFC1950, `zlib.compress()`)로 압축한 것. 순서는 ROS 관례대로 row-major, row 0이 지도 좌표계의 아래쪽(가장 작은 y)이다. 클라이언트는 `DecompressionStream('deflate')`로 풀고, 각 바이트를 `byte > 127 ? byte - 256 : byte`로 되돌리면 원래 값이 나온다.

버퍼링 대상이 아니므로 클라이언트가 접속하는 순간 최신 지도가 있으면 바로 이 쌍을 한 번 보내주고, 그 뒤로는 `map_relay_period_s`마다(그 사이에 새 지도가 왔을 때만) 다시 보낸다.

**알려진 단순화**: `origin`의 회전(yaw)은 프레임에 실어 보내지만, 지금 프론트(`S2M-Web-Monitoring`의 `MapView`)는 회전이 0이라고 가정하고 위치 오프셋만 반영한다. 만약 origin이 회전되어 있으면 `comm_relay_node`가 ROS 로그에 경고를 남긴다.

**로봇 위치 (pose):**
```json
{ "kind": "pose", "x": 1.23, "y": -0.45, "yaw": 0.78 }
```
`map_frame` → `base_frame` TF를 `pose_relay_period_s`(기본 0.1초)마다 조회해서 broadcast한다. TF 조회에 실패하면(아직 SLAM/AMCL이 준비되지 않았거나 프레임이 없는 경우) 조용히 스킵한다. 버퍼링하지 않으므로 연결이 끊긴 동안의 pose는 재전송되지 않는다.

**배터리 상태 (drive_battery):**
```json
{
  "kind": "drive_battery",
  "data": { "percentage": 87.0, "voltage": 24.6, "warning_level": "Normal" }
}
```
`/drive/battery`(`sensor_msgs/BatteryState`) 구독값을 바탕으로 만든다. `warning_level`은 `percentage <= 10`이면 `Dead`, `<= 20`이면 `Warning`, 그 외엔 `Normal`이다.

**드라이브 하드웨어 상태 (drive_status):**
```json
{
  "kind": "drive_status",
  "data": {
    "link_ok": true,
    "mcu_reboot_count": 0,
    "cmd_timeout": false,
    "frames_ok": 15234,
    "crc_errors": 0,
    "batt_warn": false,
    "batt_critical": false,
    "batt_dead": false
  }
}
```
`/drive/status`(`scout2map_msgs/DriveStatus`) 구독값을 그대로 옮긴다.

**센서 MCU 상태 (sensor_mcu_status):**
```json
{
  "kind": "sensor_mcu_status",
  "data": {
    "port_open": true,
    "link_ok": true,
    "last_line_age_s": 0.12,
    "mcu_reboot_count": 0,
    "parse_errors": 0,
    "framing_overflows": 0,
    "aht21_present": true,
    "ens160_present": true,
    "bh1750_present": true,
    "pms7003_seen": true
  }
}
```
`/sensors/status`(`scout2map_msgs/SensorStatus`, Pico 센서-퓨전 MCU를 잇는 `sensor_bridge`가 발행)를 그대로 옮긴다. `*_present`/`pms7003_seen`은 MCU 부팅 배너로 초기화된 뒤 실제 데이터로 확정되는 값이라, 한 번도 값을 못 보낸 센서(2026-08-29 BH1750 미인식 사례처럼 배선/I2C 주소 문제로 부팅 시 초기화 자체가 실패한 경우)를 운영자가 `ros2 topic echo` 없이 바로 알아챌 수 있게 해준다. `EnvSnapshot`의 `*_valid` 플래그는 "값이 있는데 오래됐다"만 구분하고 "애초에 값이 온 적 없다"는 구분하지 못하므로, 이 상태는 `EnvSnapshot`이 아니라 `SensorStatus`에서만 확인 가능하다.

**비전 카메라 상태 (vision_camera_status):**
```json
{
  "kind": "vision_camera_status",
  "data": {
    "level": "WARN",
    "message": "waiting for camera frames",
    "frame_age_s": null,
    "last_latency_ms": null,
    "p95_latency_ms": null,
    "model_sha256": "adeea580ab..."
  }
}
```
`diagnostics_topic`(기본 `/diagnostics`, `diagnostic_msgs/DiagnosticArray`)를 구독하되, `vision_diagnostic_name`(기본 `scout_vision/inference`)과 이름이 일치하는 항목 하나만 골라서 옮긴다 — `/diagnostics`는 `ekf_filter_node` 등 다른 노드도 같이 쓰는 공용 버스라서다. `level`은 `OK`/`WARN`/`ERROR`/`STALE`/`UNKNOWN` 중 하나. `vision_node`가 카메라 프레임을 한 번도 못 받았을 때(`frame_age_s`가 원본 메시지에서 문자열 `"unknown"`인 경우) 세 숫자 필드는 전부 `null`로 relay된다 — 값이 느린 게 아니라 아예 안 들어온 것과, 느리게 들어오는 것(STALE)을 구분하기 위함이다(2026-08-29 카메라 프레임 미수신 사례).

배터리/드라이브/센서 MCU/비전 카메라 상태 넷 다 `telemetry_relay_period_s`(기본 10초)마다, **클라이언트가 하나 이상 연결되어 있을 때만** 최신 값을 broadcast한다(버퍼링 없음, latest-wins). 링크가 약해지면 이 주기 자체가 `telemetry_relay_period_s_degraded`로 늘어난다 - 아래 `link_quality` 참고.

**링크 품질 (link_quality) - 신규:**
```json
{
  "kind": "link_quality",
  "tier": "degraded",
  "rtt_ms": 612.4,
  "message": "통신 품질 저하 - 텔레메트리 갱신 주기와 비전 인식 해상도를 낮췄습니다"
}
```
`link_quality_check_period_s`(기본 2초)마다 연결된 웹 클라이언트에 WebSocket ping을 보내 RTT를 측정한다(websockets 라이브러리 자체의 keepalive ping과는 별개 - 그건 연결이 죽었는지만 감지하고 RTT는 안 알려준다). `link_rtt_degraded_ms`(기본 800ms)를 `link_quality_hysteresis_checks`(기본 2회) 연속으로 넘기면 `degraded`로 전환하면서 `scout_vision`에 `LINK_QUALITY_PROFILES['degraded']`(`comm_relay_node.py`) 값을 자동으로 밀어넣는다 - `max_fps`를 낮추고 `/vision/snapshots`의 크롭·풀프레임 이미지를 더 작고 낮은 품질로 인코딩하게 한다. `link_rtt_good_ms`(기본 250ms) 밑으로 같은 횟수만큼 연속 회복하면 `good` 프로필(원래 값)로 되돌린다. 두 임계값 사이(250~800ms)는 "데드존"이라 티어가 바뀌지 않는다 - 경계값 근처에서 매번 왔다갔다(flapping)하는 걸 막기 위함. 웹 클라이언트가 하나도 없으면 측정 자체를 건너뛰고, 마지막 클라이언트가 끊기는 순간 티어는 조용히 `good`으로 리셋된다(다음 접속자는 항상 정상 화질로 시작해서 새로 측정됨).

`ros2 topic echo /relay/link_status`(아래 `status` 프레임과는 별개의 ROS 토픽)에도 `quality_tier`/`quality_rtt_ms` 필드로 같이 노출되니, 웹 클라이언트 없이도 SSH로 현재 티어를 확인할 수 있다.

**미션 상태 (mission_status):**
```json
{ "kind": "mission_status", "mission": "EXPLORE" }
```
`launch_mission`/`stop_mission` 명령을 처리한 직후 결과 상태(`IDLE` / `EXPLORE` / `RETURN`)를 broadcast한다.

**복귀 목표 지점 (return_home_goal):**
```json
{ "kind": "return_home_goal", "x": 1.5, "y": -2.25, "yaw": 1.5708 }
```
`return_home_pose_topic`(기본 `/return_home/start_pose`, `geometry_msgs/PoseStamped`)을 구독해서 만든다. `return_home_node`가 출발 지점을 캡처할 때 한 번 발행하고 그 뒤로는 바뀌지 않는다 — 실제로 복귀 시 Nav2에 보내는 목표도 이 캡처된 시작 위치 그대로다(별도의 "현재 목표"가 따로 있는 게 아니다). 토픽이 `TRANSIENT_LOCAL`(latched)이라 `map`처럼 최근값을 캐싱해뒀다가 새로 붙는 클라이언트에게 바로 보내준다.

**복귀 상태 (return_home_status):**
```json
{ "kind": "return_home_status", "state": "RETURNING", "start_captured": true }
```
`return_home_status_topic`(기본 `/return_home/status`, `std_msgs/String` JSON)을 구독해서 `state`/`start_captured` 두 필드만 추려 relay한다. 원본 JSON에는 `armed`, `heartbeat_age_sec` 등 더 많은 필드가 있지만 지도 마커 렌더링에는 이 둘이면 충분해서 나머지는 걸러낸다. `state`는 `WAITING_FOR_START` / `START_POSE_CAPTURED` / `NORMAL` / `RETURN_REQUESTED` / `RETURNING` / `ARRIVED` / `SAFE_STOP` 중 하나다. 원본 페이로드가 JSON으로 파싱되지 않으면 경고 로그만 남기고 이전 캐시 값을 그대로 유지한다(깨진 프레임 하나 때문에 화면이 빈 값으로 덮이지 않도록). `return_home_goal`과 마찬가지로 latched라 새로 붙는 클라이언트에게 최근값을 바로 보내준다.

### 클라이언트 → 서버 (command)

`kind: "command"` 프레임을 받아서 드라이브·미션 제어에 반영한다.

```json
{ "kind": "command", "command": "estop" }
{ "kind": "command", "command": "launch_mission", "mission": "explore" }
{ "kind": "command", "command": "stop_mission", "mission": "return_home" }
```

| `command` | 동작 |
|---|---|
| `estop` | `/drive/estop` (`std_srvs/Trigger`) 비동기 호출 |
| `clear_fault` | `/drive/clear_fault` (`std_srvs/Trigger`) 비동기 호출 |
| `reset_odom` | `/drive/reset_odom` (`std_srvs/Trigger`) 비동기 호출 |
| `launch_mission` (`mission: explore`) | `ros2 launch explore_lite explore.launch.py` subprocess 시작, 이미 실행 중이면 무시하고 경고 로그 |
| `launch_mission` (`mission: return_home`) | `/return_home/trigger`(`std_srvs/Trigger`) 비동기 호출 — subprocess를 새로 띄우는 게 아니라, 이미 `s2m_slam_real.launch.py use_return_home:=true`로 떠 있는 `return_home_node`에게 "지금 복귀 시작"을 지시하는 것이다. 응답이 `success: false`면 경고 로그만 남기고 넘어간다 |
| `stop_mission` (`mission: explore`) | `explore_lite` subprocess `terminate()`(핸들이 있으면) + `pkill -f explore_lite`(핸들이 없거나 stale해도 잡는 fallback) + `explore_cmd_vel_topic`(기본 `/cmd_vel`)에 0속도 `Twist` 발행 + `navigate_action`(기본 `/navigate_to_pose`)의 `_action/cancel_goal` 서비스로 진행 중인 내비게이션 goal 취소 |
| `stop_mission` (`mission: return_home`) | `/return_home/arm`(`std_srvs/SetBool`, `data: false`) 비동기 호출로 disarm — `return_home_node`가 안전 정지 처리 |

서비스가 0.5초 안에 응답 가능 상태가 아니면(`wait_for_service` 타임아웃) 경고만 로그로 남기고 넘어간다 — 호출 실패가 클라이언트에게 별도로 통보되지는 않는다.

`stop_mission`(explore)이 goal 취소까지 하는 이유: `explore_lite` subprocess를 죽여도 그게 이미 `bt_navigator`에 보낸 `NavigateToPose` goal은 취소되지 않는다 — action goal의 생명주기는 서버(`bt_navigator`) 쪽이 갖고 있어서, 보낸 클라이언트가 사라져도 goal은 계속 실행/재계획된다. `controller_server`가 `stop_mission`을 받은 뒤에도 계속 `Passing new path to controller`를 찍으면서 로봇이 안 멈추는 게 바로 이 증상이다(2026-08-29). `comm_relay`가 직접 보낸 goal이 없어서(explore_lite/return_home이 각자 보냄) 일반적인 `rclpy.action.ActionClient`로는 취소할 수 없고, 대신 모든 action 서버가 갖고 있는 `<action>/_action/cancel_goal`(`action_msgs/srv/CancelGoal`) 서비스를 직접 호출한다 — `goal_id`/`timestamp`를 비운 요청은 서버에 떠 있는 goal을 전부 취소한다는 게 액션 프로토콜 표준 규약이다.

## 프론트엔드 최소 예시

```javascript
const ws = new WebSocket("ws://<sbc-ip>:9091");
ws.onmessage = (ev) => {
  const msg = JSON.parse(ev.data);
  if (msg.kind === "status") {
    console.log(`reconnected=${msg.reconnected}, offline ${msg.offline_s}s, buffered ${msg.buffered_count}`);
  } else if (msg.kind === "event") {
    const event = msg.data; // { id, type, state, level, x, y, yaw, coordinate_status, ... }
    // event.state === 'raised' -> 마커 추가/갱신, 'cleared' -> 마커 제거
  }
};

// 드라이브/미션 명령 전송
ws.send(JSON.stringify({ kind: "command", command: "estop" }));
```

## 관제망 heartbeat (`/control/heartbeat`)

`S2M-Event-Engine`(`COMM_DEGRADED`/`COMM_LOST`)과 `return_home_node`(자동 복귀/안전
정지 판단)가 둘 다 `bridge-interface-contract.md`에 문서화된 대로 이 토픽을
구독하지만, 실제 하드웨어에서 발행하는 노드가 없어서 `return_home_node`가 armed
상태에 영영 못 들어가고 `use_return_home:=true`일 때 `cmd_vel`이 항상 막히는
문제가 있었다(2026-08-29). `comm_relay`가 문서상 "관제 서버"에 해당하는
브리지이므로, **웹 클라이언트가 실제로 연결돼 있는 동안에만** `heartbeat_period_s`
(기본 0.5초) 주기로 이 토픽을 발행한다.

일부러 `heartbeat_period_s`를 두 소비자의 타임아웃(`COMM_DEGRADED` 1.5초,
`return_home`의 `heartbeat_timeout_sec` 기본 3.0초)보다 훨씬 짧게 잡았다 —
발행 주기와 타임아웃이 딱 같으면 스케줄링 지터만으로도 오탐이 뜨는 걸
`return_home_node`의 `motion_inhibit` 레이스에서 이미 한 번 겪었다(`fcd1725`).

클라이언트 연결이 끊기면(관제망 단절) 이 토픽 발행도 즉시 멈춘다 — 이게 의도된
동작이다: 웹 클라이언트가 안 붙어 있으면 "관제망이 없다"는 게 맞고, 그래야
`event_engine`의 `COMM_LOST`와 `return_home`의 자동 복귀가 실제로 트리거된다.
벤치에서 관제 서버 없이 단독으로 테스트할 때는 `event_engine`의
`require_heartbeat_seen: true` 설정이 `COMM_LOST` 오탐을 막아준다(heartbeat를
한 번도 못 본 상태에서는 안 띄움).

## 파라미터 (`config/comm_relay.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `events_topic` | `/events` | 구독할 이벤트 토픽 |
| `ws_host` / `ws_port` | `0.0.0.0` / `9091` | WebSocket 서버 바인드 주소 |
| `buffer_max_len` | `500` | 클라이언트 없을 때 버퍼에 쌓을 최대 개수 |
| `buffer_max_age_s` | `1800.0` | 이보다 오래된 버퍼 항목은 버림 |
| `link_status_topic` | `/relay/link_status` | 로컬 ROS 연결 상태 헤르트비트 |
| `link_status_period_s` | `2.0` | 헤르트비트 발행 주기 |
| `heartbeat_topic` | `/control/heartbeat` | 웹 클라이언트가 실제로 붙어 있는 동안만 발행하는 관제망 heartbeat (`std_msgs/Empty`) |
| `heartbeat_period_s` | `0.5` | `heartbeat_topic` 발행 주기 |
| `ws_ping_interval_s` / `ws_ping_timeout_s` | `10.0` / `10.0` | WebSocket keepalive (반쯤 끊긴 연결 빨리 감지) |
| `map_topic` | `/map` | 구독할 지도 토픽 (nav_msgs/OccupancyGrid) |
| `map_relay_period_s` | `2.0` | 지도를 이 주기보다 자주 다시 보내지 않음 |
| `map_zlib_level` | `6` | 그리드 압축 레벨 (0-9) |
| `map_frame` | `map` | 로봇 위치 조회 기준 프레임 |
| `base_frame` | `base_link` | 로봇 위치 조회 대상 프레임 |
| `pose_relay_period_s` | `0.1` | 로봇 위치 broadcast 주기 |
| `battery_topic` | `/drive/battery` | 구독할 배터리 토픽 (`sensor_msgs/BatteryState`) |
| `drive_status_topic` | `/drive/status` | 구독할 드라이브 상태 토픽 (`scout2map_msgs/DriveStatus`) |
| `sensor_status_topic` | `/sensors/status` | 구독할 센서 MCU 상태 토픽 (`scout2map_msgs/SensorStatus`) |
| `diagnostics_topic` | `/diagnostics` | 구독할 공용 진단 토픽 (`diagnostic_msgs/DiagnosticArray`) |
| `vision_diagnostic_name` | `scout_vision/inference` | `diagnostics_topic`에서 골라낼 항목의 `name` |
| `telemetry_relay_period_s` | `10.0` | 배터리/드라이브/센서 MCU/비전 카메라 상태 broadcast 주기, 클라이언트 연결 시에만 |
| `return_home_pose_topic` | `/return_home/start_pose` | 구독할 복귀 시작 위치 토픽 (`geometry_msgs/PoseStamped`, latched) |
| `return_home_status_topic` | `/return_home/status` | 구독할 복귀 상태 토픽 (`std_msgs/String` JSON, latched) |
| `explore_cmd_vel_topic` | `/cmd_vel` | `stop_mission`(explore)에서 0속도 `Twist`를 발행할 토픽 |
| `navigate_action` | `/navigate_to_pose` | `stop_mission`(explore)에서 진행 중인 goal을 취소할 때 쓰는 NavigateToPose action 이름 |
| `link_quality_check_period_s` | `2.0` | 웹 클라이언트 ping RTT 측정 주기 |
| `link_rtt_good_ms` / `link_rtt_degraded_ms` | `250.0` / `800.0` | `good`/`degraded` 티어 전환 임계값 (사이는 데드존) |
| `link_quality_hysteresis_checks` | `2` | 티어를 실제로 전환하기까지 필요한 연속 확인 횟수 |
| `telemetry_relay_period_s_degraded` | `15.0` | `degraded` 티어일 때의 `telemetry_relay_period_s` |

## 빌드 & 실행

```bash
# websockets가 rosdep으로 안 풀리면 pip로 직접
pip install websockets --break-system-packages

cd ~/ros2_ws
colcon build --packages-select scout2map_comm
source install/setup.bash
ros2 launch scout2map_comm comm_relay.launch.py
```

의존 패키지: `rclpy`, `std_msgs`, `nav_msgs`, `geometry_msgs`(PoseStamped/Twist), `sensor_msgs`, `std_srvs`, `tf2_ros`, `scout2map_msgs`(DriveStatus/SensorStatus 메시지 정의), `diagnostic_msgs`, `websockets`(python3-websockets). 전부 `package.xml`에 `<depend>`로 선언돼 있어서 `rosdep install`로 해결된다.

## 테스트 방법

- `test/test_comm_relay_buffer.py`: 버퍼 적재/prune 로직 단위 테스트 (`colcon test --packages-select scout2map_comm`)
- `test/test_comm_relay_map.py`: 지도 zlib 압축, dirty-flag throttle, origin yaw 추출 단위 테스트
- 수동 확인: `event_engine`을 같이 띄운 상태에서 브라우저 콘솔이나 `wscat -c ws://<sbc-ip>:9091`로 접속해서 `status` 프레임이 오는지, 온도 임계값을 넘겨서 실제 `event` 프레임이 오는지 확인
- 버퍼링 확인: 클라이언트를 끊고 그 사이 이벤트를 몇 개 발생시킨 뒤 재접속 → `status`의 `buffered_count`와 실제로 온 `event` 프레임 개수가 맞는지, `replay: true`로 표시되는지 확인
- `ros2 topic echo /relay/link_status`로 로컬에서도 연결 상태를 볼 수 있다
- 지도 확인: SLAM/AMCL을 같이 띄운 상태에서 `ros2 topic hz /map`으로 실제 발행되는지 먼저 확인하고, 웹 화면에서 지도가 뜨는지, 로그에 `web client connected`가 찍힌 직후 최신 지도가 바로 오는지 확인
- **포즈 확인**: `map` → `base_link` TF가 발행 중인 상태에서 웹 화면의 로봇 마커가 실제 위치/헤딩과 맞게 움직이는지 확인
- **텔레메트리 확인**: `/drive/battery`, `/drive/status`를 발행한 뒤 웹 화면의 Hardware Telemetry 패널 값이 `telemetry_relay_period_s` 주기로 갱신되는지 확인
- **명령 채널 확인**: `wscat`으로 `{"kind":"command","command":"estop"}`를 보내고 `/drive/estop` 서비스가 실제로 호출되는지, `ros2 service list`에 해당 서비스가 떠 있어야 정상 응답하는지 확인. explore 미션은 `launch_mission`/`stop_mission` 전송 후 `ros2 node list`에 `explore_node`가 뜨고 내려가는지 확인하고, return_home 미션은 노드가 뜨고 내려가는 게 아니라 이미 떠 있는 `return_home` 노드에 `/return_home/trigger`·`/return_home/arm` 서비스가 호출되는지(`ros2 service call`로도 직접 확인 가능) 확인. 둘 다 `mission_status` 프레임이 브로드캐스트되는지 같이 확인

## 알려진 제한사항

- **단일 논리 소비자를 가정한다.** 클라이언트가 1개 이상 붙어있으면 그 순간부터는 버퍼링을 하지 않는다. 관제 화면을 여러 개 동시에 열어두면, 나중에 접속한 두 번째 탭은 첫 번째 탭이 이미 받은 과거 이벤트를 재전송받지 못한다. 지금 단계는 "관제 1곳"을 전제로 한 설계다.
- **버퍼 전달과 clear 사이에 아주 작은 유실 창이 있다.** 클라이언트 접속 직후 버퍼를 비우고 전송을 시작하는데, 전송 도중 그 클라이언트가 바로 끊기면 아직 못 보낸 나머지는 유실된다.
- **인증/암호화가 없다. E-Stop과 미션 launch까지 이 채널로 오간다는 점에서 이전보다 더 중요한 제약이 됐다.** 지금은 현장 로컬 Wi-Fi/AP 안에서만 쓰는 걸 전제로 한다. 같은 네트워크의 누구나 명령 프레임만 만들면 로봇을 정지시키거나 미션을 시작/종료시킬 수 있다. 인터넷 경유로 확장하거나 신뢰할 수 없는 네트워크에 노출하게 되면 이 앞단에 반드시 TLS(WSS)와 토큰 인증을 붙여야 한다.
- **지도 origin 회전을 반영하지 않는다.** `origin.yaw`가 0이 아니면 프론트 마커 위치가 틀어진다.
- **explore 미션 subprocess 추적이 프로세스 핸들 하나뿐이다.** `comm_relay_node`가 재시작되면 이전에 띄운 `explore_lite` subprocess의 핸들을 잃어버린다. `stop_mission`(explore)은 이제 그 경우를 대비해 `pkill -f explore_lite`로 프로세스 이름 기준 fallback을 같이 시도하므로(2026-08-29) 완전히 종료 불가능한 상태는 아니지만, 핸들을 잃은 상태에서는 "정상 종료했다"는 `Terminated explore_lite mission` 로그 없이 이 fallback 경고 로그만 남는다는 차이가 있다. 그래도 노드 재시작 전에는 실행 중인 미션을 먼저 종료하는 걸 권장한다. (`return_home`은 subprocess가 아니라 서비스 호출이라 이 문제가 없다 — `return_home_node`는 `s2m_slam_real.launch.py use_return_home:=true`로 이미 떠 있는 상태를 전제로 트리거/암 서비스만 부른다.)
- **서비스 호출 실패가 클라이언트에 전달되지 않는다.** `/drive/estop` 등이 응답 가능 상태가 아니면 서버 로그에만 경고가 남고, 웹 화면에는 별도 에러가 표시되지 않는다.
