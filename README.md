# S2M-CommRelay

Scout2Map의 통신 중계 레이어다. `S2M-Event-Engine`이 발행하는 `/events`를 Web-Monitoring 쪽으로 넘기되, **네트워크가 끊겼다가 다시 붙어도 그 사이에 놓친 이벤트가 유실되지 않도록** 버퍼링하는 게 이 레포의 핵심 역할이다.

rosbridge_suite를 쓰지 않는다. rosbridge는 pub/sub을 WebSocket으로 중계만 할 뿐 끊긴 동안의 메시지를 들고 있다가 재전송하는 기능이 없어서, 이벤트처럼 "놓치면 안 되는" 데이터에는 맞지 않다. 대신 `comm_relay_node`가 자체 WebSocket 프로토콜로 버퍼링·재전송·연결 상태 보고까지 직접 처리한다.

SLAM 지도나 raw 센서 스트림처럼 "최신 값만 중요하고 끊겨도 다시 받으면 그만"인 데이터는 이 레포의 스코프가 아니다. 그런 최선형(best-effort) 스트림은 나중에 rosbridge_websocket 기반으로 별도 launch profile을 추가해서 처리할 계획이며, 지금은 포함되어 있지 않다.

## 아키텍처

```
S2M-Event-Engine                 S2M-CommRelay                  Web-Monitoring
  /events (String, JSON)  ──►  comm_relay_node
                                  ├─ 클라이언트 연결됨 → 즉시 전달
                                  ├─ 클라이언트 없음   → 버퍼에 적재
                                  └─ 재연결 시         → 버퍼 순서대로 재전송
                                       │
                                       ▼
                                ws://<sbc-ip>:9091  ──────────►  new WebSocket(...)
                                /relay/link_status (String, JSON, 로컬 ROS 헤르트비트)
```

## 왜 커스텀 프로토콜인가

roslibjs/rosbridge 방식이 아니라 브라우저 기본 `WebSocket` API만으로 붙을 수 있는 얕은 JSON 프로토콜을 직접 정의했다. 이유:

- 재연결 시 "얼마나 끊겨 있었는지", "몇 개가 밀렸는지", "몇 개가 버려졌는지"를 클라이언트 접속 시점에 알려줘야 하는데, 이건 rosbridge 프로토콜에 없는 개념이라 어차피 얹으려면 커스텀 레이어가 필요하다.
- `/events` 하나만 다루는 지금 단계에서는 rosbridge의 범용성(임의 토픽/서비스 노출)이 필요 없다. 오히려 화이트리스트 설정, 인증 부재 같은 rosbridge의 일반적인 노출면을 이 시점에 신경 쓸 필요가 없어진다.

## 메시지 프로토콜

서버 → 클라이언트, 한쪽 방향으로만 흐른다 (클라이언트가 보내는 프레임은 무시함).

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
`replay: true`면 끊긴 동안 쌓였다가 재전송된 것, `false`면 실시간으로 온 것 — 프론트에서 "밀린 알림"과 "지금 막 온 알림"을 구분해서 표시하고 싶을 때 이 필드를 쓰면 된다.

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
```

## 파라미터 (`config/comm_relay.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `events_topic` | `/events` | 구독할 이벤트 토픽 |
| `ws_host` / `ws_port` | `0.0.0.0` / `9091` | WebSocket 서버 바인드 주소 |
| `buffer_max_len` | `500` | 클라이언트 없을 때 버퍼에 쌓을 최대 개수 |
| `buffer_max_age_s` | `1800.0` | 이보다 오래된 버퍼 항목은 버림 |
| `link_status_topic` | `/relay/link_status` | 로컬 ROS 연결 상태 헤르트비트 |
| `link_status_period_s` | `2.0` | 헤르트비트 발행 주기 |
| `ws_ping_interval_s` / `ws_ping_timeout_s` | `10.0` / `10.0` | WebSocket keepalive (반쯤 끊긴 연결 빨리 감지) |

## 빌드 & 실행

```bash
# websockets가 rosdep으로 안 풀리면 pip로 직접
pip install websockets --break-system-packages

cd ~/ros2_ws
colcon build --packages-select scout2map_comm
source install/setup.bash
ros2 launch scout2map_comm comm_relay.launch.py
```

## 테스트 방법

- `test/test_comm_relay_buffer.py`: 버퍼 적재/prune 로직 단위 테스트 (`colcon test --packages-select scout2map_comm`)
- 수동 확인: `event_engine`을 같이 띄운 상태에서 브라우저 콘솔이나 `wscat -c ws://<sbc-ip>:9091`로 접속해서 `status` 프레임이 오는지, 온도 임계값을 넘겨서 실제 `event` 프레임이 오는지 확인
- 버퍼링 확인: 클라이언트를 끊고(브라우저 탭 닫기 등) 그 사이 이벤트를 몇 개 발생시킨 뒤 재접속 → `status`의 `buffered_count`와 실제로 온 `event` 프레임 개수가 맞는지, `replay: true`로 표시되는지 확인
- `ros2 topic echo /relay/link_status`로 로컬에서도 연결 상태를 볼 수 있다

## 알려진 제한사항

- **단일 논리 소비자를 가정한다.** 클라이언트가 1개 이상 붙어있으면 그 순간부터는 버퍼링을 하지 않는다. 관제 화면을 여러 개 동시에 열어두면, 나중에 접속한 두 번째 탭은 첫 번째 탭이 이미 받은 과거 이벤트를 재전송받지 못한다. 지금 단계는 "관제 1곳"을 전제로 한 설계다.
- **버퍼 전달과 clear 사이에 아주 작은 유실 창이 있다.** 클라이언트 접속 직후 버퍼를 비우고 전송을 시작하는데, 전송 도중 그 클라이언트가 바로 끊기면 아직 못 보낸 나머지는 유실된다(다음 재연결에 다시 안 옴). 단일 운영자 테스트 도구 수준에서는 감수 가능한 트레이드오프로 판단했다.
- **인증/암호화가 없다.** 지금은 현장 로컬 Wi-Fi/AP 안에서만 쓰는 걸 전제로 한다. 인터넷 경유로 확장하게 되면 이 앞단에 반드시 TLS(WSS)와 토큰 인증을 붙여야 한다.

## 로드맵 (스코프 밖, 참고용)

- `/map` (nav_msgs/OccupancyGrid), raw 센서 스트림 — best-effort라 버퍼링 불필요, rosbridge_websocket 기반 별도 launch profile로 추가 예정
- 멀티 클라이언트 지원이 필요해지면 클라이언트별 커서(마지막으로 받은 `seq`) 추적으로 확장
