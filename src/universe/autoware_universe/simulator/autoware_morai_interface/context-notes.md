# context-notes — autoware_morai_interface

작업 중 내린 결정과 그 근거. 다음 세션이 같은 조사를 반복하지 않도록 남긴다.
사실 확인은 전부 이 저장소의 파일이나 실행 결과에 기반한다.

## 왜 패키지가 두 개인가

MORAI gRPC proto 트리에 **라이선스 파일이 없다.** upstream 은
`http://172.16.1.20:8081/morai-interface/morai_standard_proto` (SHA `f38b760`) 로
**사설 LAN 주소**라 서브모듈로 쓸 수도 없다 — CI, 동료 노트북, 새 설치 어디서도
`git submodule update` 가 실패한다.

그래서 벤더 종속물을 `morai_grpc_vendor` 한 곳에 격리하고, Autoware 를 향한
`autoware_morai_interface` 는 순수하게 유지했다. 전면 ROS 2 로 옮기게 되면
격리 패키지만 지우면 된다.

격리가 실제로 성립하는지는 세 가지로 확인했다.

- `find install/morai_grpc_vendor -name '*.grpc.pb.h'` → 0개 (스텁을 설치하지 않음)
- `actor_client.hpp` 를 grpcpp 없는 include path 에서 `-Wall -Wextra -Wpedantic -Werror`
  로 단독 컴파일 성공 (PIMPL 경계 유효)
- `autoware_morai_interface/package.xml` 의 모든 키가 rosdep 으로 해결됨

## 왜 gRPC 이고 UDP 는 안 쓰는가

MORAI 는 ROS 2 / UDP / gRPC 세 경로를 제공한다. 구 브리지는 셋을 섞어 썼다
(gRPC 제어·상태, UDP 램프, ROS 2 센서).

`Actor` 서비스에 `SetVehicleTailLight` 가 있어서 **UDP 경로가 통째로 불필요해졌다.**
`lib/network` + `lib/define` 134줄과 하드코딩 IP·포트 3개가 전부 사라졌다.
구 브리지의 UDP 수신기는 애초에 죽은 코드였다 — 포트 9999 를 바인드하고
`multiprocessing.Process` 를 fork 하는데 (DDS 참여자 생성 후 fork 라 위험) 아무도 읽지 않았다.

센서를 gRPC 로 받는 것은 **불가능하다.** `sensor.proto` 의 `GetSensorData` 가 돌려주는
`SensorData` 는 `oneof { GroundTruth ground_truth }` 하나뿐이고, 주석도
"일단은 ground truth만 존재함" 이다. LiDAR·카메라 원시 데이터 정의가 없다.

## 왜 `autoware_package()` 를 벤더 패키지에서 호출하지 않는가

**스타일이 아니라 동작상 필수다.** 이 매크로는 `-Wall -Wextra -Wpedantic -Werror` 를
`add_compile_options()` 로 주입하는데, 이건 **디렉토리 스코프**라 생성된 서드파티
protobuf 코드에까지 걸린다. 호출하지 않는 것 자체가 1차 방어선이다.

방어는 3중이다.

1. 벤더 패키지가 `autoware_package()` 를 안 부름 (주 메커니즘)
2. `set_source_files_properties(... COMPILE_OPTIONS "-w")` — 소스 레벨 옵션이 마지막에 붙음
3. 생성 헤더를 `SYSTEM PUBLIC` 으로 export → 소비자에서 `-isystem` 으로 들어옴

`ament_export_dependencies(Protobuf)` 가 `ament_export_targets` 보다 **먼저** 와야 한다.
`ament_register_extension` 이 단순 `list(APPEND)` 라 호출 순서 = 실행 순서이고,
export 파일이 참조하는 `protobuf::libprotobuf` 가 그 전에 정의돼 있어야 한다.

## 왜 `libgrpc++-dev` 를 package.xml 에 안 넣는가

rosdep 키가 없다 (`rosdep resolve libgrpc++-dev` 실패). 넣으면
`rosdep install --from-paths src --ignore-src -y` 가 **워크스페이스 전체**에 대해
`Cannot locate rosdep definition` 으로 죽는다. 내 패키지 하나가 아니라 전부가 막히고,
에러 메시지는 내 파일을 언급하지도 않는다.

대신 세 곳에 표현했다 — `package.xml` 의 `<description>`, CMake `message(FATAL_ERROR)`
(복붙 가능한 apt 명령 포함), `README.md` 최상단. 추가로 `ansible/roles/grpc/` 를 만들어
`install_dev_env.yaml` 에 넣었다 (`geographiclib` 이 정확히 같은 이유로 존재하는 선례).
`rosdep/grpc.yaml` 은 in-tree 에 두되 연결하지 않았다 — 원하는 사이트만 opt-in.

## 왜 방향을 두 클래스로 나눴고, 왜 벤더 클라이언트는 안 나눴는가

<!-- 26.09.25 _ YSH _ 이 절은 과거 이력이다. StatusReporter/CommandForwarder 는 프레임 고정 Engine 으로 교체돼 삭제됐다
     (아래 "26.09.25 — 프레임 고정 엔진" 절). 방향 분리는 이제 conversion/ 두 파일에만 남는다. -->

`CommandForwarder` / `StatusReporter` 가 각자 자기 pub/sub/timer 를 소유한다.
노드가 전부 소유하면 `node.cpp` 가 `create_publisher` 나열이 되어 분리 의미가 사라진다.
이 방식이면 `git log --follow src/status_reporter.cpp` 가 상태 방향의 전체 역사가 된다.

반면 `morai_grpc_vendor::ActorClient` 는 나누지 않았다. upstream 이 `service Actor`
하나를 선언하므로 채널과 스텁도 하나뿐이고, 나누면 연결 상태가 중복된다.
방향 분리는 브리지 로직의 요구사항이지 전송 계층의 것이 아니다.

<!-- 26.09.25 _ YSH _ 새 링크 계층 MoraiLink 는 채널을 둘로 나눴다 — 위 ActorClient 근거와 어긋나 보여 이유를 남긴다 -->
벤더의 새 링크 계층 `morai_grpc_vendor::MoraiLink`(C++ 엔진용)는 반대로 **상태·명령 두 채널**(서로 다른
TCP 연결)을 연다. 근거가 바뀐 것은 전송 계층 쪽 사정이다: (1) 업데이트마다 받는 큰 `GetAllActorsState`
응답(NPC 수에 비례)과 50 ms deadline `ControlVehicle` 이 한 HTTP/2 연결의 흐름 제어를 나눠 쓰지 않는다.
(2) 한쪽 채널이 끊기거나 막혀도 다른 쪽 상태(`channel_state`)·카운터로 따로 진단된다. 연결 상태가 둘이 되는
비용은 링크 안에서 같은 옵션(주소·백오프·keepalive)으로 만들어 감춘다. AMI 가 MoraiLink 로 옮겨 가면
(Task B) 이 절의 ActorClient 설명은 과거 이력으로만 남는다.

## 조향 부호 비대칭 — 절대 통일하지 말 것

실주행 사고가 났던 지점이다. 컨트롤러가 우측 (−0.415 rad) 을 명령하는 동안 차량이
좌측 풀락 (+0.632 rad) 에 붙어 조향 레이트 제한까지 포화됐다.

- **리포트**: 하드코딩 `-1.0`. MORAI `front_wheel_angle` 은 Unity 좌표계라 ROS 와 반대.
- **명령**: 파라미터 `steer_cmd_sign`, 기본 `+1.0`. `actor_control.proto:29-30` 이
  `steer = 원하는 스티어링 값(radian) / 최대 조향각` 이라고 명시 — ROS 와 같은 규약.
  <!-- 26.09.25 _ YSH _ 부호는 그대로지만 스케일은 proto 주석과 다르다 -->
  단 **스케일은 proto 주석이 틀렸다**: MORAI 는 steer 를 타이어각 라디안으로 적용한다 (main.py eba6b04 —
  0.487 rad 명령이 기계 한계 0.698 rad 로 읽힘, 왕복 게인 1.4331 = 1/0.698). 그래서 명령은 **나누지 않고**
  ±max_steering_angle 로 클램프한다. 옛 C++ 의 `÷ max` 는 1.43 배 과조향이었다.

두 규칙을 `conversion/morai_to_autoware.hpp` 와 `conversion/autoware_to_morai.hpp` 로
**갈라놓았고**, `CommandAndReportSignsAreIntentionallyOpposite` 테스트가 고정한다.
"공유 헬퍼로 통일" 하려는 시도는 즉시 테스트 실패로 드러난다.

`front_wheel_angle` 이 각도가 아니라 정규화값 [-1,1] 이라는 것도 여기 묶여 있다.
형제 필드 `steer` 에 `// -1~1` 주석이 있는 게 근거다.

## 왜 램프 병합이 캐시가 아니라 conversion 에 있는가

`LampCommandCache` 는 두 방향이 공유하는 **유일한 가변 상태**다. 여기에 MORAI 타입이
들어오면 "공유 상태" 헤더가 세 번째 MORAI 누출 지점이 된다. 캐시는 Autoware 상수만
다루고, 병합(`to_turn_signal`)은 방향별 conversion 헤더에 둔다.

`atomic<uint8_t>` 2개가 아니라 mutex + `Snapshot` 반환인 이유. 원자 2개는 찢어진 쌍
(새 turn + 옛 hazard) 을 허용한다. 램프 한 프레임 오류지만 리뷰어에겐 "이게 안전함을
증명하라" 가 된다. 10~50Hz 에 두 바이트 mutex 비용은 측정되지 않는다.

에코 경로에는 대응하는 `morai_to_autoware` 함수를 **일부러 만들지 않았다.**
MORAI 데이터가 개입하지 않으므로 대칭처럼 보이는 함수는 데이터 흐름에 대해 거짓말이다.

## 구 브리지 대비 의도적 동작 변경

| 항목               | 구 브리지            | 현재                          | 이유                                                                 |
| ------------------ | -------------------- | ----------------------------- | -------------------------------------------------------------------- |
| `CRUISE_MODE`      | `5` (DISENGAGED)     | `DISENGAGED` (+WARN)          | 26.09.25 _ YSH _ main.py 로 되돌림 — CRUISE 는 MORAI 가 스스로 모는 제2 제어원이라 AUTONOMOUS 계열로 보고하면 가려진다 |
<!-- 26.09.25 _ YSH _ 원문 행
| `CRUISE_MODE`      | `5` (DISENGAGED)     | `AUTONOMOUS_VELOCITY_ONLY`    | 리터럴 5 를 쓰면서 주석은 AUTONOMOUS_VELOCITY_ONLY 라고 적혀 있었다  |
-->
| `SYNCHRONOUS_MODE` | 미매핑 → `NOT_READY` | `AUTONOMOUS`                  | transition_manager 가 engage 를 취소했다                             |
| `steer` 범위       | ±max rad clamp       | ±max rad clamp (라디안)       | 26.09.25 _ YSH _ main.py eba6b04 와 같게. NaN 은 메시지째 버린다 (main.py 는 +max 가 됐다) |
<!-- 26.09.25 _ YSH _ 원문 행 (정규화 전제 — 틀림)
| `steer` 범위       | 무제한               | `[-1,1]` clamp                | proto 명시 범위. 컨트롤러는 물리 한계를 넘겨 명령할 수 있다          |
-->
| 기어·램프 전송     | 매 명령마다          | 값 변경 시만                  | executor 스레드의 blocking RPC 절약                                  |
| 램프 게이트        | —                    | `forward_control_to_sim` 무관 | 누가 운전하느냐와 무관. 리포트가 사실을 유지                         |
| 리포트 stamp       | 발행 시각            | 샘플 시각                     | 구 브리지는 최대 20ms+RPC 오래된 값에 현재 시각을 찍어 EKF 에 넣었다 |
| 제어 송신 주기     | 100 Hz 재송신 스레드 | MORAI 업데이트당 1 회         | 26.09.25 _ YSH _ 비동기 몰아 보내기에서 MORAI 가속 제어기 진동 실측 (아래 엔진 절) |
| 기어·램프·모드 RPC | executor 콜백        | 전용 스레드                   | 26.09.25 _ YSH _ executor 에서 blocking RPC 0 개 |

## 구 브리지에서 구조적으로 사라진 결함

- **no-op 연결 확인** — `while not is_connected(): connect()` 는 `is_connected()` 가
  `channel is not None` 만 보고 `insecure_channel` 이 lazy 라 항상 1회만 돌았다.
  → `wait_for_connected` + 재시도 타이머
- **cross-thread check-then-use 레이스** — 폴링 스레드가 `ego_state = None` 을 쓰는
  사이에 타이머가 역참조하면 `AttributeError`. → 로컬 복사
- **70초 블로킹 생성자 + throw** — `rclcpp_components` 에서는 컨테이너 전체가 멈춘다.
  → 재시도 타이머
- **deadline 없는 RPC** — executor 스레드에서 호출해 시뮬이 멈추면 퍼블리셔가 전부 정지.
  → 기본 100ms deadline + 콜백 그룹 분리
- **예외를 삼키고 stdout 출력** → `bool` + error 문자열 → ROS 로거

## 검증 방법 — 가짜 gRPC 서버

시뮬레이터 없이 전체 경로를 검증했다. `scripts/` 에 넣지 않고 일회성으로 썼지만,
방법 자체가 재현 가치가 있어 남긴다.

벤더링된 Python 스텁(브리지 repo)과 `grpcio` 로 `Actor` 서비스를 흉내내는 서버를 띄우고
`GetVehicleSpec` / `GetActorState` / `GetVehicleControlMode` 에 알려진 값을 응답시킨 뒤,
`/vehicle/status/*` 로 나오는 값을 확인했다. 명령 방향은 서버가 받은 요청을 찍어 확인했다.

이 방법으로 remap 11개, 발행 주기 6개, 변환 값 전부, 명령 전달, 서비스 응답,
`forward_control_to_sim` 차단까지 시뮬 없이 확인했다. 확인 못 하는 것은 MORAI 자체의
단위·부호 규약뿐이고, 그건 실측이 필요하다.

## 관찰만 하고 손대지 않은 것

`e2e_simulator.launch.xml` 의 `use_ground_truth` 모드(기본 true)는
`/localization/kinematic_state` 와 `/perception/object_recognition/tracking/objects` 를
브리지가 발행한다고 전제하는데, **브리지 repo 어디에도 그 코드가 없다.**
`main.py` 는 `/vehicle/status/*` 6개만 발행한다. 다른 세션의 미완성 작업으로 보이며
이번 범위 밖이라 건드리지 않았다. GT 모드로 주행하려면 그 퍼블리셔가 먼저 필요하다.
<!-- 26.09.25 _ YSH _ 해소: main.py 가 그 뒤 GT 출력을 얻었고(26.08.04), 이 패키지도 같은 식으로 kinematic_state·TF·
     TrackedObjects·/initialpose 텔레포트·/localization/initialize 를 발행한다 (publish_ground_truth, 기본 true). -->

<!-- 26.09.25 _ YSH _ 신규 절 — 프레임 고정 엔진 (Task B) -->

## 26.09.25 — 프레임 고정 엔진 (main.py 대체, GT 모드)

**무엇.** `StatusReporter`(50 Hz 폴링 타이머)·`CommandForwarder`(control_cmd 콜백마다 blocking ControlVehicle)를
지우고 `Engine` 스레드 하나로 바꿨다. MORAI 업데이트(`WaitForTick`)가 트리거하고, 업데이트마다
`GetAllActorsState` 1 회 → 판정 → `StateCache` → **ControlVehicle 정확히 1 회** → 발행. 루프 본체
`Engine::on_update(snapshot)` 는 트리거와 떨어져 있어 lockstep(`Tick`) 드라이버를 나중에 끼울 수 있다.
엔진은 `ILink`(가짜 링크 주입)·`EngineSink`(발행) 에만 의존해 네트워크·executor 없이 gtest 로 돈다.

**왜 이 모양인가 (실측, morai-time-facts).**
- 상태는 targetFPS 와 무관하게 ~50/s 바뀐다 → 더 자주 읽거나 보내도 얻는 것이 없다.
- 비동기 200 Hz(렌더 프레임당 3 개 이상)에서 MORAI 가속 제어기가 진동했고(throttle 0↔1, brake 20~43%),
  갱신당 1 회·100 Hz 동기는 매끄러웠다 → 업데이트가 트리거하는 1 회 송신. 고정 50 Hz 타이머가 아니다.
- 이 빌드는 명령을 업데이트 사이에 유지한다(2 번에 1 번 송신도 98~99% 실현) → 빈 업데이트가 치명적이지 않다.
  그래도 stale(0.2 s) 감속·조향 유지는 main.py 그대로 둔다 (MRM EMERGENCY_STOP 이 no-op 이라 유일한 보호).
- WaitForTick 값은 ActorStates.timestamp 와 다른 시계(일치 0%) → 트리거로만, stamp 는 스냅샷 timestamp.

**main.py 대비 바뀐 것 / 그대로인 것.**
- 그대로: 토픽 이름·QoS(depth 10)·파라미터 이름, 조향 보정(`_steer_comp_gain_now`)·종방향 모드(`_fill_longitudinal`)·
  stale 대체 명령·기어 송신기(`_GearSender`)·조향 리포트 0 유지·map 변환(정·역)·각속도 body 회전·uuid5·
  TrackedObjects 필드·kinematic_state 공분산·측위 DIRECT 1 회. 전부 python3 골든으로 고정 (test/).
- 조향 명령은 **라디안** (옛 C++ 의 ÷max 제거), heading_rate 는 **world rad/s 를 body 로** (옛 C++ 의 deg/s 부호반전 제거).
- stamp = MORAI 샘플 시각 (`time_source`), 새 표본에만 발행 (중복·무효·역행 건너뜀), 나이 > 0.1 s 면 진단 ERROR.
- NaN/inf control_cmd 는 메시지째 버린다 (main.py 는 `max(-m, min(m, nan))` = +m, 좌측 끝).
- 배율(`steer_comp_gain`)은 새 메시지를 처음 보낼 때 그 순간 속도로 한 번 정한다 (main.py 는 콜백 시점 속도).
- 기어 성공 = Result + 되읽기 (main.py 는 예외 없음). control_mode_request·텔레포트도 되읽기로 확인 — MORAI Result 는
  거짓일 수 있다 (SetSynchronousMode 실측).
- 재연결: 스냅샷 UNAVAILABLE 또는 5 연속 실패 → 연결 단계 (deadline 만 이어지면 채널 재생성) → 스펙 재조회 →
  마지막 요청이 AUTO 인데 KEYBOARD 면 AUTO 재지정 (`auto_reassert_auto`). Ego 가 1 초 넘게 없거나 전 필드 0 이어도
  같은 경로 (시나리오 교체로 차가 바뀌면 스펙도 바뀐다). → 리뷰 반영에서 바꿨다 (아래 "엔진 리뷰 반영").
- 중복 writer 가드: `/vehicle/status/velocity_status` 에 다른 노드가 보이면 ControlVehicle 을 막고 ERROR.
  → 리뷰 반영: 자기 제외를 이름 전체에서 endpoint 하나로 (아래).
- 종료 때 최근 송신 중이었으면 ~0.3 s 감속 대체 명령.
- CRUISE → DISENGAGED(5) + WARN (main.py 와 같게 되돌림), SYNCHRONOUS → AUTONOMOUS (main.py 는 NOT_READY).
- 토픽을 코드에서 **절대 이름**으로 둔다 (`~/input` + remap 을 버림) — 단독 실행 파일·GT launch 가 remap 없이 main.py 를
  그대로 대체하게. `vehicle_interface.launch.xml` 의 remap 은 이제 아무 일도 하지 않는다.
- 파라미터 전부 코드 기본값 (yaml 없이 기동), 주소·포트 기본값은 main.py 와 같은 env `MORAI_SIM_ADDRESS/PORT`.
  main.py 가 콜백마다 읽던 것만 런타임 변경 (on-set 콜백), 나머지는 read-only.

**액터 목록 유지(actor_hold_s)는 사실상 발동하지 않는다.** (리뷰 반영: "사실상" 이 아니라 **발동할 수 없다** — 코드에서 빼고
파라미터는 예약으로만 남겼다.) 액터 목록과 Ego 가 같은 GetAllActorsState 에서 오므로
둘의 신선도가 같다 — 스냅샷이 끊기면 Ego 도 끊겨 아무것도 발행하지 않는다. 규칙(낡은 목록은 내지 않고 진단
ERROR)은 코드와 진단에 두었지만, 액터를 따로 조회하게 되기 전까지는 Ego 신선도 게이트와 같이 움직인다.
main.py 는 조회 실패 때 빈 목록을 10 Hz 로 계속 냈다(장애물이 순간 사라짐) — 이 노드는 아예 내지 않는다
(1 s 넘게 끊기면 perception 모니터가 실패해 comfortable stop — 결정 D21 의 "유지 후 중단").

**검증 (26.09.25).** gtest 111 개 green (-Werror). 가짜 MORAI(로컬 고포트, 50/s)로 노드 전체: 업데이트당 송신 분포
{1: 250/250}, 발행 50/25/10 Hz, 150 ms 정체에도 폴백 없음, MORAI 재기동 → ~2 s 에 재연결·스펙·AUTO 재지정,
가드 차단·재개, 텔레포트 오차 0.00 m, SIGINT 감속 15 회, CPU ~1%. 실제 MORAI(7789)에는 붙지 않았다.

<!-- 26.09.25 _ YSH _ 신규 절 — 엔진 리뷰 반영 -->

## 26.09.25 — 엔진 리뷰 반영

리뷰(패리티·런타임) 18 건을 main.py·코드와 대조해 고쳤다. 결정과 이유만 남긴다.

- **노드 이름 = `morai_bridge`** (GT launch 기본). daq_publisher 가 `/morai_bridge` 에서 `map_origin_*` 를 읽고(없으면 0 으로
  조용히 진행 → DAQ 장애물이 틀린 프레임), mission_server 가 `teleport_on_initialpose`·`forward_control_to_sim` 을 쓴다.
  클래스 기본 이름(`morai_interface`)은 `vehicle_interface.launch.xml` 경로 때문에 그대로 두었다 — `ros2 run` 이면 `-r __node:=morai_bridge`.
- **가드 자기 제외는 endpoint 하나.** 이름·namespace 가 같은 endpoint 를 전부 빼면 같은 이름의 main.py(또는 이 노드 두 개)가 안
  보였다. GID 로 빼려 했으나 **rmw_cyclonedds_cpp(Humble) 의 `Publisher::get_gid()` 는 graph 의 endpoint GID 와 다르다**
  (실측: `85d5079df81c88bb00…` 8 바이트 instance handle ↔ `0110…1503` 16 바이트 GUID). 그래서 GID 가 맞으면 그것,
  아니면 같은 이름의 **첫** endpoint 하나만 뺀다. domain 91 에서 같은 이름 두 인스턴스로 확인(혼자면 조용, 둘이면 둘 다 ERROR).
  이 노드끼리 둘 다 막히면 아무도 명령하지 않는다 — 그래서 막히는 순간 송신 중이었으면 ~0.3 s 감속을 보내고 멈춘다(MORAI 는
  마지막 명령을 유지하므로 감속이 남는다). GID 순서 같은 결정론적 양보는 main.py 를 구분할 수 없어(main.py 가 늘 보낸다) 택하지
  않았다. 주기 1 s, 첫 판정 전 송신 보류, `duplicate_writer_guard:=false` 면 진단만(bag 재생·relay).
- **timestamp 를 못 쓰는 빌드.** 판정이 ActorStates.timestamp 에만 기대면 0·멈춘 빌드에서 첫 표본 뒤 영원히 멈춘다(main.py·
  daq_publisher 는 이 필드를 안 읽어 로컬 빌드에서만 검증됨). timestamp 가 0 이면 즉시, 같은 timestamp 로 Ego 내용이 3 번 바뀌면
  **내용 판정**으로 바꾼다: 내용이 바뀌면 새 업데이트, 같아도 트리거가 새 틱을 확인했으면(WaitForTick·lockstep Tick) ≥15 ms
  간격으로 받는다(정지한 차도 명령을 받게), 폴링 폴백에서 같은 내용은 중복(일시정지를 다시 내지 않게). stamp·`/clock` 은 수신 시각.
  timestamp 가 연속 5 번 늘면 되돌아간다. timestamp·내용이 모두 그대로면(일시정지·동기 모드 대기) 아무것도 안 내고 진단 ERROR —
  멈춘 timestamp + 정지한 차 + 폴링이면 구분할 수 없어 멈춘다(알려진 한계, 서브 PC 전 `morai_probe all` 로 확인).
- **역행 새 기준 1 s → 3 표본/0.5 s 점프.** 1 s 동안 Autoware 명령(브레이크 포함)이 MORAI 로 가지 않았다. 역행 중에는 여전히
  송신·발행하지 않는다 (stamp 가 뒤로 가지 않게) — 최대 2 업데이트(~40 ms).
- **막지 않는 WaitForTick.** "성공 + 새 업데이트 없음(같은 값·중복 스냅샷)" 이 10 번 이어지면 폴백, 그 사이 1 ms 쉼
  (WaitForTick 은 렌더 프레임마다, 상태는 ~50/s — targetFPS 200 이면 사이에 중복 ~3 번이 정상이라 여유를 둔다). Ego 없음이면
  5 ms 쉼(중복을 가를 수 없다). 폴백 재확인은 40 ms deadline 으로 3 번 연속 — 값이 매번 다르고 합쳐 ≥5 ms 기다려야 복귀
  (가짜 링크로 1 s 1600 만 번 돌던 경우를 시험으로 고정).
- **Ego 1 s 없음은 재연결이 아니다.** 연결 단계(백오프 0.75~5 s, 스냅샷 안 읽음)로 가던 것을 없애고, 돌아온 첫 표본에서
  스펙만 다시 읽는다(실패 0.5 s 재시도, 그동안 명령 보류, 상태·GT 는 곧바로). 모드 재지정 없음 — 운영자의 시나리오 교체 뒤
  AUTO 를 강제하지 않는다. 램프는 다시 보낸다.
- **ControlVehicle deadline = max(`morai.command_deadline_ms`, 4 × 스냅샷 왕복 p50), 상한 max(1 s, 설정)** — main.py
  `cmd_timeout_for_rtt`(26.09.19). 연결 때 스펙 왕복으로 시작해 50 업데이트마다 갱신. 스냅샷 응답이 명령보다 커서 보수적이다.
  왕복 p50 > 5 ms 면 WARN·`link` 진단 WARN. 업데이트마다 순차라 원격에서는 루프가 ~2.5×왕복보다 빨리 못 돈다(알려진 한계).
- **제어 모드**: 재연결 때 알던 모드를 지우고(`?`) 재지정 판단을 재연결 뒤 **첫 조회 성공**으로 미룬다 — 조회가 실패하면 옛 AUTO
  로 판단해 재지정을 건너뛰던 문제. 조회가 0.5 s 넘게 실패하면 ControlModeReport 생략(main.py 는 실패 즉시 None).
  요청 모드는 RPC **전에** 기록(AUTO 실패면 되돌림) — 되읽기 500 ms 사이 재연결이 disengage 를 AUTO 로 되돌리지 않게.
  모드 요청은 전용 작업 스레드(텔레포트 뒤 ~2 s 줄 서지 않게).
- **Ego 대체 매치**(대소문자 무시)로 고른 항목도 actors 에서 뺀다 — 자차가 base_link 에 장애물로 나갔다 (벤더 `to_snapshot`).
- `/clock` = 메시지 stamp (`receive` 면 수신 시각, 기동 WARN). |MORAI − 수신| > 1 h 면 `clock` 진단 ERROR(epoch 가 아닌 시각).
<!-- 26.09.25 _ YSH _ cmd_resend_hz 는 이제 선언한다 (아래 "명령 트리거 A/B"). 원문:
- main.py 전용 파라미터(`cmd_resend_hz`·`cmd_rpc_timeout_s`·`cmd_send_mode`·`gear_async`·`gear_rpc_timeout_s`·`ego_spec_wait_s`)는
  선언하지 않고 `-p` 로 오면 기동 WARN. 파라미터 처리 차이는 README "main.py 와 다른 점" 표.
-->
- main.py 전용 파라미터(`cmd_rpc_timeout_s`·`cmd_send_mode`·`gear_async`·`gear_rpc_timeout_s`·`ego_spec_wait_s`)는
  선언하지 않고 `-p` 로 오면 기동 WARN. `cmd_resend_hz` 는 `command_trigger=timer` 의 주기로 선언한다(아래 절).
  파라미터 처리 차이는 README "main.py 와 다른 점" 표.
- MultiThreadedExecutor 는 동시성을 주지 않는다(모든 콜백이 기본 MutuallyExclusive 그룹). 명세라 유지하고, executor 전용 멤버가
  이 직렬화에 기댄다는 것을 node.hpp 에 적었다.

**검증.** gtest 136 green (-Werror, 엔진 31→46·표본 판정 4→10·가드 +4, 벤더 대체 매치 시험 갱신). 가짜 MORAI(17891) 스모크: 업데이트당
{1: 250/250}, velocity 50.0 Hz, 가드 1 s 뒤 송신, 모드·기어·stale·SIGINT 감속 15. 실제 MORAI(7789)에는 붙지 않았다.

<!-- 26.09.25 _ YSH _ 신규 절 — 가짜 MORAI 종단 시험 (Task C) -->

## 26.09.25 — 가짜 MORAI 종단 시험

**무엇.** 실제 MORAI 없이 노드 전체(DDS 입출력 + gRPC 두 채널)를 돌리는 스크립트화된 시험. 가짜 서버는 벤더 패키지
(`test/fake/fake_morai_server.cpp`, BUILD_TESTING 에서만 빌드, 설치 안 함 — 서버 스텁이 필요해 벤더에 둔다)에, Autoware 역할을
하는 판정기는 이 패키지 `test/e2e/` 에 둔다 (rclpy, 수동 실행 — colcon test 는 네트워크를 쓰지 않는다는 원칙 유지).
가짜는 실측 사실만 흉내낸다: 50 upd/s, timestamp = epoch µs(가끔 2 프레임), WaitForTick 은 다른 시계, 한 번의
GetAllActorsState 에 Ego+NPC, 명령은 업데이트 사이 유지, AUTO(3)·SYNC(7) 에서만 먹음, SetVehicleControlMode Result 거짓.
업데이트 사이 ControlVehicle 수를 가짜가 직접 센다 — 노드 쪽 통계가 아니라 받는 쪽에서 "업데이트당 1 회" 를 확인한다.

**찾은 결함 두 개 (둘 다 gtest 로 고정).**
- *일시정지 = 트리거 고장으로 오판.* WaitForTick deadline 초과를 무조건 실패로 세어, MORAI 가 1 s 멈추면(일시정지·로딩)
  10 번 → 폴링 폴백 → 10 s 동안 업데이트 트리거를 잃었다. 멈춘 MORAI 에서 WaitForTick 이 안 돌아오는 것은 정상이다.
  → deadline 초과는 **그 뒤 스냅샷이 새 업데이트일 때만**(업데이트가 왔는데 알리지 못함) 센다. deadline 아닌 오류는 늘 센다.
  WaitForTick 이 실시간 모드에서 안 도는 빌드는 여전히 1 s 에 폴백한다 (그때는 스냅샷이 새 업데이트를 보인다).
- *재연결 뒤 명령 채널이 죽은 연결로 READY.* 가짜 서버를 닫았다 다시 열자 상태 채널(호출 중)만 끊김을 알고, 호출이 없던
  명령 채널은 `wait_connected` 에 READY 로 보였다. 재연결 직후 첫 명령 RPC(AUTO 재지정)가 `UNAVAILABLE: Socket closed` 로
  실패했고, 재지정은 한 번뿐이라 MORAI 가 KEYBOARD 에 남았다 (ControlVehicle 은 다음 업데이트부터 됐다).
  → UNAVAILABLE 로 끊긴 재연결도 두 채널을 새로 만든다 (deadline 연속과 같은 경로). 그리고 되읽기 전에 실패한(=MORAI 에
  닿지 못한) 재지정은 다음 모드 조회마다 다시, 합쳐 3 번. 되읽기 불일치(MORAI 가 거부)는 되풀이하지 않는다 — 되읽기
  500 ms 가 엔진 루프를 막기 때문이다.

**결과 (2026-09-25, 20/20).** 수치는 README "가짜 MORAI 종단 시험". 채널 재생성 뒤 재기동→재개가 0.3 s → 0.8 s 로 늘었다
(새 채널이 끊긴 동안 백오프를 1 s 상한까지 키운다) — MORAI 재기동 자체가 수십 초라 문제로 보지 않았다.

**확인 못 한 것.** 실제 MORAI 의 일시정지 때 WaitForTick 동작(가짜는 안 돌아오게 만들었다), 실제 MORAI 재기동 때 명령 채널
상태(가짜의 Shutdown 은 GOAWAY 뒤 소켓을 닫는다), 원격 PC 왕복, Autoware 폐루프(실제 컨트롤러가 내는 control_cmd 주기·값).

<!-- 26.09.25 _ YSH _ 신규 절 — 명령 트리거 A/B (feat/cmd-timer-mode) -->

## 26.09.25 — 명령 트리거 A/B (`command_trigger`)

**왜.** 알려진 정상 조건은 MORAI 기본(~50 Hz) + main.py `_CmdResender`(최신 명령을 고정 100 Hz 로 동기 재송신)였다. C++ 노드
(업데이트당 1 회)로 Autoware 폐루프를 돌리자 MORAI 가 실제로 적용한 throttle 이 가속 중 0 ↔ on 을 초당 ~4.4 번 오가고 제동
페달이 4~8 % 로 약했다. 송신 방식이 원인인지 가르려면 **같은 노드 안에서** 두 방식을 바꿔 볼 수 있어야 한다 — 노드·빌드·파라미터·
변환이 다르면 A/B 가 아니다.

**무엇.** `command_trigger` = `update`(기본, 엔진 루프가 받아들인 업데이트마다 1 회) | `timer`(전용 명령 스레드가 최신 명령을
`cmd_resend_hz` 로 동기 송신). 규칙은 main.py `_run`/`_tick` 그대로: steady 절대 deadline, `delay > 0` 이면 기다리고 다시 본다,
보낸 뒤 `다음 += 주기`, 지금보다 뒤면 `다음 = 지금` + late (몰아 보내지 않음 — gtest `NoBurstAfterAStall`), 호출은 동기 1 개.
명령을 만드는 곳은 `send_command(speed, params, now)` 하나다 — update 는 그 업데이트 표본의 속도, timer 는 StateCache 최신 표본의
속도를 준다. 그래서 조향 라디안·결손 보정(새 메시지를 처음 보낼 때 한 번)·stale·NaN(슬롯에서 거름)·forward·가드 감속·종료
감속이 둘 다 같다. timer 에서 엔진 루프는 보내지 않는다(`on_update` 가 트리거를 보고 건너뛴다).

**결정.**
- *런타임 전환 가능.* A/B 를 한 주행 안에서(같은 Autoware·같은 MORAI 상태) 할 수 있게 `RuntimeParams` 에 넣었다. 타이머 스레드는
  늘 떠 있고 update 면 20 ms 마다 설정만 본다 (idle CPU 무시할 만함). 전환 순간 두 스레드가 겹칠 수 있어 송신 상태를
  `cmd_mutex_` 로 묶었다 — 평소에는 한 스레드만 잡아 다툼이 없다. 발행(build_outputs)이 읽는 조향 배율·마지막 조향은 원자,
  10 초 로그 창 최댓값은 `window_mutex_` (RPC 를 잡은 채 엔진 스레드가 기다리지 않게 따로).
- *연결 단계에서는 보내지 않는다.* main.py 는 벤더 래퍼가 조용히 재연결하는 동안에도 두드렸지만, 여기서는 엔진이 연결 상태
  기계·백오프·채널 재생성을 가지므로 타이머는 `phase != running` 이면 쉰다 (100 Hz 로 죽은 채널을 두드리지 않는다). 끊김을
  엔진이 알아채기 전까지의 몇 호출은 deadline 으로 묶이고 late 로 잡힌다. 첫 유효 표본·스펙 전에도 보내지 않는다.
- *MORAI 가 멈춰도(링크 살아 있음) 계속 보낸다.* main.py 와 같다 — update 와의 차이 자체가 비교 대상이다. Ego 무효가 1 s 넘으면
  스펙 재조회까지 보류하는 규칙은 공통이라 timer 도 그때 멈춘다 (가짜 e2e 6).
- *종료 감속은 timer 면 같은 주기*(100 Hz × 0.3 s = 30 회), update 면 전처럼 업데이트마다.
- `cmd_resend_hz` 는 정수도 받는다(`dynamic_typing`) — `cmd_resend_hz:=100`·`ros2 param set … 100` 이 정수로 와서 double 로만
  선언하면 기동 때 타입 예외로 죽는다. 10~400 밖이면 기동 때 WARN 후 100, 런타임은 거부.
- *기본은 update 로 둔다.* 설계 근거 실측(비동기 몰아 보내기에서 가속 제어기 진동)과 종단 시험을 거친 경로다. timer 가 폐루프에서
  낫다고 실측되면 그때 바꾼다.

**검증 (2026-09-25).** gtest 140 → 156 (timer 16 개, 가짜 시계 한 스레드로 업데이트 20 ms·명령 30 ms·타이머를 번갈아 돌린다 +
실제 스레드 1 개). 변이 확인: late 재설정을 빼거나 timer 에서도 엔진 루프가 보내게 하면 8 개가 깨진다. 가짜 MORAI e2e 두 모드
21/21: update 49.45 upd/s 에 ControlVehicle 49.45/s(업데이트당 1.000), timer 100.00/s(업데이트당 2.026, 간격 p50 10.00 ·
max 10.18 ms, 지연재설정 0), CPU 2.1 % → 2.8 %. GT launch 로 `command_trigger:=timer cmd_resend_hz:=100`(정수) 기동, 런타임
50 허용·5·bogus 거부 확인. 실제 MORAI(7789)에는 붙지 않았다.

**확인 못 한 것.** 실제 MORAI 가속 제어기가 업데이트당 2 개(같은 값)를 받을 때와 1 개를 받을 때 적용 throttle 이 어떻게 다른지 —
그것이 이 A/B 의 목적이다(사용자 폐루프). MORAI 가 한 업데이트 안에서 여러 명령을 어떻게 소비하는지(마지막 것만? 평균?)는
`morai_actuate` 로 따로 잴 수 있다.
