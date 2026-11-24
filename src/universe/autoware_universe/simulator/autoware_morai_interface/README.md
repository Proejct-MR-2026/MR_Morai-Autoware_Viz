# autoware_morai_interface

<!-- 26.09.25 _ YSH _ 엔진 교체: 차량 인터페이스 + GT(측위·장애물) 브릿지. main.py 를 대체한다 -->

MORAI SIM 과 Autoware 사이의 **차량 인터페이스 + GT 브릿지**. `Morai_Autoware_Bridge/scripts/main.py`
를 대체하는 C++ 노드다. 토픽 이름·QoS·변환식·파라미터 이름은 main.py 와 같다.

- 차량 I/O: `/vehicle/status/*` 발행, `/control/command/*` 를 MORAI 로 전달
- GT 모드(센서 없이 MORAI 정답값 사용): `/localization/kinematic_state` + `map→base_link` TF,
  `/perception/object_recognition/tracking/objects`, `/initialpose` 텔레포트, `/localization/initialize`

<!-- 26.09.25 _ YSH _ 원문
MORAI SIM 과 Autoware 사이의 **차량 인터페이스**. `autoware_carla_interface` 와 같은 자리를
차지하지만, 담당 범위는 차량 I/O 뿐이다 — 센서는 MORAI 가 ROS 2 로 직접 발행하고
`morai_sensor_kit_launch` 가 변환한다.
-->

MORAI 와의 통신은 전부 gRPC 로, [morai_grpc_vendor](../morai_grpc_vendor/README.md) 의
`MoraiLink`(상태·명령 두 채널)를 통한다. 이 패키지에는 MORAI 종속물이 없다 — `package.xml` 의 모든 키가
rosdep 으로 해결된다.
<!-- 26.09.25 _ YSH _ 원문: `ActorClient` 를 통한다. (ActorClient 는 이제 센서 장착에만 쓴다) -->

## Prerequisites

의존 패키지 `morai_grpc_vendor` 는 **rosdep 이 설치해주지 않는** gRPC 개발 패키지를 요구한다.
한 번만 설치하면 된다.

```bash
sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc
```

이유는 [morai_grpc_vendor/README.md](../morai_grpc_vendor/README.md) 참조.

<!-- 26.09.25 _ YSH _ 신규 절 — GT 모드로 Autoware 와 같이 돌리는 법 -->

## 실행 — GT 모드 (main.py 대신)

> **main.py 와 절대 같이 띄우지 말 것.** 둘 다 `/vehicle/status/*`·`/localization/kinematic_state`·
> TF 를 발행하고 ControlVehicle 을 보낸다. 노드에 **중복 writer 가드**가 있어 다른 publisher 가
> `/vehicle/status/velocity_status` 를 발행하면(1 초마다 확인, 첫 확인 전에는 송신하지 않는다) ERROR 를 내고
> ControlVehicle 송신을 막지만, 상태·GT 발행은 막지 못한다. 송신 중에 막히면 ~0.3 s 감속 대체 명령을 먼저 보낸다
> (MORAI 는 마지막 명령을 유지한다 — 가속이 남지 않게). 자기 publisher 는 **하나만** 빼므로 같은 이름(main.py 의
> `/morai_bridge`)도, `launch_vehicle_interface:=true` 로 이 노드를 한 번 더 띄운 것도 잡힌다 — 이 노드끼리면
> 둘 다 막히고 MORAI 에는 감속이 남는다(domain 91 에서 같은 이름 두 개로 확인). bag 재생·relay 가
> velocity_status 를 내야 하면 `duplicate_writer_guard:=false`(진단만).
<!-- 26.09.25 _ YSH _ 리뷰 반영: 가드 자기 제외를 이름 전체 → 한 endpoint 로, 주기 2 s → 1 s, 차단 순간 감속, 끌 수 있게 -->

1. 빌드 (워크스페이스 루트, 깨끗한 셸):

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
     --packages-up-to autoware_morai_interface
   ```

2. MORAI 에서 시나리오를 띄우고 Ego 를 둔다 (targetFPS 는 60 으로 충분하다 — 상태는 FPS 와 무관하게
   초당 ~50 번 바뀐다). 제어 모드는 KEYBOARD 여도 된다 — Autoware engage 가 `/control/control_mode_request`
   로 AUTO 로 바꾼다.

3. Autoware (GT 모드, 차량 인터페이스는 끈다):

   ```bash
   ros2 launch autoware_launch e2e_simulator.launch.xml simulator_type:=morai \
     use_ground_truth:=true launch_vehicle_interface:=false \
     map_path:=<MORAI 에 로드된 맵과 같은 lanelet2 맵> vehicle_model:=morai_vehicle sensor_model:=morai_sensor_kit
   ```

4. 이 노드 (다른 터미널, 같은 `ROS_DOMAIN_ID`·RMW):

   ```bash
   # 같은 PC 의 MORAI
   ros2 launch autoware_morai_interface morai_interface_gt.launch.xml
   # 다른 PC 의 MORAI — 인자 또는 main.py 와 같은 환경변수
   ros2 launch autoware_morai_interface morai_interface_gt.launch.xml morai_address:=10.10.10.1
   MORAI_SIM_ADDRESS=10.10.10.1 ros2 launch autoware_morai_interface morai_interface_gt.launch.xml
   ```

   **노드 이름은 main.py 와 같은 `/morai_bridge`** 다 (launch 인자 `node_name`). daq_publisher 가 `map_origin_*` 를,
   mission_server 가 `teleport_on_initialpose`·`forward_control_to_sim` 을 `/morai_bridge` 파라미터 서비스로 읽고 쓴다 —
   다른 이름이면 DAQ 장애물이 원점 0 으로 틀린 프레임에 찍히고 mission_server 는 "브릿지 접근 실패" 가 난다.
   launch 없이 띄울 때도 이름을 맞춘다:
   `ros2 run autoware_morai_interface autoware_morai_interface_node --ros-args -r __node:=morai_bridge`.
   모든 파라미터에 코드 기본값이 있고, 토픽은 절대 이름이라 remap 이 필요 없다 (단독 실행 파일은 MultiThreadedExecutor).
   맵 원점이 MORAI 원점과 다르면 `map_origin_offset_*` 를 파라미터 파일에 넣는다 (main.py 에서 쓰던 값 그대로).
   <!-- 26.09.25 _ YSH _ cmd_resend_hz 는 이제 이 노드 파라미터다 (command_trigger=timer 주기). 원문:
   main.py 에만 있던 파라미터(`cmd_resend_hz` 등)를 넘기면 효과가 없다는 WARN 이 뜬다 (아래 "main.py 와 다른 점").
   -->
   main.py 에만 있던 파라미터(`cmd_rpc_timeout_s` 등)를 넘기면 효과가 없다는 WARN 이 뜬다 (아래 "main.py 와 다른 점").
   <!-- 26.09.25 _ YSH _ launch 인자 대신 yaml·ros2 param set (GT 기본 timer). 원문:
   `cmd_resend_hz` 는 이 노드에서도 쓴다 — `command_trigger:=timer` 일 때의 주기다 (아래 "명령 트리거 A/B").
   -->
   `cmd_resend_hz` 는 이 노드에서도 쓴다 — `command_trigger` 가 `timer` 일 때(GT yaml 기본)의 주기다 (아래 "명령 트리거 A/B").

5. RViz 에서 경로를 찍고 engage. 측위는 첫 유효 표본 뒤 `/localization/initialize`(DIRECT)로 자동 초기화된다.
   "2D Pose Estimate" 를 찍으면 MORAI Ego 가 그 자리로 순간이동한다(`teleport_on_initialpose`).

### 다른 PC 의 MORAI — 시계

`time_source: morai`(기본)면 모든 stamp 가 **MORAI 호스트 시계**(ActorStates.timestamp, Unix epoch µs)다.
같은 PC 면 이 PC 시계와 같다. **두 PC 면 chrony 로 시계를 맞춰야 한다** — 안 맞으면 Autoware 의 TF 조회·
타임아웃 판정이 오프셋만큼 틀린다. 노드는 업데이트마다 `MORAI timestamp − 수신 시각` 을 재서
`/diagnostics` 의 `morai_bridge: clock` 에 중앙값·p01·p99 를 내고, `|중앙값| > clock_offset_warn_ms`(20 ms)
면 WARN 한다. 같은 PC 에서 이 값은 −(프레임 나이) ≈ −1 ms 다. 두 PC 에서 이 값에서 같은 PC 기준선을 뺀 것이
대략의 시계 오프셋이다 (`chronyc tracking` 으로 교차 확인). 시계를 맞출 수 없으면 `time_source: receive`
(이 PC 수신 시각)로 돌린다.

<!-- 26.09.27 _ YSH _ GT 기본값 변경 -->
**GT 파라미터 파일(`morai_interface.gt.param.yaml`)은 26.09.27 부터 `time_source: receive` 다.** 원래 컴퓨터(ADS PC 10.10.10.2 ↔ MORAI PC 10.10.10.1)는
시계 동기화가 확인되지 않았기 때문이다(옛 main.py 도 수신 시각을 썼다). 이때 `/daq/*` 의 stamp 도 수신 시각이고, MORAI 원래 시각은
`/daq/raw/actor_states` 의 `morai_timestamp_us` 에 그대로 남는다. 두 PC 시계를 chrony 로 맞추면 `time_source: morai` 로 돌려도 된다.

측정은 노드와 같이 `morai_probe monitor` 를 **`--trigger poll`(기본)** 로 띄워 볼 수 있다 — 읽기 전용이다
([morai_grpc_vendor README](../morai_grpc_vendor/README.md) "monitor").

<!-- 26.09.25 _ YSH _ 리뷰 반영: 서브 PC 전 확인 절차, timestamp 를 못 쓰는 빌드, 원격 왕복 -->
<!-- 26.09.25 _ YSH _ 최종 점검: morai_probe 에는 --address 가 없다 (unknown option, 종료 코드 1) → --host. 원문:
**서브 PC(다른 빌드일 수 있다)로 가기 전에** 그 MORAI 에 `ros2 run morai_grpc_vendor morai_probe all --address <IP>`
-->
**서브 PC(다른 빌드일 수 있다)로 가기 전에** 그 MORAI 에 `ros2 run morai_grpc_vendor morai_probe --host <IP> all`
(읽기 전용)를 먼저 돌려 `ActorStates.timestamp` 가 epoch µs 로 초당 ~50 번 느는지, WaitForTick 이 업데이트마다
돌아오는지 본다. 이 노드는 timestamp 로 새 업데이트를 가른다. timestamp 가 0 이거나 그대로인데 Ego 가 움직이면
**내용 판정**으로 바꾸고(ERROR 로그, `morai_bridge: update` WARN) stamp 는 수신 시각을 쓴다 — 첫 표본 뒤 송신·발행이
멈추지 않게. timestamp 도 내용도 그대로면(일시정지·동기 모드 대기) 아무것도 보내거나 내지 않고 `update` 가 ERROR 다.
왕복이 길면(p50 > 5 ms) ControlVehicle deadline 을 max(`morai.command_deadline_ms`, 4 × 왕복 p50)(상한 1 s)로 늘리고
`link` 진단이 WARN 이다 (main.py `cmd_timeout_for_rtt`). 업데이트마다 WaitForTick → 스냅샷 → 송신이 순차라 루프는
~2.5 × 왕복보다 빨리 돌 수 없다 — 직결 LAN(왕복 < 1 ms)을 쓴다.

<!-- 26.09.25 _ YSH _ 원문 "실행" 절 — vehicle_interface.launch.xml 경로는 그대로 쓸 수 있지만(launch_vehicle_interface:=true) GT 모드 기본 경로는 위 launch 다
## 실행

```bash
ros2 launch morai_vehicle_launch vehicle_interface.launch.xml
```

보통은 직접 띄우지 않고 `e2e_simulator.launch.xml` 이
`launch_vehicle_interface:=true` (기본값) 로 자동 기동한다.

> **구 브리지와 동시에 실행하지 말 것.**
> `Morai_Autoware_Bridge/scripts/main.py` 도 `/vehicle/status/*` 를 발행하므로
> 둘을 같이 띄우면 리포트가 뒤섞인다. 이 패키지가 그 스크립트를 대체한다.

> **`component_container_mt` 가 필수다.**
> 노드는 명령 방향과 상태 방향을 서로 다른 `MutuallyExclusive` 콜백 그룹에 둔다.
> 단일스레드 executor 에서는 그 분리가 무의미해져, blocking 상태 RPC 가 제어 명령
> 전달을 최대 `rpc_deadline_ms` 만큼 막는다. 동봉 launch 는 이미 `_mt` 를 쓴다.
-->

`launch_vehicle_interface:=true` 로 `morai_vehicle_launch/vehicle_interface.launch.xml` 이 이 노드를 컴포넌트로
띄우는 경로도 남아 있다 (`config/morai_interface.param.yaml`). 그 launch 의 `~/input`·`~/output` remap 은 이제
아무 일도 하지 않는다 — 토픽이 코드에서 절대 이름이다.

<!-- 26.09.25 _ YSH _ 신규 절 — 엔진 구조 -->

## 엔진 — MORAI 업데이트당 한 번

실측(로컬 MORAI): 상태 스냅샷은 targetFPS 와 무관하게 초당 ~50 번 바뀌고, `WaitForTick` 은 실시간 모드에서도
업데이트마다 한 번 돌아온다. 명령을 비동기로 몰아 보내면(렌더 프레임당 3 개 이상) MORAI 가속 제어기가 진동했다
(throttle 0↔1, 브레이크 개입) — 현장 증상 "0 ↔ 명령" 의 유력 원인이다. 그래서 엔진은 **업데이트가 트리거하고,
업데이트마다 ControlVehicle 을 정확히 한 번** 보낸다. 고정 50 Hz 타이머가 아니다 — MORAI 가 멈추면 같이 멈추고,
MORAI 프레임에 맞춰 규칙적으로 들어간다.

엔진 스레드 한 바퀴 (`Engine::step`):

1. **트리거**: `WaitForTick` (deadline `morai.wait_for_tick_deadline_ms`, 100 ms). 값은 다른 시계라 트리거로만 쓴다.
2. **스냅샷**: `GetAllActorsState` 한 번 = Ego + NPC + timestamp.
3. **판정** (`SampleGate`): Ego 없음·전 필드 0(daq_publisher `ego_is_valid`) → 건너뜀.
   timestamp 가 직전과 같음 → 건너뜀(보내지도 발행하지도 않는다). 역행 → 건너뜀(1 s 넘게 이어지면 새 기준).
4. **StateCache** 에 저장 (텔레포트 확인·측위 초기화·기어 재동기가 읽는다).
5. **ControlVehicle 1 회**: 최신 `control_cmd` 로 만든 명령. 0.2 s 넘게 새 명령이 없으면 감속 대체 명령
   (−2.4 m/s², 조향은 마지막 값). `forward_control_to_sim=false`·중복 writer 가드면 보내지 않는다.
<!-- 26.09.26 _ YSH _ CARLA·simple_planning_simulator 방식으로 통일, 모드 조회는 전용 스레드. 원문:
6. **발행**: velocity·steering·kinematic_state·TF 매 업데이트, gear·control_mode 2 번에 1 번,
   TrackedObjects 5 번에 1 번. 제어 모드 조회는 5 번에 1 번.
-->
6. **발행**: 차량 리포트 전부(velocity·steering·gear·control_mode·turn·hazard)와 kinematic_state·TF 를 매 업데이트,
   **같은 stamp** 로 낸다 (CARLA·Autoware simple_planning_simulator 와 같은 "한 스텝 = 한 묶음"). TrackedObjects 는
   업데이트 개수가 아니라 MORAI 시각 100 ms 스케줄(10 Hz)로 낸다 — MORAI 업데이트가 1 ms 쌍·40 ms 구멍으로 고르지 않거나
   업데이트 수가 바뀌어도(targetFPS) 평균 ~10 Hz 다. 간격은 80 ms 이상(몰아 내지 않음), 길어야 100 ms + MORAI 업데이트
   간격이다. 시각 재기준·판정 방식 전환 때는 스케줄을 새로 잡는다. 제어 모드 조회는 이 루프에서 하지 않는다 (아래 모드 조회 스레드).

루프 본체(`Engine::on_update(snapshot)`)는 트리거와 떨어져 있다 — 이후 lockstep(`Tick`) 드라이버는 트리거만 바꾼다.

그 밖의 스레드와 콜백:

| 누가                 | 무엇                                                                                                  |
| -------------------- | ----------------------------------------------------------------------------------------------------- |
| 구독 콜백 (executor) | 슬롯·캐시에 넣기만 한다 (**RPC 없음**). NaN/inf `control_cmd` 는 버리고 마지막 유효 명령 유지         |
| `GearSender` 스레드  | 기어 변경 시 송신, 실패 0.2 s 뒤 재시도, 1 s 마다 MORAI 실제 기어와 대조해 재송신 (main.py `_GearSender`), Result + 되읽기 확인 |
| 작업 스레드          | `control_mode_request`(지연 응답, 되읽기 확인), `/initialpose` 텔레포트, 램프, 센서 장착               |
<!-- 26.09.25 _ YSH _ 가드 주기는 리뷰 반영 때 1 s 로 바뀌었는데 표가 남아 있었다. 원문 행:
| executor 타이머      | 램프 리포트 10 Hz, 측위 초기화 1 Hz, 중복 writer 가드 2 s, `/diagnostics` 1 Hz                         |
-->
<!-- 26.09.26 _ YSH _ 램프 리포트는 업데이트마다 발행으로 옮기고 모드 조회 스레드를 더했다. 원문 행:
| executor 타이머      | 램프 리포트 10 Hz, 측위 초기화 1 Hz, 중복 writer 가드 1 s, `/diagnostics` 1 Hz                         |
-->
| 모드 조회 스레드     | `GetVehicleControlMode` 10 Hz (MORAI 는 다음 프레임에야 답한다 — 엔진 루프 안에서 부르던 때는 5 업데이트마다 1 개를 놓쳐 41.5 Hz 였다). 조회 도중 재연결·AUTO 설정이 끼면 그 결과는 버린다. AUTO 재지정 판단은 엔진 스레드가 새 조회 결과마다 한다 |
| executor 타이머      | forward 갱신·램프 재동기 10 Hz, 측위 초기화 1 Hz, 중복 writer 가드 1 s, `/diagnostics` 1 Hz              |

<!-- 26.09.25 _ YSH _ 리뷰 반영: 폴백·역행·Ego 없음·모드 재지정 규칙 갱신 -->
<!-- 26.09.25 _ YSH _ 가짜 MORAI 종단 시험 반영: deadline 초과는 그사이 업데이트가 왔을 때만 실패로 센다. 원문 첫 줄:
- **폴백**: `WaitForTick` 이 UNIMPLEMENTED 이거나 스냅샷은 되는데 10 번 연달아 실패하면(동기 모드 등), 또는
-->
- **폴백**: `WaitForTick` 이 UNIMPLEMENTED 이거나, 스냅샷은 **새 업데이트를 보이는데** 10 번 연달아 실패하면(deadline
  아닌 오류는 늘 센다. deadline 초과 + 스냅샷도 그대로면 MORAI 가 멈춘 것이라 — 일시정지·정체·아무도 Tick 하지 않는
  동기 모드 — WaitForTick 이 맞게 기다린 것으로 보고 세지 않는다. 1 s 넘는 일시정지 한 번에 10 s 폴백으로 넘어가던 것을
  가짜 MORAI 1 s 멈춤으로 확인하고 고쳤다), 또는
  성공은 하는데 새 업데이트가 없는 일(같은 값·중복 스냅샷)이 10 번 이어지면(막지 않고 바로 돌아오는 빌드 — 헛돌기 방지.
  WaitForTick 은 렌더 프레임마다 돌아오고 상태는 ~50/s 라 targetFPS 200 이면 사이에 중복 ~3 번은 정상)
  WARN 과 진단을 내고 `fallback_hz`(100 Hz) 폴링으로 바꾼다. 송신은 여전히 **새 업데이트마다 한 번**. 10 초마다
  짧은 deadline(40 ms)으로 3 번 연달아 확인해(값이 매번 다르고 실제로 기다려야) 되면 돌아간다.
- **역행**: MORAI 호스트 시계가 뒤로 가면(chrony makestep, Windows 시간 서비스) 서로 다른 역행 표본 3 개째(~60 ms),
  또는 0.5 s 넘게 한 번에 뒤로 가면 바로 새 기준으로 받아들인다 — 그동안만 송신·발행을 건너뛴다.
<!-- 26.09.25 _ YSH _ 가짜 MORAI 종단 시험 반영: UNAVAILABLE 끊김도 채널 재생성, AUTO 재지정 전송 실패는 재시도. 원문:
- **재연결**: 스냅샷이 UNAVAILABLE 이거나 5 번 연달아 실패하면 연결 단계로 돌아가 `wait_connected` → **스펙 재조회**
  → 제어 모드 조회 순으로 다시 붙는다. deadline 만 이어지면(TCP 가 소리 없이 죽은 경우) 채널을 새로 만든다.
  끊기기 전 제어 모드는 잊는다. 재연결 뒤 **첫 조회 성공**에서 MORAI 가 KEYBOARD 인데 마지막 요청이 AUTO 였으면
  AUTO 로 되돌린다(`auto_reassert_auto`).
-->
- **재연결**: 스냅샷이 UNAVAILABLE 이거나 5 번 연달아 실패하면 연결 단계로 돌아가 `wait_connected` → **스펙 재조회**
  → 제어 모드 조회 순으로 다시 붙는다. 이때 **두 채널을 새로 만든다** — UNAVAILABLE 이면(서버 재기동: 상태 채널만 끊김을
  알고 명령 채널은 옛 연결로 READY 에 남아 첫 명령 RPC 가 'Socket closed' 로 실패했다, 가짜 MORAI 로 확인) 그리고
  deadline 만 이어지면(TCP 가 소리 없이 죽은 경우). 끊기기 전 제어 모드는 잊는다. 재연결 뒤 **첫 조회 성공**에서 MORAI 가
  KEYBOARD 인데 마지막 요청이 AUTO 였으면 AUTO 로 되돌린다(`auto_reassert_auto`). 그 RPC 가 MORAI 에 닿지 못했으면
  (되읽기 전 전송 오류) 다음 모드 조회(~100 ms)마다 다시 한다 (합쳐 3 번). MORAI 가 받고도 AUTO 가 안 되면 되풀이하지 않는다. 요청 모드는 `control_mode_request` RPC **전에** 기록한다 (disengage 도중
  재연결이 끼어도 AUTO 로 되돌리지 않게).
- **Ego 없음**: Ego 가 1 초 넘게 없거나 전 필드 0 이면(시나리오 교체) 연결 단계로 가지 않고, 다시 보이는 첫 표본에서
  스펙만 다시 읽는다(실패하면 0.5 s 마다). 상태·GT 는 곧바로 다시 나가고, 명령은 스펙을 읽을 때까지 보류한다.
  제어 모드는 건드리지 않는다(재연결이 아니다). 램프는 다시 보낸다.
- **제어 모드 보고**: 조회(별도 스레드 10 Hz)가 0.5 s 넘게 실패하면 ControlModeReport 를 내지 않는다 — 낡은 AUTONOMOUS 가
  KEYBOARD 전환을 가리지 않게 (main.py 는 조회 실패 즉시 보고를 멈췄다).
- **종료**: SIGINT 때 최근 1 초 안에 명령을 보내고 있었으면 ~0.3 s 동안 업데이트마다 감속 대체 명령을 보내고 끝난다.
  <!-- 26.09.25 _ YSH _ timer 트리거 --> `command_trigger=timer` 면 업데이트마다가 아니라 `cmd_resend_hz` 주기로(100 Hz 면 30 회).
- **생성자는 막히지 않는다.** MORAI 가 없어도 노드는 뜨고, 파라미터 서비스는 바로 응답한다.

<!-- 26.09.25 _ YSH _ 신규 절 — 명령 트리거 A/B (main.py 조건 재현) -->

### 명령 트리거 A/B — `command_trigger`

알려진 정상 조건은 **MORAI 기본(~50 Hz 업데이트) + main.py 가 최신 명령을 고정 100 Hz 로 동기 재송신**
(`_CmdResender`)이었다. C++ 노드(업데이트당 1 회)로 폐루프를 돌렸을 때 MORAI 가 실제로 적용한 throttle 이 가속 중
0 ↔ on 을 초당 ~4 번 오가고 제동 페달이 약했다 — 원인이 송신 방식인지 가르려고 **같은 노드 안에서** 두 방식을 고른다.

| `command_trigger` | 누가 ControlVehicle 을 보내나 | 주기 |
| ----------------- | ---------------------------- | ---- |
| `update` (**기본**) | 엔진 루프 — 받아들인 MORAI 업데이트마다 정확히 1 회 | MORAI 업데이트율 (~50/s, 업데이트당 1) |
| `timer`           | 전용 명령 스레드 — 최신 명령을 동기 송신. 엔진 루프는 **보내지 않는다** (이중 송신 없음) | `cmd_resend_hz` 고정 (기본 100 Hz → MORAI 50 Hz 면 업데이트당 ~2) |

`timer` 는 main.py `_CmdResender` 와 같은 규칙이다: steady 시계 **절대 deadline**(`다음 = 이전 + 주기`), 송신이 늦어
deadline 을 넘기면 **몰아 보내지 않고** 기준을 지금으로 다시 잡고 `지연재설정`(late)을 센다, 호출은 동기 1 개씩
(deadline 은 update 와 같은 max(`morai.command_deadline_ms`, 4 × 왕복 p50)). 명령을 만드는 경로는 **같다** — 조향 라디안 +
결손 보정(새 메시지를 처음 보낼 때 **가장 최근 표본**의 속도로 한 번), stale 0.2 s → −2.4 m/s² 감속(조향 유지), NaN 은
버리고 마지막 유효 명령 유지, `forward_control_to_sim`, 중복 writer 가드(막히는 순간 ~0.3 s 감속), 종료 감속. 연결 단계
(재연결·스펙 대기) 중에는 보내지 않고(엔진의 재연결 백오프를 따른다 — 막힌 링크에 100 Hz 로 두드리지 않는다), 첫 유효
표본·스펙 전에도 보내지 않는다. main.py 처럼 MORAI 가 멈춰도(업데이트 없음, 링크는 살아 있음) 최신 명령을 계속 보낸다.

```bash
# 26.09.25 _ YSH _ launch 인자는 없앴다 (인자 기본값이 yaml 을 덮어써 timer 설정이 무시됐다). GT 기본은 이제 yaml 의
#   command_trigger: "timer", cmd_resend_hz: 100.0 (config/morai_interface.gt.param.yaml). 원문:
# # main.py 조건 재현: MORAI 기본 50 Hz + command_trigger:=timer cmd_resend_hz:=100
# ros2 launch autoware_morai_interface morai_interface_gt.launch.xml command_trigger:=timer cmd_resend_hz:=100
# main.py 조건 재현: MORAI 기본 50 Hz + (GT yaml 기본) timer 100 Hz
ros2 launch autoware_morai_interface morai_interface_gt.launch.xml
# 주행 중 전환 (둘 다 런타임 파라미터 — 노드 재시작 없이 A/B, e2e launch_morai_interface:=true 로 띄웠을 때도)
ros2 param set /morai_bridge command_trigger timer
ros2 param set /morai_bridge command_trigger update
ros2 param set /morai_bridge cmd_resend_hz 50      # 10~400, 정수도 받는다. 밖이면 거부
```

전환하면 노드가 `command_trigger → timer (100 Hz 고정 주기, main.py 조건)` INFO 를 남긴다 (bag·MORAI 기록과 맞출 시각).
비교는 `/diagnostics` `morai_bridge: command` 의 `command_trigger`·`sends_per_s`·`sends_per_update`(update ≈ 1, timer 100 Hz ≈ 2)·
`send_gap_p50/p99/max_ms`·`cmd_timer_late_window` 와 10 초 `제어 재송신` 줄(`목표 100 Hz`, `지연재설정`, `최대간격`)로 한다.

**기본이 `update` 인 이유**: (1) 설계 근거 실측 — 명령을 비동기로 몰아 보내면(렌더 프레임당 3 개 이상) MORAI 가속 제어기가
진동했다, (2) 업데이트당 1 회는 MORAI 프레임에 맞춰 규칙적이고 MORAI 가 멈추면 같이 멈춘다, (3) 가짜 MORAI 종단 시험(20/20)을
거친 경로다. `timer` 는 알려진 정상 조건(main.py)을 같은 노드에서 재현하는 A/B 용이다 — 실측으로 더 낫다고 나오면 그때
기본을 바꾼다.

## I/O

<!-- 26.09.25 _ YSH _ 토픽을 main.py 와 같은 절대 이름으로 코드에 둔다 (remap 불필요). 주기는 MORAI 업데이트(~50 Hz) 기준 -->
토픽은 main.py 와 같은 **절대 이름**이고 QoS 는 전부 depth 10 (RELIABLE, VOLATILE — rclcpp 기본)이다.
TF 는 `tf2_ros::TransformBroadcaster` 기본(depth 100).

<!-- 26.09.25 _ YSH _ 원문
토픽은 노드 안에서 `~/input/*` `~/output/*` 로 선언하고
[vehicle_interface.launch.xml](../../../../launcher/autoware_launch/vehicle/morai_vehicle_launch/morai_vehicle_launch/launch/vehicle_interface.launch.xml)
에서 아래 절대 이름으로 remap 한다.
-->

### SIM → ADS

| 토픽                                              | 타입                   | Hz (MORAI 50/s 기준) | 비고                                    |
| ------------------------------------------------- | ---------------------- | -------------------- | --------------------------------------- |
| `/vehicle/status/velocity_status`                 | `VelocityReport`       | ~50 (업데이트마다)   | frame `base_link`                       |
| `/vehicle/status/steering_status`                 | `SteeringReport`       | ~50                  |                                         |
<!-- 26.09.26 _ YSH _ 차량 리포트 전부 업데이트마다·같은 stamp 로 통일. 원문 행:
| `/vehicle/status/gear_status`                     | `GearReport`           | ~25 (2 번에 1 번)    | main.py 20                              |
| `/vehicle/status/control_mode`                    | `ControlModeReport`    | ~25                  | 조회는 ~10 Hz                           |
| `/vehicle/status/turn_indicators_status`          | `TurnIndicatorsReport` | 10 (타이머)          | 로컬 에코, stamp 는 이 PC 시각          |
| `/vehicle/status/hazard_lights_status`            | `HazardLightsReport`   | 10 (타이머)          | 로컬 에코                               |
-->
| `/vehicle/status/gear_status`                     | `GearReport`           | ~50                  | main.py 20                              |
| `/vehicle/status/control_mode`                    | `ControlModeReport`    | ~50                  | 조회는 별도 스레드 10 Hz                |
| `/vehicle/status/turn_indicators_status`          | `TurnIndicatorsReport` | ~50                  | 마지막 명령의 로컬 에코, 업데이트 stamp |
| `/vehicle/status/hazard_lights_status`            | `HazardLightsReport`   | ~50                  | 로컬 에코, 업데이트 stamp               |
| `/localization/kinematic_state`                   | `nav_msgs/Odometry`    | ~50                  | GT. `publish_ground_truth`              |
| `/tf` (`map`→`base_link`)                         | `TFMessage`            | ~50                  | kinematic_state 와 같은 stamp·pose      |
<!-- 26.09.26 _ YSH _ 원문 행:
| `/perception/object_recognition/tracking/objects` | `TrackedObjects`       | ~10 (5 번에 1 번)    | GT. Ego 제외, id = uuid5                |
-->
| `/perception/object_recognition/tracking/objects` | `TrackedObjects`       | 10 (MORAI 시각 100 ms) | GT. Ego 제외, id = uuid5              |
| `/clock`                                          | `rosgraph_msgs/Clock`  | ~50                  | `publish_clock: true` 일 때만            |
| `/diagnostics`                                    | `DiagnosticArray`      | 1                    | link·update·command·clock·ground_truth  |

<!-- 26.09.26 _ YSH _ 원문:
실측(가짜 MORAI 50/s): velocity/steering/kinematic 50.0, gear/mode 25.0, objects 10.0, lamps 10.0 Hz.
-->
실측(가짜 MORAI 49.5/s, 2026-09-26): velocity·steering·kinematic·tf·gear·mode·turn·hazard 49.48 Hz(모두 업데이트율, 같은 stamp),
objects 10.00 Hz. 전에는 실제 MORAI 에서 상태가 41.5 Hz 였다(엔진 루프 안 모드 조회가 5 업데이트마다 한 프레임을 놓쳤다).

한 업데이트의 출력은 **모두 같은 스냅샷에서 만들고 같은 stamp 를 쓴다** (`time_source`: 기본 MORAI 샘플 시각,
`receive` 면 이 PC 수신 시각). **새 유효 표본이 없으면 아무것도 발행하지 않는다** — 0 을 내거나 이전 샘플에 새
stamp 를 찍으면 EKF·제어기가 진짜 정지와 구분하지 못한다. `max_sample_age_s`(0.1 s) 넘게 새 표본이 없으면
`morai_bridge: update` 진단이 ERROR 다.

<!-- 26.09.25 _ YSH _ 원문 표·설명 (50 Hz 폴링 타이머 시절)
| `/vehicle/status/velocity_status`        | `VelocityReport`       | 50  |
| `/vehicle/status/steering_status`        | `SteeringReport`       | 50  |
| `/vehicle/status/gear_status`            | `GearReport`           | 50  |
| `/vehicle/status/control_mode`           | `ControlModeReport`    | 50  |
| `/vehicle/status/turn_indicators_status` | `TurnIndicatorsReport` | 10  |
| `/vehicle/status/hazard_lights_status`   | `HazardLightsReport`   | 10  |

앞의 네 개는 **한 번의 `GetActorState` 폴링에서 같이 만들어지고 같은 타임스탬프를 쓴다.**
타이머를 나누면 리포트끼리 서로 다른 샘플을 보게 되기 때문이다.
뒤의 두 개는 MORAI 에 read-back 이 없어 마지막 명령을 에코하는 것이다 (아래 참조).
-->

### ADS → SIM

| 엔드포인트                             | 타입                                     | MORAI RPC                               |
| -------------------------------------- | ---------------------------------------- | --------------------------------------- |
| `/control/command/control_cmd`         | `autoware_control_msgs/Control`          | `ControlVehicle` — MORAI 업데이트당 1 회 |
| `/control/command/gear_cmd`            | `GearCommand`                            | `SetVehicleGear` — 변경 시 (+재시도·재동기) |
| `/control/command/turn_indicators_cmd` | `TurnIndicatorsCommand`                  | `SetVehicleTailLight` — 변경 시          |
| `/control/command/hazard_lights_cmd`   | `HazardLightsCommand`                    | `SetVehicleTailLight` — 변경 시          |
| `/control/control_mode_request` (srv)  | `ControlModeCommand`                     | `SetVehicleControlMode` + 되읽기 확인    |
| `/initialpose`                         | `geometry_msgs/PoseWithCovarianceStamped` | `SetTransform` + 다음 스냅샷 위치 확인(0.5 m) |
| `/localization/initialize` (client)    | `autoware_localization_msgs/InitializeLocalization` | — (첫 유효 표본 뒤 DIRECT 1 회, 1 Hz 재시도) |

`forward_control_to_sim` 이 `false` 면 제어 · 기어 · 제어모드 요청이 모두 차단된다 (램프는 차단하지 않는다).
다시 켜도 제어는 새 `control_cmd` 가, 기어는 새 `gear_cmd` 가 와야 보낸다 (main.py 와 같음). 램프는 다시 켜는
순간 강제로 재전송한다 — 차단 중 사람이 키보드로 바꿨을 수 있다.

`raw_vehicle_cmd_converter` 는 쓰지 않는다. MORAI 가 가속도 명령을 직접 받으므로
(`long_cmd_type = ACCELERATION`) 페달 맵 변환이 필요 없다. AWSIM 인터페이스와 갈리는 지점이다.

<!-- 26.09.25 _ YSH _ 원문 (콜백에서 바로 blocking RPC 를 부르던 시절)
기어와 램프는 **값이 바뀔 때만** 전송한다. 명령이 계속 들어오는데 매번 보내면
executor 스레드에서 blocking unary RPC 가 그 주기로 발생한다.
제어모드는 서비스라 호출 자체가 드물어 매번 전송한다 (변경 감지 없음).

`forward_control_to_sim` 이 `false` 면 제어 · 기어 · 제어모드가 모두 차단된다.
`true` 로 되돌리는 순간 기어와 램프 캐시가 비워져 다음 명령이 반드시 재전송된다 —
차단 중 사람이 시뮬 키보드로 기어를 바꿨을 수 있기 때문이다.
-->

## 부호 규약 — 두 방향이 서로 반대다

**이건 실주행 사고가 났던 지점이고, 의도적으로 비대칭이다.** 컨트롤러가 우측
(−0.415 rad) 을 명령하는 동안 차량이 좌측 풀락 (+0.632 rad) 에 붙어 조향 레이트
제한까지 포화된 적이 있다.

<!-- 26.09.25 _ YSH _ 스케일 행 교체: 명령은 정규화하지 않는다 (main.py eba6b04, 옛 ÷max 는 1.43 배 과조향) -->
|        | 리포트 (상태)                                           | 명령                                                         |
| ------ | ------------------------------------------------------- | ------------------------------------------------------------ |
| 파일   | `conversion/morai_to_autoware.hpp`                      | `conversion/autoware_to_morai.hpp`                           |
| 부호   | **하드코딩 `-1.0`**, 설정 불가                          | **파라미터 `steer_cmd_sign`**, 기본 `+1.0`                   |
| 스케일 | `× max_steering_angle × steer_comp_gain`                | **라디안 그대로** `÷ steer_comp_gain`, `±max_steering_angle` 클램프 |
| 근거   | MORAI `front_wheel_angle` 은 Unity 좌표계라 ROS 와 반대 | 명령 필드는 ROS 와 같은 규약 (양수 = 좌)                     |

<!-- 26.09.25 _ YSH _ 원문 스케일·근거 행
| 스케일 | `× max_steering_angle`                                  | `÷ max_steering_angle`                                                                        |
| 근거   | MORAI `front_wheel_angle` 은 Unity 좌표계라 ROS 와 반대 | `actor_control.proto:29-30` — `steer = 원하는 스티어링 값(radian) / 최대 조향각`, ROS 와 동일 |
-->

두 규칙을 **서로 다른 파일에 두었고**, `CommandAndReportSignsAreIntentionallyOpposite`
테스트가 이를 고정한다. 하나로 합치려는 시도는 즉시 테스트 실패로 드러난다.

**명령 조향은 라디안이다.** proto 주석은 `[-1,1] = 라디안 / 최대 조향각` 이라고 하지만 MORAI 는 `steer` 를 타이어각
라디안으로 적용한다: 0.487 rad 명령이 이미 기계 한계 0.698 rad 로 읽혔고, 왕복 게인 회귀가 1.4331 = 1/0.698 이었다
(main.py eba6b04). 옛 C++ 처럼 나누면 횡방향 루프가 1.43 배 과하게 돈다.
**조향 결손 보정**(`steer_comp_*`, 기본 켬): MORAI 차는 명령보다 덜 돈다(곡률비 2 m/s 0.870 … 11 m/s 0.796).
보낼 때 속도별 배율로 나누고, 리포트는 보낸 명령의 에코라 같은 배율로 곱해 되돌린다. 배율은 **새 `control_cmd`
를 처음 보낼 때 그때 속도로 한 번** 정한다 (main.py 가 콜백에서 정하던 것과 같다).

`front_wheel_angle` 은 각도가 아니라 **정규화값 [-1, 1]** 이다 (풀락 = ±1.0).
`max_steering_angle` 은 MORAI 가 **degree** 로 주며 (`actor_get.proto:112`)
연결(재연결)할 때마다 한 번 radian 으로 변환한다 (main.py 식 `deg / (180/π)`).
**조향 리포트 0 드롭아웃 유지**(`steer_report_hold`): MORAI 유휴 레코드의 `front_wheel_angle = 0` 은 마지막으로
보낸 조향이 1e-3 rad 를 넘으면 직전값으로 둔다. 주행 표본 500 개마다 유지 비율을 WARN 으로 남긴다.

<!-- 26.10.01 _ YSH _ 외부 조향 필터 -->
**외부 조향 필터**(`steer_report_ignore_foreign`, 기본 끔): MORAI 시나리오가 도는 동안 Ego 에 우리가 보내지 않은 조향이
적용되는 프레임이 있다(09-29 44런 11.5 %). `front_wheel_angle` 은 그 입력값과 같아서 리포트가 그대로 튄다. 켜면
`−front_wheel_angle × steer_input_norm_deg` 가 최근 300 ms 에 보낸 조향 중 하나와 0.0001° 안으로 같을 때만 그대로 내고,
아니면 직전 우리 값을 낸다. 연속 25 프레임을 넘으면 MORAI 값으로 돌아가고, 주행 표본 500 개 중 일치가 20 % 아래면
(기준각이 안 맞는 차량) 스스로 꺼진다. 차가 실제로 꺾이는 것은 막지 못한다 — Autoware 가 튄 리포트에 반응하는 것만 없앤다.

## 그 밖의 변환

<!-- 26.09.25 _ YSH _ heading_rate·각속도 행 교체 (morai_coordinates.py), GT 행 추가 -->
| MORAI                                   | Autoware                                         | 비고                                                      |
| --------------------------------------- | ------------------------------------------------ | --------------------------------------------------------- |
| `velocity.x` [km/h]                     | `longitudinal_velocity` [m/s]                    | ×1/3.6, 부호 유지                                         |
| `velocity.y` [km/h]                     | `lateral_velocity` [m/s]                         | ×1/3.6, **부호 반전**                                     |
| `angular_velocity` (world rad/s, 축 (−z, x, −y)) | `heading_rate`, twist.angular [rad/s]     | **body 로 회전** (R = Rz·Ry·Rx, `morai_coordinates.py`)  |
| `transform` (ENU m, rotation deg)       | kinematic_state pose, TF                         | `map = R(yaw_off)·morai + offset` (main.py `get_map_pose`) |
| actor `bounding_box.extent` (반치수, x=반폭) | `Shape.dimensions` (x=길이)                   | (2y, 2x, 2z), 하한 0.1 m                                  |
| actor id                                | `object_id`                                      | `uuid5(NAMESPACE_OID, "morai:"+id)` — Python 과 바이트 동일 |
| `GearMode` M/P/R/N/D/L                  | `GearReport` NONE/PARK/REVERSE/NEUTRAL/DRIVE/LOW |                                                           |
| `GearCommand` 2~19 등 **범위**          | `GearMode`                                       | 리포트 매핑의 역이 아니다                                 |
| `VehicleControlMode` 0~7                | `ControlModeReport`                              | 아래 참조                                                 |

<!-- 26.09.25 _ YSH _ 원문 행 (틀림: MORAI 각속도는 body deg/s 가 아니다)
| `angular_velocity.z` [deg/s]   | `heading_rate` [rad/s]                           | ×π/180, **부호 반전**     |
-->

### 램프 — 비대칭이 의도된 것

MORAI `VehicleTailLight` 는 필드가 하나(`VehicleTurnSignal`)인데 Autoware 는 방향지시등과
비상등을 독립적으로 보낸다. 게다가 MORAI 는 램프 read-back 을 제공하지 않는다.

- `LampCommandCache` 가 두 명령을 모두 들고 있고, 병합해서 보낸다. **hazard 우선.**

  ```text
  hazard ENABLE          → ALL(3)
  else turn ENABLE_LEFT  → LEFT(1)
  else turn ENABLE_RIGHT → RIGHT(2)
  else                   → OFF(0)
  ```

- 상태 방향은 그 캐시를 그대로 에코한다. **MORAI 데이터가 개입하지 않으므로
  `morai_to_autoware` 에 대응 함수가 없다** — 대칭처럼 보이는 함수는 데이터 흐름에
  대해 거짓말이 된다.
- 램프는 `forward_control_to_sim` 게이트를 받지 않는다. 누가 운전하느냐와 무관하고,
  살려두면 사람이 시뮬 키보드로 몰 때도 리포트가 사실을 유지한다.
- <!-- 26.09.25 _ YSH _ 추가 --> 전송은 작업 스레드가 값이 바뀔 때만 한다. (재)연결 때마다 현재 값을 강제로 다시 보낸다.

### 제어 모드

<!-- 26.09.25 _ YSH _ CRUISE 는 main.py 대로 DISENGAGED(5) 로 되돌렸다 -->
| MORAI                     | Autoware                   |                                                                   |
| ------------------------- | -------------------------- | ----------------------------------------------------------------- |
| `UNSPECIFIED`             | `NO_COMMAND`               |                                                                   |
| `KEYBOARD` / `GAME_WHEEL` | `MANUAL`                   |                                                                   |
| `AUTO_MODE`               | `AUTONOMOUS`               |                                                                   |
| `AUTO_MODE_LATERAL`       | `AUTONOMOUS_STEER_ONLY`    |                                                                   |
| `AUTO_MODE_LONGITUDINAL`  | `AUTONOMOUS_VELOCITY_ONLY` |                                                                   |
| `CRUISE_MODE`             | `DISENGAGED` (5) + WARN    | main.py 와 같다. MORAI 가 스스로 운전하는 제2 제어원을 가리지 않는다 |
| `SYNCHRONOUS_MODE`        | `AUTONOMOUS`               | main.py 는 미매핑(`NOT_READY`) — lockstep 에서도 운전은 Autoware |
| 그 외                     | `NOT_READY`                |                                                                   |

<!-- 26.09.25 _ YSH _ 원문 CRUISE 행
| `CRUISE_MODE`             | `AUTONOMOUS_VELOCITY_ONLY` | **변경** — 구 브리지는 `DISENGAGED`(5) 를 냈다          |
-->

`autoware_operation_mode_transition_manager` 는 `AUTONOMOUS` 계열이 아니면 "Autoware 가 제어 중이 아님" 으로 보고
transition timeout 후 engage 를 취소한다. `control_mode_request` 의 MANUAL 은 MORAI KEYBOARD 로 보낸다 (그 뒤로는
AUTO 재지정을 하지 않는다).

<!-- 26.09.25 _ YSH _ 파라미터 표 전면 교체 — 전부 코드 기본값이 있다 (yaml 없이 기동) -->

## 파라미터

GT 모드 파일은 [config/morai_interface.gt.param.yaml](config/morai_interface.gt.param.yaml), 스키마는
[schema/morai_interface.schema.json](schema/morai_interface.schema.json). **모든 파라미터에 코드 기본값이 있다.**
"런타임" 표시는 `ros2 param set /morai_bridge <이름> <값>` 으로 재시작 없이 바뀐다(main.py 가 콜백마다 읽던 것들).
나머지는 기동 시 한 번 읽으며 read-only 로 선언돼 `param set` 이 거부된다.

| 이름                               | 기본값                           | 런타임 | 설명                                                          |
| ---------------------------------- | -------------------------------- | ------ | ------------------------------------------------------------- |
| `morai.address`                    | env `MORAI_SIM_ADDRESS` 또는 `10.10.10.1` |   | MORAI PC 주소 (main.py 와 같은 환경변수)                     |
| `morai.port`                       | env `MORAI_SIM_PORT` 또는 `7789` |        | gRPC 포트                                                     |
| `morai.ego_name`                   | `Ego`                            |        | `ObjectInfo.id.value` (문자열)                                |
| `morai.wait_for_tick_deadline_ms`  | `100`                            |        | 업데이트 트리거 대기                                          |
| `morai.state_deadline_ms`          | `100`                            |        | GetAllActorsState / GetVehicleControlMode                     |
| `morai.command_deadline_ms`        | `50`                             |        | ControlVehicle 하한 (main.py `cmd_rpc_timeout_s`). 실제 = max(이 값, 4×왕복 p50), 상한 1 s |
| `morai.gear_deadline_ms`           | `200`                            |        | SetVehicleGear (main.py `gear_rpc_timeout_s`)                 |
| `morai.control_mode_deadline_ms`   | `200`                            |        | SetVehicleControlMode                                         |
| `morai.transform_deadline_ms`      | `1000`                           |        | SetTransform                                                  |
| `morai.tail_light_deadline_ms`     | `200`                            |        | SetVehicleTailLight                                           |
| `morai.spec_deadline_ms`           | `3000`                           |        | GetVehicleSpec                                                |
| `morai.keepalive_time_ms`          | `0` (끔)                         |        | gRPC keepalive                                                |
| `forward_control_to_sim`           | `true`                           | 예     | `false` = 제어·기어·모드 요청 차단 (키보드 수동 주행)         |
| `steer_cmd_sign`                   | `1.0`                            | 예     | 위 "부호 규약"                                                |
| `steer_comp_enable`                | `true`                           | 예     | 조향 결손 보정                                                |
| `steer_comp_speeds_mps`            | `[2.0, 5.0, 8.0, 11.0]`          | 예     | 보정 표 속도 (엄격 오름차순)                                  |
| `steer_comp_gains`                 | `[0.870, 0.855, 0.807, 0.796]`   | 예     | 보정 표 배율 (0.5~1.5). 표가 잘못되면 배율 1.0 + WARN         |
| `long_cmd_mode`                    | `acceleration`                   | 예     | `acceleration` \| `throttle` \| `velocity` (그 밖의 값은 거부)  |
| `send_pedal_with_accel`            | `false`                          | 예     | acceleration 모드에서 페달값 병행                            |
| `pedal_accel_max`                  | `3.0`                            | 예     | throttle = a / 이 값 [m/s²]                                   |
| `pedal_brake_max`                  | `4.5`                            | 예     | brake = −a / 이 값 [m/s²]                                     |
| `velocity_cmd_unit`                | `kph`                            |        | `VehicleCtrlCmd.velocity` = m/s × 3.6 \| × 1 \| 0             |
| `cmd_stale_timeout_s`              | `0.2`                            |        | 이보다 오래된 `control_cmd` 는 감속 대체 명령                 |
| `cmd_stale_accel`                  | `-2.4`                           |        | 감속 대체 명령 가속도 (음수만, 아니면 −2.4)                   |
| `command_trigger`                  | `update`                         | 예     | <!-- 26.09.25 _ YSH _ --> `update` (MORAI 업데이트당 1 회) \| `timer` (`cmd_resend_hz` 고정 주기, main.py `_CmdResender`) — 위 "명령 트리거 A/B" |
| `cmd_resend_hz`                    | `100.0`                          | 예     | <!-- 26.09.25 _ YSH _ --> `timer` 일 때 ControlVehicle 주기 [Hz] (main.py 이름·기본값, 10~400, 정수 허용). `update` 면 안 쓴다 |
| `gear_resync_s`                    | `1.0`                            |        | MORAI 기어 불일치 재송신 간격                                 |
| `gear_retry_s`                     | `0.2`                            |        | 기어 송신 실패 재시도 간격                                    |
| `steer_report_hold`                | `true`                           | 예     | 조향 리포트 0 드롭아웃 유지                                   |
| `steer_report_ignore_foreign`      | `false`                          | 예     | <!-- 26.10.01 _ YSH _ --> 외부 조향 필터 — 우리가 보내지 않은 조향이 적용된 프레임의 리포트를 직전 우리 값으로 (launch 가 환경변수 `MORAI_STEER_FILTER` 로 덮는다) |
| `steer_input_norm_deg`             | `40.0`                           | 예     | <!-- 26.10.01 _ YSH _ --> MORAI `front_wheel_angle` 1.0 의 조향각 [deg] (외부 조향 판정용) |
| `map_origin_offset_x/y/z`          | `0.0`                            | 예     | MORAI ENU → map 평행이동 [m]                                  |
| `map_origin_yaw_offset_deg`        | `0.0`                            | 예     | MORAI ENU → map 회전 [deg]                                    |
| `teleport_on_initialpose`          | `true`                           | 예     | `/initialpose` → MORAI Ego 순간이동                           |
| `publish_ground_truth`             | `true`                           |        | kinematic_state·TF·TrackedObjects·측위 초기화                 |
| `time_source`                      | `morai`                          |        | `morai` (MORAI 샘플 시각) \| `receive` (이 PC 수신 시각)       |
| `publish_clock`                    | `false`                          |        | `/clock` 발행 (use_sim_time 쓸 때만). 값은 메시지 stamp 와 같다 (`receive` 면 수신 시각) |
| `max_sample_age_s`                 | `0.1`                            |        | 새 표본이 이보다 오래 없으면 진단 ERROR                       |
| `actor_hold_s`                     | `0.5`                            |        | **예약 — 지금은 효과 없음.** 액터가 Ego 와 같은 호출에서 와 따로 낡지 않는다 (새 표본이 없으면 TrackedObjects 도 멈춘다) |
| `clock_offset_warn_ms`             | `20.0`                           |        | 시계 오프셋 WARN 기준                                         |
| `fallback_hz`                      | `100.0`                          |        | WaitForTick 폴백 폴링 주기                                    |
| `auto_reassert_auto`               | `true`                           |        | 재연결 뒤 KEYBOARD 면 AUTO 재지정 (마지막 요청이 AUTO 일 때)  |
| `engage_on_start`                  | `false`                          |        | 첫 연결 때 MORAI 를 AUTO 로                                   |
| `sensor_model`                     | `""`                             |        | 비우면 센서를 건드리지 않는다 (GT 모드 기본)                  |
| `duplicate_writer_guard`           | `true`                           |        | 다른 velocity_status publisher 가 있으면 ControlVehicle 차단 (`false` = 진단만) |

<!-- 26.09.25 _ YSH _ 원문 표
| `morai.address`           | `10.10.10.1`               | MORAI SIM 이 도는 PC 의 IP                                    |
| `morai.port`              | `7789`                     | gRPC 포트 (시뮬 Network Settings)                             |
| `morai.ego_id`            | `Ego`                      | `ObjectIdentifier.value`. 숫자가 아니라 **문자열**            |
| `morai.client_key`        | `autoware_morai_interface` | MORAI 가 되돌려주는 추적용 문자열                             |
| `morai.rpc_deadline_ms`   | `100`                      | RPC 당 deadline. 시뮬 무응답 시 호출자가 붙잡히는 상한        |
| `forward_control_to_sim`  | `true`                     | `false` 면 제어·기어를 MORAI 로 안 보냄. **런타임 전환 가능** |
| `steer_cmd_sign`          | `1.0`                      | 위 "부호 규약" 참조                                           |
| `connection_retry_period` | `1.0`                      | 연결·제원조회 재시도 주기 [s]                                 |
-->

측위·인지를 디버깅하면서 시뮬 키보드로 직접 몰고 싶을 때.

```bash
ros2 param set /morai_bridge forward_control_to_sim false
```

SIM → ADS 방향은 그대로라 Autoware 는 계속 관측한다. 기어 명령도 같이 막힌다 —
수동 주행 중 Autoware 가 P/N 으로 바꿔버리면 차를 몰 수 없기 때문이다.

<!-- 26.09.25 _ YSH _ 신규 절 — 진단 -->

<!-- 26.09.26 _ YSH _ 신규 절 — DAQ 를 브리지 노드 안으로 -->
## DAQ — ViLS 명세 25 항목 100 Hz (`publish_daq`)

`daq_publisher.py`(MORAI 에 따로 붙어 초당 200 RPC)를 대신해 이 노드가 `/daq/*` 를 낸다. 엔진이 업데이트마다 받은
**같은 스냅샷**(Ego + NPC + MORAI 시각)을 전용 스레드가 10 ms 마다 다시 낸다(~50 Hz 표본의 ZOH). 운전 경로(엔진 루프·
명령 타이머·50 Hz 상태)와 락을 공유하지 않는다.

| 토픽 | 타입 | 명세 | 비고 |
| --- | --- | --- | --- |
<!-- 26.10.05 _ YSH _ 명세 토픽을 MORAI 공식 메시지(morai_ros2_msgs 26.R1, 같은 저장소 simulator/morai_ros2_msgs)로 바꿨다. 원래 세 줄은
| `/daq/vehicle_state` | `nav_msgs/Odometry` | 1~6, 10~12 | kinematic_state 와 같은 식 + vz (kinematic_state 는 main.py 대로 vz = 0) |
| `/daq/acceleration` | `AccelWithCovarianceStamped` | 7~9 | frame `base_link`, (x, −y, z) × `daq.accel_scale` |
| `/daq/track_objects` | `TrackedObjects` | 13~24 | tracking/objects 와 같은 객체 + 가속도, NPC 없어도 빈 목록 |
-->
| `/daq/ego_status` | `morai_ros2_msgs/EgoVehicleStatus` | 1~9, 12 | position m · velocity m/s(local, MORAI 축) · acceleration m/s²(local) × `daq.accel_scale` · angular_velocity deg/s(gRPC 원값 축) · heading deg · accel·brake = 적용 페달 · front_steer_angle deg(좌 +). gRPC 에 없는 칸은 0 |
| `/daq/object_status` | `morai_ros2_msgs/ObjectStatusList` | 13~24 | type 1 NPC → `npc_list`, 0 보행자 → `pedestrian_list`, 2 정적 → `obstacle_list`. velocity = `global_velocity` km/h(world), size = (폭, 길이, 높이) m, unique_id = uuid5("morai:"+이름) 앞 4바이트(CSV `objN_id_u32` 와 같은 비트), NPC 없어도 빈 목록 |
| `/daq/raw/actor_states` | `std_msgs/String` | 10~11 | roll·pitch (MORAI 메시지에 칸이 없다) — 아래 원시 기록. **`daq.raw` 필수** |
| `/daq/vehicle_cmd` | `autoware_control_msgs/Control` | 25 | Autoware control_cmd 원본 그대로. 새 명령이 `daq.cmd_stale_s`(0.2 s) 없으면 멈춘다 (0 을 지어내지 않음 — 앞 런 값이 남지 않는다) |

<!-- 26.10.05 _ YSH _ 원문: - **값**: `daq_publisher.py` 와 같은 식·연산 순서라 비트 단위로 같다 (gtest golden). 다른 점은 ego·객체가 한 스냅샷에서 온다는 것과
  `map_origin_*` 를 런타임에 반영한다는 것뿐이다. -->
- **값**: gRPC 스냅샷을 MORAI 정의 단위로만 바꿔 담는다 — 축 반전·map 변환(`map_origin_*`) 없음, MORAI world 그대로. 제출 CSV 는
  `daq_export` 가 옛 부호(ROS, y 반전)로 되돌려 지금과 같은 열을 만든다 (명세 20~21 만 `global_velocity` 로 바뀐다). 옛 bag
  (`/daq/vehicle_state` 등)도 계속 내보낼 수 있다. 운용 `map_origin_*` 은 0 이라 map = MORAI world.
- **stamp**: 기본 `daq.stamp: sample` — MORAI 표본 시각(= kinematic_state stamp, 반복 행은 같은 stamp), vehicle_cmd 는 control_cmd
  원래 stamp. 오프라인 도구(`daq/`, `daq_export`, `daq_verify`)는 bag 수신 시각을 쓰므로 영향이 없다. `tick` = 예전처럼 발행 시각.
- **게이트** (`daq.mode: gated`, GT yaml 기본): cue_server 가 녹화 시작 **직전** `/daq/run_state` 에 `active <bag>`, 녹화 종료
  **직후** `idle <bag>` 을 낸다 (std_msgs/String, RELIABLE + TRANSIENT_LOCAL). 켜진 동안만 낸다 — `daq_verify` S5 가 `/daq` 표본을
  bag 전체 길이와 비교하므로 녹화 구간을 덮어야 한다. latched 라 런 도중 노드가 재시작돼도 곧바로 다시 받아 이어 낸다(가짜 MORAI
  실측 1.6 s). `idle <run>` 은 이름이 같은 런만 끝낸다(늦게 온 앞 런의 idle 이 다음 런을 끄지 않게), 이름 없는 `idle` 은 무엇이든.
  `active` 가 `daq.run_timeout_s`(1200 s) 넘게 이어지면 자동 종료. `daq.mode: always` (런타임) 면 게이트 없이 늘 낸다 — 수동 주행·
  `daq_record.py` 녹화용.
- **런 요약**: 끝날 때 `/daq/run_summary` (latched JSON: run, end_reason idle|timeout|replaced|shutdown, duration_s, state_hz,
  gap_ticks, longest_gap_ticks, link_down_ticks, yield_ticks, late, updates …). cue_server 가 run JSON 의 `daq` 키로 옮긴다.
  Ctrl-C 종료 때도 컨텍스트가 닫히기 전(pre-shutdown)에 멈춰 'shutdown' 요약이 나간다.
- **다른 writer**: 다른 노드가 이미 `/daq/ego_status` 를, 또는 `daq_publisher.py` 등이 옛 `/daq/vehicle_state` 를 내면 양보한다 (가드 타이머 1 s, 기동 직후 첫 판정 전에는
  조용히 대기). `daq_publisher.py` 는 반대로 노드가 이미 내면 시작을 거부한다(exit 3).
- **표본 결손**: 최신 표본이 `daq.max_sample_age_s`(0.1 s) 이상 낡으면 상태 2 토픽(ego_status·object_status)을 멈추고 결손 틱으로 센다 (링크 끊김은 따로).
- 진단 `morai_bridge: daq` — mode·게이트·창 Hz·결손·지연·표본/명령 나이·다른 writer.

검증 (2026-09-26): gtest 467 (변환 golden 비트 비교, 샘플러 가짜 시계 22 건), 가짜 MORAI e2e D0~D5 (게이트 idle 무발행, 4 토픽 100.0 Hz,
stamp = kinematic stamp·표본당 중앙 2 행, 명령 stale 0.2 s, 켜진 채 노드 재시작 → 1.6 s 뒤 재개 + shutdown 요약, idle → 멈춤 + 요약),
cue_server 순서로 녹화한 bag 이 `daq_export`·`daq_verify` 의 DAQ 항목(S3b·S5 100.1 %·S6·X1·C1·C2·P8) 통과.

<!-- 26.09.26 _ YSH _ 신규 절 — 원시 gRPC 기록 -->
### 원시 gRPC 기록 (`daq.raw`)

"gRPC 가 준 값을 그대로" 남긴다. 위 25 항목 토픽은 그대로 두고 두 토픽을 더한다 (`daq.raw: true`, 코드 기본 false, GT yaml true,
`publish_daq` 필요). 둘 다 `std_msgs/String` 에 JSON 한 개, RELIABLE·VOLATILE, depth = max(64, `daq.raw_queue_depth`).

| 토픽 | 언제 | `data` |
| --- | --- | --- |
| `/daq/raw/actor_states` | 받아들인 MORAI 업데이트마다 (~50/s, ZOH 아님 — 한 틱에 둘이 와도 둘 다) | `{"schema":"morai_raw.actor_states/1","run","update_index","rx_system_ns","morai_timestamp_us","actor_states":<GetAllActorsState 응답 전체>}` |
| `/daq/raw/vehicle_ctrl_cmd` | ControlVehicle 을 부를 때마다 (timer 100/s, 감속 대체·가드 감속 포함. 종료 감속은 빠진다 — 아래) | `{"schema":"morai_raw.vehicle_ctrl_cmd/1","run","send_seq","send_system_ns","stale","ok","grpc_code","rpc_ms","result_status","vehicle_ctrl_cmd":<보낸 요청 그대로>}` |

- **값**: 메시지 본문은 `MessageToJsonString` (proto 필드 이름, 기본값 원시 필드도 찍음, enum 은 정수). 단위·축·부호 변환 없음
  (location m, rotation deg, velocity km/h local, bounding_box.extent 반치수, vehicle_state 전 필드). int64/uint64 는 문자열
  (`"timestamp":"1790…"`), double 은 되읽으면 같은 값이 되는 자릿수(최대 17), NaN/Infinity 는 문자열. 없는 하위 메시지(장애물의
  `vehicle_state`)는 키가 없다. 알려진 한계 둘 — 받은 응답을 다시 직렬화하므로 −0.0 은 0 이 되고(값은 같다), 우리 proto 에 없는
  필드(MORAI 가 더 새 proto 를 쓸 때)는 JSON 에 없다.
- **명령**: `vehicle_ctrl_cmd` 는 링크가 보낸 요청과 같은 변환·같은 `actor_info` 로 다시 만든 것이다 (gtest 가 같음을 고정).
  `stale` = 감속 대체 명령, `grpc_code` −1 = 보내지 않음(비유한 값) 이고 그때 `rpc_ms` 는 null. `ok` 는 gRPC 전송 결과만이다
  (`grpc_code` 0 과 같다). MORAI 가 응답한 `Result.status` 는 `result_status` 에 따로 둔다 (1 = SUCCESS, 0 = UNKNOWN_FAILURE,
  응답이 없으면 null). `ok` 가 true 인데 `result_status` 가 1 이 아니면 "닿았지만 MORAI 가 거부", `ok` 가 false 면 "안 닿음 또는 모름"
  (`grpc_code` 4 DEADLINE_EXCEEDED 는 적용됐는지 모른다). `send_seq` 는 기록 여부와 무관하게 송신마다 +1 이라 빈 번호 = 게이트
  밖이나 큐 넘침이다. 명세 25 원형 = `vehicle_ctrl_cmd.velocity`(km/h, long_cmd_type 2 일 때만 MORAI 가 씀)와
  `vehicle_ctrl_cmd.acceleration`(ACCELERATION 모드에서 MORAI 가 따르는 값).
- **종료 감속**: 노드를 끌 때 엔진이 ~0.3 s 보내는 감속 명령은 기록되지 않는다. DAQ 가 엔진보다 먼저 멈추기 때문이다
  (Ctrl-C 면 컨텍스트가 닫히기 전에 `shutdown` 요약을 내야 한다). 첫 `stop` 뒤의 송신은 always 모드여도 `tx_discarded` 로 센다.
- **스레드**: 엔진(응답 바이트 포인터)·명령 스레드(명령 구조체)는 복사해 큐에 넣기만 하고, proto3 JSON 변환은 DAQ 스레드가 틱마다
  한다. 큐는 종류마다 `daq.raw_queue_depth`(256), 넘치면 가장 오래된 것을 버리고 센다.
- **게이트**: `/daq/*` 와 같다. 넣을 때와 낼 때 모두 발행 중이어야 나가고, 발행 중이 아닐 때 생긴 것은 버린다 (다음 런으로 넘어가지
  않는다). `run` 은 넣을 때 켜진 런 이름 (always 모드에서 런이 없으면 `""`). 한 틱 안에 `active A` → `active B` 가 오면
  A 의 남은 칸은 A 가 끝날 때 A 요약에 세고 다음 틱에 A 이름으로 낸다 (그사이 idle 이 와도) — 요약 수 = 그 런 이름으로 낸 수.
  `idle`·timeout 으로 끝난 런에 남은 칸(10 ms 미만)은 `/daq/*` 처럼 버린다 (요약에도 없다).
- **요약·진단**: 런 요약에 `raw_published`·`raw_dropped`·`tx_published`·`tx_dropped`, 10 초 `DAQ raw …` 줄, 진단 `daq` 에 누적 수
  (dropped = 큐 넘침 → WARN, discarded = 게이트 밖이라 버림 — 정상). 기동 로그에 `DAQ raw: …` 줄이 따로 나온다 (기존 `DAQ:` 줄은 그대로).

## 진단 (`/diagnostics`, 1 Hz)

| 항목                             | ERROR                                   | WARN                                   | 주요 값                                                  |
| -------------------------------- | --------------------------------------- | -------------------------------------- | -------------------------------------------------------- |
| `<노드>: link`          | 연결 안 됨 (재연결·스펙 대기)           | WaitForTick 폴백 중 / 왕복 p50 > 5 ms  | reconnects, link_losses, 채널별 호출·실패, deadline 초과, command_deadline_ms |
| `<노드>: update`        | 표본 없음 / 나이 > `max_sample_age_s` (timestamp·내용이 그대로면 그 뜻을 적는다) | 내용 판정 중 (timestamp 못 씀) | update_rate_hz, sample_age_ms, 중복/역행/무효, RTT p50/p99, sample_gate, spec_refresh_pending |
<!-- 26.09.25 _ YSH _ command 행에 명령 트리거·타이머 통계 추가. 원문 행:
| `<노드>: command`       | 중복 writer (송신 차단)                 | stale 송신·송신 실패·NaN 무시·스펙 재조회 대기 | sends/ok/fail per s, stale, NaN, RTT p50/p99, 모드·조회 나이, 가드 차단 감속 |
-->
| `<노드>: command`       | 중복 writer (송신 차단)                 | stale 송신·송신 실패·NaN 무시·스펙 재조회 대기·timer 지연재설정 > 송신의 10 % | sends/ok/fail per s, stale, NaN, RTT p50/p99, 모드·조회 나이, 가드 차단 감속, `command_trigger`·`cmd_resend_hz`·`sends_per_update`·`send_gap_p50/p99/max_ms`·`cmd_timer_late_window/total` |
| `<노드>: clock`         | \|MORAI − 수신\| > 1 h (epoch 가 아닌 시각) | \|MORAI − 수신\| 중앙값 > 20 ms / 내용 판정 | offset 중앙값·p01·p99·min·max, stamp_source |
| `<노드>: ground_truth`  | 다른 kinematic_state writer / 표본 낡음 |                                        | actor 수·표본 나이, 측위 초기화, 텔레포트 성공/실패      |

`<노드>` 는 노드 이름이다 (GT launch 기본 `morai_bridge`).
<!-- 26.09.25 _ YSH _ 리뷰 반영: 진단 이름 앞머리는 노드 이름 (원문 표기 `morai_interface: ...`) -->

엔진은 10 초마다 main.py 재송신기와 같은 항목의 통계 줄(`제어 재송신 10.0s: 송신 … 응답성공 … stale …`)과
`MORAI 갱신 10.0s: 새 표본 … (Hz) 중복 … 간격>45ms … 스냅샷 RTT … MORAI−수신 …` 줄을 INFO 로 남긴다.
<!-- 26.09.25 _ YSH _ 명령 트리거 --> `제어 재송신` 줄은 main.py 와 같은 순서 그대로이고, `목표` 가 `update` 면
`MORAI 업데이트당 1회`, `timer` 면 main.py 처럼 `100 Hz` 다. `지연재설정` 은 timer 의 late 수(update 는 0), 끝에
`명령 트리거 <update|timer> (업데이트당 N)` 가 붙는다.

<!-- 26.09.25 _ YSH _ 신규 절 — main.py 와의 차이 -->

## main.py 와 다른 점

| 항목                 | main.py                                            | 이 노드                                                        |
| -------------------- | -------------------------------------------------- | -------------------------------------------------------------- |
<!-- 26.09.25 _ YSH _ timer 트리거 추가. 원문 행:
| 명령 송신            | 100 Hz 재송신 스레드 (원격이면 비동기)             | **MORAI 업데이트당 1 회**, 업데이트가 트리거 (WaitForTick)     |
-->
| 명령 송신            | 100 Hz 재송신 스레드 (원격이면 비동기)             | GT yaml 기본 `command_trigger: timer` — main.py 처럼 `cmd_resend_hz`(100) 고정 주기 동기 송신 (비동기 모드는 없다). `update` 면 MORAI 업데이트당 1 회 (WaitForTick 트리거) <!-- 26.09.25 _ YSH _ GT 기본 timer 로 변경 --> |
| 상태 조회            | Ego 200 Hz + 액터 30 Hz 폴링 (따로)                | 업데이트당 GetAllActorsState 1 회 (Ego·액터·timestamp 한 프레임) |
| stamp                | 발행 시각 (이 PC)                                  | MORAI 샘플 시각 (`time_source`)                                |
| 무효·중복 표본       | 그대로 발행                                        | 발행·송신 안 함, 진단                                          |
| NaN `control_cmd`    | 그대로 계산 (NaN 조향 → +max 로 좌측 끝)           | 버리고 마지막 유효 명령 유지 (나이가 차면 stale 감속)          |
| 기어 성공 판정       | 예외 없음                                          | Result + 되읽기                                                |
| 제어 모드 요청       | 없음 (MORAI UI 에서 수동)                          | `/control/control_mode_request` (되읽기 확인)                  |
| SYNCHRONOUS 모드     | NOT_READY                                          | AUTONOMOUS                                                     |
| 램프                 | MORAI 로 안 보냄 (UDP 폐지)                        | SetVehicleTailLight                                            |
| 텔레포트 확인        | Result 만                                          | 다음 스냅샷 위치 (0.5 m)                                       |
| 연결 끊김            | 벤더 래퍼가 조용히 재연결                          | 재연결 + 스펙 재조회 + AUTO 재지정 + 진단                      |
| 종료                 | —                                                  | ~0.3 s 감속 대체 명령                                          |
| 중복 실행            | —                                                  | 중복 writer 가드 (~0.3 s 감속 뒤 송신 차단 + ERROR, `duplicate_writer_guard`) |
| 노드 이름            | `/morai_bridge`                                    | 같다 (GT launch 기본). `ros2 run` 이면 `-r __node:=morai_bridge` |
| 제어 모드 조회 실패  | 보고 즉시 중단 (None)                              | 0.5 s 넘게 실패하면 보고 중단                                  |
| Ego 1 s 넘게 없음    | 계속 폴링                                          | 계속 읽고, 돌아오면 스펙 재조회 (그때까지 명령 보류)           |
| ControlVehicle deadline | max(`cmd_rpc_timeout_s`, 4×왕복), 상한 1 s (기동 때 1 회 측정) | 같은 식, 스냅샷 왕복 p50 으로 계속 갱신 (`morai.command_deadline_ms` 가 하한) |
<!-- 26.09.25 _ YSH _ 리뷰 반영: 아래 행들은 파라미터 이식(rig.sh 인자) 때 알아야 할 차이 -->
| `long_cmd_mode` 잘못된 값 (런타임) | WARN 후 acceleration 으로 동작          | `param set` 거부 (기동 때 잘못된 값은 WARN 후 acceleration)    |
| `cmd_stale_timeout_s` ≤ 0 | max(x, 0) — 늘 stale                          | 0.2 s 로 바꾸고 WARN                                           |
| `steer_comp_*` 에 NaN | 비교를 통과해 표에 들어간다                        | 표를 잘못된 것으로 보고 배율 1.0 + WARN                        |
| `cmd_stale_accel` 정리 | 재송신기가 켜졌을 때만 음수 강제                   | 늘 음수 강제 (아니면 −2.4)                                    |
<!-- 26.09.25 _ YSH _ cmd_resend_hz 는 이제 있다 (command_trigger=timer). 원문 행:
| `cmd_resend_hz`·`cmd_rpc_timeout_s`·`cmd_send_mode`·`gear_async`·`gear_rpc_timeout_s`·`ego_spec_wait_s` | 있음 | **없다** — 넘기면 기동 WARN. 대응: 업데이트당 1 회 송신·`morai.command_deadline_ms`·항상 동기·항상 기어 스레드·`morai.gear_deadline_ms`·무한 대기 |
-->
| `cmd_rpc_timeout_s`·`cmd_send_mode`·`gear_async`·`gear_rpc_timeout_s`·`ego_spec_wait_s` | 있음 | **없다** — 넘기면 기동 WARN. 대응: `morai.command_deadline_ms`·항상 동기·항상 기어 스레드·`morai.gear_deadline_ms`·무한 대기 |
| `cmd_resend_hz`      | 재송신 주기 (기본 100)                             | 같은 이름·기본값. `command_trigger=timer` 일 때만 쓴다 (런타임 변경 가능, 10~400) |

<!-- 26.09.25 _ YSH _ 기동 순서 교체 -->

## 기동 순서

노드는 **생성자에서 블로킹하지 않는다.** 생성자는 파라미터·퍼블리셔·스레드만 만들고, 연결은 엔진 스레드가 한다.

<!-- 26.09.25 _ YSH _ UNAVAILABLE 로 끊긴 뒤 첫 시도에서도 채널 재생성. 원문:
1. 연결 확인 (`wait_connected` 500 ms, 실패하면 0.2 s 뒤 재시도, 5 번마다 채널 재생성)
-->
1. 연결 확인 (`wait_connected` 500 ms, 실패하면 0.2 s 뒤 재시도, 5 번마다 채널 재생성 — 끊겨서 돌아온 첫 시도에서도)
2. `GetVehicleSpec` — Ego 가 없으면(시나리오 교체 중) 0.75 s ~ 5 s 백오프로 **죽지 않고** 기다린다
3. 제어 모드 조회 → (`engage_on_start` / 재연결 AUTO 재지정)
4. 램프 재전송, (센서킷이 지정됐으면) 센서 장착
5. 업데이트 루프

<!-- 26.09.26 _ YSH _ 램프도 업데이트마다 발행으로 옮겼다. 원문:
MORAI 연결 전에도 `/vehicle/status/*` 퍼블리셔는 생기지만 **아무것도 발행되지 않는다** (램프 에코만 10 Hz).
-->
MORAI 연결 전에도 `/vehicle/status/*` 퍼블리셔는 생기지만 **아무것도 발행되지 않는다** (램프 리포트도 — CARLA 와 같다).
스펙(`max_steering_angle`)을 모르는 동안은 ControlVehicle 을 보내지 않는다.

<!-- 26.09.25 _ YSH _ 원문
1. 생성자 — 파라미터, gRPC 클라이언트, 콜백 그룹, 재시도 타이머만
2. 재시도 타이머 — `wait_for_connected` → `GetVehicleSpec`
3. 성공 시 — degree→radian 변환, `StatusReporter` 와 `CommandForwarder` 생성, 타이머 취소

따라서 **MORAI 연결 전에는 토픽이 하나도 안 생긴다.** 정상이다. 명령 경로가
`max_steering_angle` 로 나누므로, 그 값을 모르는 채 전달하면 쓰레기 조향이 나간다.
-->

## 검증

### 시뮬 없이

<!-- 26.09.25 _ YSH _ 136 → 140 (가짜 MORAI 종단 시험에서 찾은 폴백·재연결 결함 회귀 시험 4 개). 원문 개수 136 -->
<!-- 26.09.25 _ YSH _ 140 → 156 (command_trigger=timer 16 개: 업데이트율과 무관한 고정 주기·엔진 루프 무송신·연결/스펙/첫 표본 전
     무송신·링크 끊김에 헛돌지 않음·stale·NaN·정체 뒤 몰아 보내지 않음·forward·가드·런타임 전환·조향 보정 속도·종료 감속 주기·
     느린 왕복·실제 스레드). 원문: `colcon test --packages-select autoware_morai_interface` — gtest 140 개 (네트워크 없음): -->
`colcon test --packages-select autoware_morai_interface` — gtest 156 개 (네트워크 없음):
변환 골든(각속도는 `morai_coordinates.py`, map pose·역변환·조향 보정은 main.py 식을 python3 로 돌린 값,
uuid5 는 Python 과 바이트 비교), 엔진 루프(가짜 링크·가짜 시계: 업데이트당 1 회 송신, 중복·무효 건너뜀, stale,
NaN, forward 차단, 폴백, 재연결·스펙 재조회, AUTO 재지정, 발행 주기, stamp, 시계 오프셋, 종료 감속),
기어 송신기(main.py `_GearSender` 의미), 표본 판정, 조향 유지 필터.
<!-- 26.09.25 _ YSH _ 리뷰 반영 시험 추가 --> 리뷰 반영분: timestamp 0·멈춤 → 내용 판정(수신 stamp, 정지한 차도 송신,
일시정지는 무발행), 역행 3 표본/0.5 s 새 기준, 막지 않는 WaitForTick → 폴백(헛돌기 없음), 폴백 재확인(짧은 deadline,
실제로 기다려야 복귀), 낡은 모드 보고 생략, 재연결 뒤 첫 조회로 AUTO 재지정, Ego 복귀 스펙 재조회(재연결 아님),
왕복 맞춤 deadline(4×RTT, 상한 1 s), 가드 차단 순간 감속, 가드 자기 제외(같은 이름 두 번째 인스턴스).

<!-- 26.09.25 _ YSH _ 원문: `colcon test --packages-select autoware_morai_interface` 로 37개 변환 테스트 실행. -->

가짜 MORAI(gRPC, 로컬 고포트)로 노드 전체를 돌려 확인한 것 (2026-09-25): 업데이트당 ControlVehicle 분포
{0: 0, 1: 250, 2+: 0} / 5 s, 발행 주기(위 표), control_mode_request(Result 가 거짓이어도 되읽기로 성공),
기어 D, stale 감속, NaN 무시, forward 차단·재개 때 램프 재전송, 중복 writer 가드(차단→재개), 텔레포트
확인(오차 0.00 m), MORAI 재기동 뒤 ~2 s 안에 재연결·스펙 재조회·AUTO 재지정, 150 ms 정체에도 폴백 없음,
SIGINT 때 감속 15 회 뒤 정상 종료, CPU ~1 %.
<!-- 26.09.25 _ YSH _ 리뷰 반영 뒤 재확인 --> 리뷰 반영 뒤 다시 (같은 날): 업데이트당 송신 {0: 0, 1: 250, 2+: 0} / 5 s,
velocity 50.0 Hz, 가드 첫 판정(1 s) 뒤 송신 시작, control_mode_request·기어 D·stale·SIGINT 감속 15 회. domain 91 에서
같은 이름(`/morai_bridge`) 두 인스턴스: 혼자면 가드 조용, 둘이면 둘 다 ERROR·차단, 하나가 나가면 재개.

<!-- 26.09.25 _ YSH _ 신규 — 가짜 MORAI 종단 시험 (스크립트화, 다시 돌릴 수 있다) -->
### 가짜 MORAI 종단 시험 (`test/e2e/`, 수동 실행)

`colcon test` 에는 들어가지 않는다 (DDS·고포트 gRPC·약 1 분). 빌드한 뒤:

```bash
src/universe/autoware_universe/simulator/autoware_morai_interface/test/e2e/run_fake_morai_e2e.sh /tmp/e2e_out
# → 단계별 [PASS]/[FAIL] 과 /tmp/e2e_out/{evidence.json, node.log, fake.log, fake_events.jsonl}
```

깨끗한 셸(`env -i`, `ROS_DOMAIN_ID=91`, `ROS_LOCALHOST_ONLY=1`)에서 `morai_grpc_vendor` 의 가짜 서버
(`build/morai_grpc_vendor/fake_morai_server`, 설치 안 함, 7789 거부)를 127.0.0.1:17871 에 띄우고, 노드 실행 파일을
GT param yaml + `-r __node:=morai_bridge` 로 띄운다. 스크립트가 Autoware 역할(control_cmd 33 Hz, gear_cmd·turn 10 Hz,
`control_mode_request`, `/initialpose`, `/localization/initialize` 서버, 파라미터 설정)을 하며 확인하는 것:
MORAI 없이 기동 → 나중에 연결, 측위 DIRECT 1 회, AUTO(가짜 Result 는 거짓 실패 — 되읽기로 성공)·기어 D·램프,
업데이트당 ControlVehicle 1 회(가짜가 업데이트 사이 수를 센다)·업데이트→명령 지연, 가속 1.0 이 가짜 Ego 속도 기울기로,
조향 보정·리포트·heading_rate, 발행 주기, 네 토픽 stamp 가 가짜 timestamp 와 같음, TrackedObjects uuid5,
NaN 조향 무시, stale 감속(0.2 s 뒤 첫 업데이트), 1 s 멈춤(발행 중단·재개, 트리거 유지), 서버 2 s 끊김 + KEYBOARD
리셋(재연결·AUTO 재지정), 전 필드 0 Ego(무발행·스펙 재조회), 텔레포트 역변환, forward 토글, SIGINT 감속, CPU.

2026-09-25 결과 (세 번 연속 20/20 PASS): 업데이트 49.5/s 에 ControlVehicle 495/495 (업데이트당 2 개 이상 0),
업데이트→명령 p50 0.25 ms · max ≤ 0.6 ms, velocity·steering·kinematic_state·TF 49.5 Hz(같은 stamp 495 개 전부 가짜
timestamp), gear·mode 24.7 Hz, objects 9.9 Hz, 램프 10 Hz, stale 첫 감속 0.20~0.22 s(0.2 s 뒤 첫 업데이트),
멈춤 뒤 재개 +1 ms, 서버 재기동 뒤 재개 0.8 s, 노드 CPU 1.8~2.0 % (한 코어), GT launch 파일로도 `/morai_bridge` 49.5 Hz·정상 종료.

<!-- 26.09.25 _ YSH _ 명령 트리거 A/B 종단 시험 -->
명령 트리거는 `--command-trigger update|timer [--cmd-resend-hz 100]` 로 고른다 (기본 update). timer 에서는 업데이트당 판정이
고정 주기 판정으로 바뀐다 (1: 목표 ±3 %·간격 중앙값 = 주기·5 주기 창 최대 6 개, 2d: 0.2 s 뒤 첫 틱, 4: 멈춤 중에도 main.py 처럼
계속 송신, 6: Ego 무효 첫 1 s 는 송신·그 뒤 보류, F: 켠 뒤 ~hz × 0.8). 두 모드 모두 `AB`(command 진단이 트리거·업데이트당
송신·간격·지연재설정을 싣는지)를 더해 21 항목.

2026-09-25 A/B (가짜 MORAI 50 업데이트/s, 한 PC, 부하 평균 ~30):

| | update | timer 100 Hz |
| --- | --- | --- |
| 판정 | 21/21 PASS | 21/21 PASS |
| MORAI 업데이트 | 49.45 /s | 49.35 /s |
| ControlVehicle | 49.45 /s, 업데이트당 1.000 (분포 {1: 495}) | 100.00 /s, 업데이트당 2.026 (분포 {2: 488, 3+: 6} — 3+ 는 가짜의 2 프레임 건너뜀) |
| 송신 간격 p50 / p99 / max | 20.0 / 40.0 / 40.0 ms | 10.00 / 10.08 / 10.18 ms (min 9.88, 지연재설정 0) |
| stale 첫 감속 | 0.208 s (0.2 s 뒤 첫 업데이트) | 0.204 s (0.2 s 뒤 첫 틱) |
| 1 s 멈춤 중 ControlVehicle | 0 | 99 (main.py 처럼 계속) |
| SIGINT 감속 | 15 회 | 30 회 |
| 노드 CPU (한 코어) | 2.1 % | 2.8 % |

### 시뮬 연결 후 — 조향 부호 검증

**engage 전에 4단계를 순서대로 전부 수행할 것.** 사고가 재발하는 경로다.

1. **정지 상태 리포트 부호.** 기어 D, `forward_control_to_sim:=false`, 키보드로 **좌측 풀락**
   → `steering_status.steering_tire_angle` 이 **양수**. 음수면 중단.
2. **IMU 교차검증.** 2 m/s 이상 좌선회 → `/sensing/imu/imu_data` 의 `angular_velocity.z` 와
   `steering_tire_angle` 이 **동시에 양수**. 좌우 각 15표본, **15/15 일치 필요** (과반 아님).
   <!-- 26.09.25 _ YSH _ GT 모드는 IMU 가 없으니 velocity_status.heading_rate 로 대신한다 -->
   GT 모드(센서 없음)에서는 `/vehicle/status/velocity_status` 의 `heading_rate` 로 대신한다.
3. **자전거 모델 크기.**
   `|steering_tire_angle| ≈ atan(wheel_base × heading_rate / longitudinal_velocity)`,
   비율 **0.8~1.2**. 57 근처면 deg/rad 오류.
4. **개루프 명령 부호 — 사고 직결 검사.** 컨트롤러 끄고 정지 상태에서

   ```bash
   ros2 topic pub -r 10 /control/command/control_cmd autoware_control_msgs/msg/Control \
     '{lateral: {steering_tire_angle: 0.2}, longitudinal: {acceleration: 0.0}}'
   ```

   시뮬 바퀴가 **좌측**으로, `steering_status` 가 **+0.2 로 수렴**.
   −0.2 로 수렴하면 `steer_cmd_sign` 이 틀린 것 — 중단.

그 외: 속도(36 km/h → `longitudinal_velocity ≈ 10.0`), 기어(P→R→N→D → 22→20→1→2),
제어모드(키보드↔자동 → 4↔1), 램프(`command: 2` → 좌측 점등 + report 에코),
`max_steer_angle` (부팅 로그의 값과 `morai_vehicle_description/config/vehicle_info.param.yaml` 대조).
<!-- 26.09.25 _ YSH _ 추가 --> 그리고 `/diagnostics` 의 `morai_bridge: command` 에서 sends_per_s ≈ update_rate_hz,
`send_fail_per_s = 0` 인지, `morai_bridge: update` 의 update_rate_hz 가 ~50 인지 본다.
<!-- 26.09.25 _ YSH _ timer 트리거 --> `command_trigger=timer` 면 sends_per_s ≈ `cmd_resend_hz`, sends_per_update ≈ `cmd_resend_hz` / update_rate_hz
(100 Hz·50 Hz 면 ≈ 2), `cmd_timer_late_window` 가 0 근처인지 본다.
