# morai_grpc_vendor

MORAI SIM 의 gRPC 인터페이스를 격리하는 벤더 패키지. `autoware_morai_interface` 가
이 패키지 하나만 의존하도록 해서, MORAI 종속물이 Autoware 쪽 코드로 새지 않게 한다.

담고 있는 것.

<!-- 26.09.24 _ YSH _ closure 범위 15 -> 31개 (Actor·Sensor·Simulation·Simulator), morai_probe 항목 추가, 누락돼 있던 SensorClient 항목 추가 (목록이 끊기지 않게 이 주석 하나로 표기)
- `proto/` — MORAI `Actor` 서비스의 transitive import closure 15개. **upstream 그대로, 수정 금지.**
-->
- `proto/` — MORAI `Actor`·`Sensor`·`Simulation`·`Simulator` 서비스의 transitive import closure 31개. **upstream 그대로, 수정 금지.**
- 빌드 시점에 생성되는 protobuf / gRPC C++ 코드 (커밋하지 않음)
- `ActorClient` — `Actor` 서비스에 대한 얇은 C++ 래퍼
<!-- 26.09.25 _ YSH _ 엔진용 링크 계층 MoraiLink 항목 추가 -->
- `MoraiLink` — C++ 엔진용 링크 계층. 상태·명령 두 채널, 공개 헤더에 grpc·protobuf 없음 (아래 절 참조)
- `SensorClient` — `Sensor` 서비스에 대한 얇은 C++ 래퍼
- `morai_probe` — MORAI 시간 동기 특성을 재는 **읽기 전용** CLI (`tools/`, 아래 절 참조)
<!-- 26.09.25 _ YSH _ monitor 서브커맨드 한 줄 -->
  - `morai_probe monitor` — 다른 무엇이 모는 동안 MORAI 가 실제로 적용한 값을 지켜보는 관찰 모드 (아래 절)
<!-- 26.09.25 _ YSH _ 구동 측정 도구 morai_actuate 항목 추가 -->
- `morai_actuate` — ControlVehicle 이 업데이트당/물리 스텝당 소비되는지 재는 CLI. **Ego 를 움직인다** (아래 절 참조)
<!-- 26.09.25 _ YSH _ lockstep 모드와 동기 모드 비상 복구 도구 항목 추가 -->
  - `--lockstep` — MORAI 동기 모드에서 도구가 `Tick` 으로 MORAI 를 페이싱하는 모드 (아래 "lockstep mode")
- `morai_sync_restore` — lockstep 클라이언트가 죽어 동기 모드로 **얼어 버린 MORAI 를 실시간으로 되돌리는** 비상 복구 CLI (아래 절 참조)

## Prerequisites

**이 패키지는 `rosdep install` 만으로는 빌드되지 않는다.** gRPC C++ 런타임과 코드 생성
플러그인에 rosdep 키가 없기 때문이다. 한 번만 설치하면 된다.

```bash
sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc
```

`libgrpc++-dev` 를 `package.xml` 에 **일부러 넣지 않았다.** 넣으면
`rosdep install --from-paths src --ignore-src -y` 가
`Cannot locate rosdep definition for [libgrpc++-dev]` 로 죽는데, 이건 이 패키지 하나가
아니라 **워크스페이스 전체**의 의존성 해결을 막는다. 대신 CMake 가 설정 단계에서
복사 가능한 apt 명령과 함께 명확히 실패한다.

`rosdep` 자족성이 필요한 사이트는 동봉된 로컬 룰을 opt-in 할 수 있다.

```bash
sudo sh -c 'echo "yaml file://$PWD/src/universe/autoware_universe/simulator/morai_grpc_vendor/rosdep/grpc.yaml" \
  > /etc/ros/rosdep/sources.list.d/50-morai.list'
rosdep update
```

그 다음 `package.xml` 에 `<depend>libgrpc++-dev</depend>` 를 추가하면 된다.

## proto/ 를 건드리지 말 것

<!-- 26.09.24 _ YSH _ 파일 수 15 -> 31
`proto/` 아래 15개 파일은 MORAI 사설 저장소에서 **바이트 그대로** 가져온 것이고
-->
`proto/` 아래 31개 파일은 MORAI 사설 저장소에서 **바이트 그대로** 가져온 것이고
**라이선스 파일이 없다**. 출처·SHA·라이선스 상태는 [PROVENANCE.md](PROVENANCE.md) 참조.

<!-- 26.09.24 _ YSH _ 파일 수 15 -> 31 (다음 문단 "15개 파일을" -> "31개 파일을")
리포지토리 루트 `.pre-commit-config.yaml` 이 이 디렉토리를 제외한다. 제외가 없으면
`end-of-file-fixer` 가 15개 파일을 조용히 재작성한다 (일부 파일이 의도적으로 개행 없이
-->
리포지토리 루트 `.pre-commit-config.yaml` 이 이 디렉토리를 제외한다. 제외가 없으면
`end-of-file-fixer` 가 31개 파일을 조용히 재작성한다 (일부 파일이 의도적으로 개행 없이
끝난다). 라이선스가 불명확한 트리에서는 **"바이트를 안 건드렸다, upstream SHA 는 이것"**
이 훨씬 방어 가능한 입지다.

업스트림 갱신 절차도 PROVENANCE.md 에 있다.

## 왜 `autoware_package()` 를 호출하지 않나

`autoware_package()` 는 `-Wall -Wextra -Wpedantic -Werror` 를 `add_compile_options()` 로
주입하는데, 이건 **디렉토리 스코프**라 생성된 서드파티 코드에도 그대로 걸린다.
호출하지 않는 것 자체가 1차 방어선이다. 스타일 문제가 아니라 동작상 필수 결정이다.

3중 방어.

1. 이 패키지가 `autoware_package()` 를 부르지 않음 (주 메커니즘)
2. `set_source_files_properties(... COMPILE_OPTIONS "-w")` — 소스 레벨 옵션이 마지막에 붙어서 이김
3. 생성 헤더를 `SYSTEM PUBLIC` 으로 export → 소비자 쪽에 `-isystem` 으로 들어가 경고가 안 남

## 격리 경계

`ActorClient` 는 PIMPL 이라 공개 헤더에 `<grpcpp/...>` 가 나타나지 않고, 생성된
`*.grpc.pb.h` 는 **설치하지 않는다**. gRPC 는 `PRIVATE` 로만 링크한다.

결과적으로 `autoware_morai_interface` 의 `package.xml` 은 모든 키가 rosdep 으로 해결되고,
rosdep 키 부재 문제가 이 패키지 밖으로 전파되지 않는다.

검증.

```bash
find install/morai_grpc_vendor -name '*.grpc.pb.h'   # 빈 결과여야 함
```

<!-- 26.09.25 _ YSH _ MoraiLink 격리 — ActorClient 보다 한 단계 더 (protobuf 도 공개 헤더에 없다) -->
`MoraiLink` 는 한 단계 더 격리한다. 공개 헤더(`morai_link.hpp`, `morai_link_types.hpp`)에는
protobuf 도 없다 — 값은 전부 평범한 구조체·열거형 사본이고, proto 와의 변환(`src/morai_link_convert.*`)은
설치하지 않는다. `test/test_morai_link_isolation.cpp` 가 공개 헤더를 가장 먼저 include 한 뒤
protobuf·gRPC 가드 매크로가 정의돼 있으면 컴파일을 깨고, 헤더의 `#include` 도 스캔한다.
열거형 사본 값은 `morai_link.cpp` 의 `static_assert` 가 proto 와 대조한다.

## 생성되는 심볼

C++ 네임스페이스는 `.proto` 의 `package` 선언을 따른다.

<!-- 26.09.24 _ YSH _ simulation / simulator 행 추가. upstream 패키지명이 "simualtor"(오타) 라 C++ 네임스페이스도 그대로 따라간다 -->
<!-- 26.09.24 _ YSH _ common 행 정정 — common/*.proto 6개는 `package morai_sim_api;` 라 네임스페이스도 morai_sim_api (morai_probe 가 morai_sim_api::Empty·ObjectInfo 사용). 31개에 포함된 sensor·infrastructure·map 행 추가. 원래 행:
| `morai_sim_api.common` | `morai_sim_api::common` |
-->
| proto package | C++ namespace |
|---|---|
| `morai_sim_api.actor` | `morai_sim_api::actor` |
| `morai_sim_api` (`common/*`) | `morai_sim_api` |
| `morai_sim_api.infrastructure` | `morai_sim_api::infrastructure` |
| `morai_sim_api.map` | `morai_sim_api::map` |
| `morai_sim_api.sensor` | `morai_sim_api::sensor` |
| `morai_sim_api.simulation` | `morai_sim_api::simulation` |
| `morai_sim_api.simualtor` (sic) | `morai_sim_api::simualtor` |

생성 헤더는 proto 트리 구조를 미러링하므로 include 경로도 그대로다.

```cpp
#include <morai/actor/actor_control.pb.h>   // VehicleCtrlCmd
#include <morai/actor/actor_enum.pb.h>      // GearMode, VehicleControlMode, VehicleTurnSignal
#include <morai/common/result.pb.h>         // Result, StatusCode
```

<!-- 26.09.24 _ YSH _ 신규 절 — Stage 1 읽기 전용 측정 도구 morai_probe 사용법 -->
<!-- 26.09.24 _ YSH _ 리뷰 반영 — 깨끗한 셸 실행법, 기본 측정 창 60 s, U1 신뢰성 판정·주기 기준 정체 지표, raw 는 변화 시에만 저장, Hz 비교 방법 -->
## morai_probe — 읽기 전용 측정 도구

엔진 설계 전에 MORAI 가 시간 동기에 실제로 무엇을 주는지 잰다. 프레임 주기를 **하드코딩하지
않고 MORAI 에서 읽기** 위한 근거 수집용이다.

**읽기 전용.** `Get*` / `WaitForTick` / `GetTimestamp` / `GetSimulatorVersion` 만 호출한다.
`test/test_probe_readonly.cpp` 가 소스를 스캔해서 허용 목록 밖의 RPC(`Set*`, `ControlVehicle`,
`Tick`, `Start/Stop/Pause/Resume`, `SetSynchronousMode`, spawn/destroy …)가 보이면 실패한다.

### 실행 환경

**이 워크스페이스의 `install/setup.bash` 가 마지막으로 source 된 셸에서 실행한다.** 다른
오버레이(예: 예전 테스트 워크스페이스)가 `AMENT_PREFIX_PATH` / `LD_LIBRARY_PATH` 에 남아 있으면
`ros2 run morai_grpc_vendor morai_probe` 가 그쪽 패키지를 찾아 실행 파일이 없다고 하거나,
예전 `libmorai_grpc_vendor.so`(Simulation/Simulator 스텁 없음)를 불러 `undefined symbol` 로
죽는다. 가장 확실한 것은 다른 환경을 끊은 깨끗한 셸이다. `env -i` 는 `MORAI_SIM_ADDRESS` 도
지우므로 주소는 `--host` 로 준다.

```bash
# 워크스페이스 루트에서 — 전부, 약 3 분 (rtt 60 s + frames 60 s + simtime 60 s + waitfortick)
env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin TERM=xterm LANG=C.UTF-8 bash --noprofile --norc -c \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 run morai_grpc_vendor morai_probe --host 10.10.10.1 all --json /tmp/probe.json'

# install/setup.bash 를 마지막으로 source 한 셸이면 — 하나만
ros2 run morai_grpc_vendor morai_probe --host 10.10.10.1 frames --seconds 120
```

설치된 바이너리(`install/morai_grpc_vendor/lib/morai_grpc_vendor/morai_probe`)는 DT_RPATH 로
자기 `libmorai_grpc_vendor.so` 를 `LD_LIBRARY_PATH` 보다 먼저 찾으므로 직접 실행해도 된다.

### 서브커맨드

| 서브커맨드    | RPC                                                          | 답하려는 것                                                                                                          |
| ------------- | ------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------- |
| `sync`        | Simulation.GetSynchronousMode                                | 동기 모드 종류, `tick_period` (proto: msec, 1~100)                                                                   |
| `spec`        | Actor.GetVehicleSpec(ego)                                    | 제원, 최대 조향각 (deg / rad)                                                                                        |
| `mode`        | Actor.GetVehicleControlMode(ego)                             | 제어 모드                                                                                                            |
| `rtt`         | Actor.GetActorState(ego), `--rtt-seconds` 동안 `--rate` 간격 | 정상 호출 p50/p90/p99/max 와 타임아웃 포함 전체 호출 분포, 분당 >20 ms·>45 ms 건수                                   |
| `frames`      | Actor.GetAllActorsState 루프 `--seconds`                     | timestamp 단위·단조성·해상도, sim step·벽시계 프레임 Hz(신뢰성 판정 포함), 주기 기준 정체, ego 포함 여부, tick_count |
| `simtime`     | Simulator.GetTimestamp 루프 `--seconds`                      | 단위 추정, sim/wall 비율, ActorStates.timestamp 와의 관계                                                            |
| `waitfortick` | Simulation.WaitForTick ≤ `--wft-n` 회                        | 실시간 모드에서 프레임마다 돌아오는지 (항상 deadline 적용, 절대 안 멈춤)                                             |
| `all`         | 위 전부 (sync→spec→mode→rtt→frames→simtime→waitfortick)      | 마지막에 U1/U4/U17/U7/Uw/Ut/U5/U12 요약                                                                              |
| `monitor`     | GetAllActorsState 폴링(또는 WaitForTick 뒤), 0.5 s 마다 GetVehicleControlMode | 다른 무엇이 모는 동안 MORAI 가 실제로 적용한 값 — 아래 "monitor" 절 (`all` 에 포함되지 않음) |

옵션: `--host` (기본 `$MORAI_SIM_ADDRESS`, 없으면 127.0.0.1), `--port` (기본 `$MORAI_SIM_PORT`,
없으면 7789), `--ego` (기본 `Ego`, main.py 와 동일), `--seconds` (frames/simtime/waitfortick 창,
기본 60), `--rtt-seconds` (기본 60), `--n` (rtt 호출 수 상한, 기본 없음), `--rate` (rtt 호출 간격 Hz,
기본 200 = main.py 폴링, 0 = 연속), `--poll-hz` (frames/simtime, 기본 0 = tight loop), `--wft-n` (200),
`--deadline-ms` (200), `--connect-timeout-ms` (3000), `--max-raw-rows` (200000), `--json PATH`,
`--no-raw`. 초는 0 초과 3600 이하, Hz 는 0 또는 0.1~100000, 개수는 1 이상의 정수만 받는다
(`-1`, `nan`, `inf`, `12abc` 는 종료 코드 1). 연결 실패 시 종료 코드 2.
<!-- 26.09.25 _ YSH _ monitor 전용 옵션 -->
`monitor` 만 `--seconds 0`(Ctrl-C 까지, 기본)과 최대 86400 초, `--csv PATH`, `--trigger poll|wft` 를 받는다
(다른 서브커맨드에 주면 종료 코드 1). monitor 의 `--poll-hz` 기본은 1000. `--csv` 쓰기 실패는 종료 코드 3.
`--json`·`--csv` 경로는 **연결 전에** 확인한다 — 없는 부모 디렉터리는 만들고, 쓸 수 없으면 측정 없이 종료 코드 3.

<!-- 26.09.25 _ YSH _ 신규 절 — monitor 서브커맨드 (Autoware 가 C++ 노드로 MORAI 를 모는 동안 관찰) -->
### monitor — 주행 중 MORAI 가 실제로 적용한 값 관찰

Autoware(GT 모드)가 C++ 노드로 MORAI 를 모는 동안 **옆에서 읽기만 하며** 진동·0 떨어짐·모드 뒤집힘을
잡는다. 서로 다른 업데이트(ActorStates.timestamp 변화)마다 한 행: MORAI timestamp·수신 시각·속도
(velocity.x km/h ÷ 3.6, main.py 와 같음)·Ego vehicle_state(throttle, brake, steer, front_wheel_angle,
gear)·충돌 수. 0.5 s 마다 Ego 제어 모드. 시계 오프셋은 따로 묻지 않고 스냅샷 timestamp 로 본다(아래 `clk>=`).

```bash
# 워크스페이스 루트, 깨끗한 셸 — Ctrl-C 로 끝낸다 (끝나면 CSV·JSON 을 마저 쓴다)
env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin TERM=xterm LANG=C.UTF-8 bash --noprofile --norc -c \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ts=$(date +%Y%m%d-%H%M%S) && mkdir -p ~/Data/cpp_iface/monitor && \
   ros2 run morai_grpc_vendor morai_probe --host 127.0.0.1 monitor \
     --csv ~/Data/cpp_iface/monitor/$ts.csv --json ~/Data/cpp_iface/monitor/$ts.json'
```

- 기본은 **Ctrl-C 까지**(`--seconds 0`, 최대 86400) 와 **1000 Hz 폴링**(`--poll-hz`) 이다. 업데이트가
  ~20 ms 간격이라 더 낮은 Hz 로도 업데이트는 다 잡지만, 1 ms 폴링이라야 `clk>=` 하한이 1 ms 안으로 빠듯하다.
- **엔진이 도는 동안에는 기본 `--trigger poll` 을 쓴다.** `--trigger wft` 는 WaitForTick 이 돌아올 때마다
  한 번만 읽지만(`stale` = WFT 는 돌아왔는데 스냅샷이 그대로였던 수), 엔진도 WaitForTick 을 프레임 트리거로
  쓴다. 대기자가 둘일 때 MORAI 가 둘 다에게 매 업데이트를 주는지는 실측 전이라, 관찰자가 엔진의 업데이트당
  송신을 흔들 수 있다(도구도 시작할 때 경고한다).
- **관찰자 부하.** 1000 Hz 폴링은 엔진 옆에서 GetAllActorsState 를 초당 ~1000 번 더 부른다. 이전 실측
  (Ego 1 대, 정지, 엔진 없음)에서는 tight loop 초당 ~15000 번에도 업데이트 ~50/s·>45 ms 정체 분당 ~5 로
  변화가 없었지만, NPC 가 많거나 엔진과 함께일 때는 재지 않았다. 의심되면 같은 주행을 `--poll-hz 1000` 과
  `--poll-hz 100`(업데이트는 여전히 다 잡는다) 으로 한 번씩 보고 `upd/s`·`gaps>45ms` 를 비교한다.
  차이가 있으면 낮은 Hz 로 쓴다(`clk>=` 는 폴링 간격만큼 덜 빠듯해진다).
- 사건은 즉시 한 줄: 제어 모드 변화(`AUTO(3) -> KEYBOARD(1)`), 기어 변화, 충돌 시작(상대 이름),
  timestamp 역행(MORAI 재시작), Ego 사라짐/복귀.
- 10 s 마다 한 줄 요약:

| 항목 | 뜻 |
|---|---|
| `upd/s` | 서로 다른 업데이트 수 / s (실측 ~50) |
| `gaps>45ms ts / wall` | MORAI timestamp 간격 / 수신 간격이 45 ms 초과한 수 (최대값 병기). MORAI 내부 정체는 분당 몇 번 정상 |
| `thr 0<->+` | throttle 밟음 ↔ 뗌 전환 수. 히스테리시스: 0.02 초과면 밟음, 1e-3 이하면 뗌, 그 사이는 직전 상태 — 등속 주행의 0 근처 잔떨림은 세지 않는다. 초당 2 회 이상이면 줄 끝에 `OSCILLATION?` — **판정이 아니라 힌트**다 (실측 진동은 업데이트마다 0↔1 로 뒤집혔다. 등속 주행에서의 오탐률은 아직 모른다 — `brake on`·CSV 와 함께 본다) |
| `brake on` | brake 0.01 초과로 들어간 횟수(비율). 유휴 레코드(0.003)는 제외 |
| `idle` | 유휴 레코드(throttle ≤ 1e-3, brake 0.003±0.0015) 수 — 명령이 못 들어간 업데이트 |
| `0-drop steer / fwa` | 조향 중(절댓값 > 1e-3)에 steer / front_wheel_angle 이 정확히 0 으로 떨어졌다가(≤ 5 업데이트) 다시 조향 중으로 돌아온 횟수 |
| `mode ... chg` | 현재 제어 모드와 창 안 변화 수 (ControlVehicle 은 AUTO(3)·SYNCHRONOUS(7) 에서만 먹는다) |
| `gear`, `coll` | 현재 기어·변화 수, 충돌 시작 수 |
| `recv-ts` | 로컬 수신 시각 − MORAI timestamp 의 min / p50 / max. 같은 PC 면 프레임 나이, 원격이면 나이 − (MORAI − 로컬) 시계 차 |
| `clk>=` | `-min(recv-ts)` = MORAI − 로컬 오프셋의 **하한** [ms]. 아래 "시계 오프셋 읽는 법" |

- **시계 오프셋 읽는 법.** MORAI 에는 "호출 시점의 서버 시각" 이 없다. `GetTimestamp` 도 ActorStates.timestamp
  와 같은 **프레임 시각**이라(실측: 폴링의 99.7% 가 같은 값 반복, 초당 ~50 번만 바뀜, 두 값 차 p50 0) NTP 식
  추정은 프레임 나이(p50 ~10 ms)만큼 틀린다. 프레임은 자기 timestamp 뒤에야 읽히므로 `MORAI − 로컬 >= ts − recv`
  이고, 서로 다른 업데이트의 첫 응답이 가장 빠듯하다 → 창마다 `clk>= = max(ts − recv)`. 이 값은 참 오프셋보다
  최소 프레임 나이(MORAI 내부 발행 지연 + 응답 편도 + 폴링 간격)만큼 낮아서 **같은 PC 에서도 0 이 아니라 음수**다.
  1. 로컬 MORAI(참 오프셋 0)에서 잰 `clk>=` 가 **기준선**이다(음수가 정상, 값은 로컬 실측으로 정한다). 창마다
     1 ms 안팎으로 안정적이어야 한다.
  2. 서브 PC 의 MORAI 에 같은 `--poll-hz` 로 붙어 `clk>=` 를 읽는다. **오프셋 ≈ 원격 값 − 기준선**
     (`clock_offset_from_baseline_ms`). 망 편도 지연(LAN 수백 µs)만큼 더 불확실하다.
  3. 두 PC 모두에서 `chronyc tracking`(System time / Last offset)·`chronyc sources -v` 로 교차 확인한다.
- 끝에 전체 합계, 사건 수, `OSCILLATION?` 창 수, 응답 코드 분포. `--csv` 는 진행 중에도 쓰고 10 s 마다
  flush 한다(열: `t_s, recv_unix_s, morai_ts_us, dts_ms, recv_minus_ts_ms, rpc_ms, polls, ego_found,
  ego_valid, speed_mps, accel_x_raw, throttle, brake, steer, front_wheel_angle, gear, control_mode,
  collisions, n_actors, x, y, yaw_deg`). `--json` 에는 창별 요약(`clock_offset_lower_bound_ms` 포함)·전체·
  사건·시계 하한·임계값과 (`--no-raw` 가 아니면) 같은 열의 원자료가 들어간다.
- MORAI 가 멈추거나 재시작해도 끝나지 않는다 — 연속 실패가 쌓이면 100 ms 간격으로 늦추고 50 회마다 한 줄 알린다.
- 집계 순수부는 `tools/monitor_stats.*`, 검증은 `test/test_monitor_stats.cpp`(합성 업데이트: 평탄 주행,
  몰아 보내기 진동, 등속 잔떨림 히스테리시스, 시계 하한, 유휴 레코드, 조향 0 떨어짐 경계, 창을 넘는 떨어짐, 정체·역행, 모드·기어·충돌 사건,
  Ego 사라짐). 이 모드도 **읽기 전용** 가드(`test_probe_readonly`) 아래에 있다 — `MoraiLink`·`ActorClient`
  와 쓰기 메서드 이름은 `morai_probe.cpp` 에 나타날 수 없다.

### U1 을 믿어도 되는가

`frames`/`simtime` 은 프레임 주기를 **폴링이 그것을 뒷받침할 때만** 낸다(`frame_period_reliable`).
아래 중 하나라도 해당하면 `frame_period_s`·`U1_frame_period_ms` 는 null 이고 `WARNING` 줄이 이유를 말한다.

- `undersampled` — 값 변화 하나당 폴링이 2회 미만. 최빈 델타가 실제 스텝의 배수일 수 있다.
- `poll_limited` — 변화 간격 ≤ 1.5 × 폴링 간격. "주기"가 폴링 주기이거나 연속으로 흐르는 시계일 수 있다.
- `resolution_limited` — 최빈 델타가 1 raw 단위. 연속 시계를 배제할 수 없다.
- `deltas_are_multiples` — 잦은(≥1%) 델타가 최빈값의 정수배(양자화 ±1 raw 허용)가 아니고 더 작은
  공약수를 가진다(예: 15/20 ms → 5 ms). 렌더 프레임마다 timestamp 가 갱신되거나 폴링이 프레임을
  건너뛴 것이다. 공약수는 물리 스텝 후보(`gcd_step_ms`)로 따로 출력된다.

U1 은 두 값을 낸다. `sim_step` 은 프레임당 sim 시간(델타 합 / 프레임 수), `wall_frame_rate_hz` 는
벽시계 초당 프레임 수다. 실시간 모드에서 "프레임마다 명령 한 번" 의 주기는 후자다
(예: 20 ms 스텝을 0.998 배속으로 돌리면 1/sim_step 은 50 Hz, 벽시계로는 49.9 Hz).

정체는 **측정된 주기 기준**으로 센다. 벽시계 간격이 1.5 주기를 넘거나 한 번에 sim 이 2 프레임
이상 진행한 변화가 `missed_frame_events` 이고, 창 길이와 무관하게 `missed_frame_events_per_min`
으로 비교한다. 절대 임계(>20 ms, >45 ms)는 Hz 마다 뜻이 달라 보조 지표로만 남긴다.

### MORAI Hz 설정 비교

MORAI 물리 Hz 는 임의로 바꿀 수 있으므로, 후보 Hz 마다(예: 50·100·200) 같은 시나리오·같은
PC 부하로 `all --json` 을 한 번씩 돌려 아래 값을 나란히 본다.

- `U1_reliable` / `U1_wall_frame_rate_hz` — 설정한 Hz 가 실제 벽시계로 나오는지, 폴링이 그 Hz 를 따라가는지
- `missed_frame_events_per_min`, `max_gap_in_frames` — 주기 기준 정체 빈도와 최악 길이
- `U5_all_calls_ms.p99`, `U5_over_45ms_per_min`, `U5_timeouts` — 통신 꼬리 (명령 한 번 보내는 비용)
- `U17_rate_ratio` — 실시간을 유지하는지 (1.0 미만이면 MORAI 가 그 Hz 를 못 따라감)

### 주의

- **main.py 기본 주소는 10.10.10.1** 이고 이 도구 기본값은 127.0.0.1 이다. 원격 MORAI 면
  `MORAI_SIM_ADDRESS` 나 `--host` 를 준다.
- `waitfortick` 의 `--deadline-ms` 는 한 틱 대기보다 길어야 한다. 짧으면 OK 와
  DEADLINE_EXCEEDED 가 번갈아 나오고 판정은 `blocks_interval_unknown` 이 된다. WaitForTick
  판정에는 **검증된** 벽시계 주기만 기준으로 쓴다.
- MORAI 가 TCP 는 받지만 응답하지 않으면(메인 스레드 정지, 시나리오 로드) rtt·frames·simtime·
  waitfortick 은 DEADLINE_EXCEEDED 10 회 연속 뒤 멈추고 `stopped_early` 로 남긴다.
- `frames` tight loop 자체가 MORAI 부하가 될 수 있다(관측자 효과). 의심되면 `--poll-hz 1000`
  으로 다시 재서 정체 빈도를 비교한다.
- JSON raw 행: rtt·waitfortick 은 호출마다 1행이다. frames·simtime 은 **값이 바뀔 때**(또는 실패 호출,
  actor 비교 시)만 1행을 쓰고, 그 사이 같은 값을 돌려준 폴링 수와 첫/끝 시각을 `repeats_before`·
  `repeat_first_t_ms`·`repeat_last_t_ms` 로 남긴다. 실패 호출도 `grpc_code`·`call_ms` 와 함께 행으로
  남는다. 구간당 `--max-raw-rows` 를 넘으면 나머지는 `dropped_rows` 로 세고 WARNING 을 찍는다.

순수 계산부(`tools/probe_stats.*`, `tools/probe_json.*`, `tools/probe_options.*`)는 gRPC 비의존이라
`test/test_probe_stats.cpp`·`test/test_probe_options.cpp` 가 네트워크 없이 검증한다(100 Hz ms + 중복,
50 ms 정체, 역행 표본, µs 단위, 폴링 부족·폴링 한계로 인한 가짜 주기, 렌더 주기 갱신(15/20 ms → gcd 5),
연속 시계, 49.9 Hz(20.04 ms 스텝·0.998 배속), 200 Hz 의 10/15 ms 끊김, int64 오버플로, .NET epoch,
WaitForTick 판정, 기본 측정 창 길이와 잘못된 옵션 값 거부).

<!-- 26.09.25 _ YSH _ 신규 절 — 엔진용 링크 계층 MoraiLink -->
## MoraiLink — C++ 엔진용 링크 계층

`include/morai_grpc_vendor/morai_link.hpp` (+ 자료형 `morai_link_types.hpp`). main.py 를 대체할 C++ 노드가
MORAI 와 주고받는 모든 것을 blocking + deadline 호출로 감싼다. 기존 `ActorClient`/`SensorClient` 는 그대로다.

**설계 전제 (실측).** 한 업데이트(~50/s, targetFPS 와 무관)에 `GetAllActorsState` 한 번이면 Ego+NPC+
timestamp 가 다 온다. `ControlVehicle` 은 **업데이트당 정확히 한 번** 보낸다 — 비동기 200 Hz 로 몰아
보내면(렌더 프레임당 3 개 이상) MORAI 가속 제어기가 진동했다. `WaitForTick` 은 실시간 모드에서도 업데이트마다
한 번 돌아오지만 값은 ActorStates.timestamp 와 다른 시계라 **트리거로만** 쓴다. 주기·트리거 정책은 이 계층에
없다: 엔진이 `on_update(snapshot)` 본체와 트리거(지금은 WaitForTick, 이후 lockstep `Tick`)를 분리한다.

| 채널 | 호출 |
|---|---|
| 상태 (`LinkChannel::kState`) | `wait_for_tick(deadline)`, `get_snapshot()`, `get_ego_state()`, `get_control_mode()`, `get_vehicle_spec()`, `get_sync_mode()`, `probe_server_time()` |
| 명령 (`LinkChannel::kCommand`) | `control_vehicle(cmd)`, `set_gear(g, verify)`, `set_control_mode(m, verify)`, `set_transform(pose, verify, tol_m, tol_deg)`, `set_tail_light(s)` |

- 두 채널은 서로 다른 TCP 연결이다(역할별 채널 인자 + 로컬 subchannel pool). 큰 상태 응답과 50 ms deadline
  명령이 한 HTTP/2 연결을 나눠 쓰지 않는다. 채널·스텁은 스레드 안전이라 채널마다 다른 스레드에서, 또
  한 채널을 여러 스레드에서 동시에 써도 된다. 재연결 백오프 상한은 1 s (gRPC 기본 120 s 면 MORAI 재기동 뒤
  최대 2 분 동안 붙지 않는다).
- **keepalive** (`keepalive_time_ms`, 기본 0 = 끔). 원격 MORAI 와의 TCP 가 RST 없이 죽으면(케이블·스위치
  단절) keepalive 없이는 커널 재전송 한도(수 분)까지 채널이 READY 로 남고 호출은 DEADLINE_EXCEEDED 만 낸다.
  켜면(예: 5000 / `keepalive_timeout_ms` 1000, 호출 중일 때만 PING) 끊김을 감지해 재연결한다. MORAI 서버의 PING
  정책을 아직 몰라(너무 잦으면 GOAWAY `too_many_pings`) 기본은 끔이다 — 서브 PC 에서 켜 보고 `counters()`·
  monitor 로 끊김이 없는지 확인한 뒤 쓴다. 끈 채로도 엔진이 연속 DEADLINE_EXCEEDED 를 세다가 `MoraiLink` 를
  새로 만들면 복구된다.
- 모든 결과는 `Status {ok, grpc_code, result_status, message, rpc_ms, readback_checked, readback_matched}`
  또는 `Reply<T>{status, value}`. 예외는 API 밖으로 나가지 않는다. Result 를 돌려주는 RPC 는
  `Result.status == SUCCESS(1)` 까지 본다(0 은 UNKNOWN_FAILURE). NaN·inf 명령·자세는 보내지 않는다.
- **되읽기 확인.** MORAI Result 는 거짓일 수 있다(SetSynchronousMode 실측). `set_gear`/`set_control_mode`/
  `set_transform` 에 `verify_timeout > 0` 을 주면 상태 채널로 되읽어(`readback_poll` 10 ms 간격) 그 결과가
  `ok` 가 된다. DEADLINE_EXCEEDED 는 적용 여부를 모르므로 되읽고, UNAVAILABLE 같은 전송 실패는 되읽지 않는다.
- 값은 **MORAI 원값 그대로**다: 위치 ENU m, rotation deg, velocity local km/h, angular_velocity world rad/s
  원축(-z, x, -y), bounding box 반치수(x=반폭, y=반길이), front_wheel_angle Unity 부호, 명령 steer 는 타이어각
  rad. 단위·부호·축 변환은 전부 `autoware_morai_interface` conversion 이 맡는다(조향 부호 비대칭 규칙을 한
  곳에 둔다). `EgoSample.valid` 는 daq_publisher 의 all-zero 규칙, Ego 식별은 main.py 와 같은 `"Ego"`
  (정확 일치 첫 항목, 없으면 대소문자 무시 첫 항목 — `exact_match` 로 구분). `Snapshot.actors` 는 main.py
  TrackedObjects 처럼 id 가 정확히 `"Ego"` 인 항목을 **전부** 뺀 나머지다(대체 매치로 ego 가 된 항목도 뺀다 — 자차가 장애물로 나가지 않게),
  `client_key` 와 필터 `client_key` 는 main.py 처럼 비운다(필터 키를 채우면 actor id 체계가 바뀔 수 있다).
- **시계.** 스냅샷은 요청 직전·응답 직후 system/steady 시각(ns)을 싣는다. MORAI 에는 호출 시점 서버 시각이
  없다 — `probe_server_time()`(`Simulator.GetTimestamp`)도 스냅샷과 같은 **프레임 시각**이라 한 표본은 하한
  (`offset_lower_bound_ms = server − recv`)만 준다. 여러 표본의 `clock_offset_lower_bound_ms()`(= max(server − recv))
  를 같은 PC 에서 잰 기준선과 빼서(`clock_offset_from_baseline_ms()`) 원격 오프셋을 본다. 방법은 위 monitor 의
  "시계 오프셋 읽는 법" 과 같다.
- 기본 deadline 은 main.py 실전 값: 상태 100 ms, ControlVehicle 50 ms, 기어·모드 200 ms, SetTransform 1 s,
  스펙 3 s.

검증: `test/test_morai_link.cpp` (네트워크 없음 — 변환·main.py 식 Ego 제외·명령 채우기·Result 0/1·되읽기
일치·yaw 래핑·프레임 시계 하한과 기준선(합성 60 fps 프레임 + 1 kHz 폴링, NTP 식이 틀리는 반례 포함)·닫힌
포트로 만든 링크의 실패 경로와 카운터), `test/test_morai_link_isolation.cpp` (공개 헤더 격리).

<!-- 26.09.25 _ YSH _ 신규 절 — Ego 를 움직이는 구동 측정 도구 morai_actuate 사용법 -->
## morai_actuate — ControlVehicle 소비 단위 측정 (Ego 를 움직인다)

**목적.** MORAI 가 `ControlVehicle` 을 **업데이트당**(렌더/업데이트 루프, 실측 ~50 회/s) 소비하는지
**물리 스텝당**(MORAI Hz, 60/100/200) 소비하는지 가린다. 명령을 받지 못한 프레임은 이전 명령을
유지하지 않고 유휴 레코드(throttle 0, brake ~0.003)로 돈다는 것은 main.py 에서 이미 알려져 있다.
스텝당 소비라면 MORAI Hz 를 올릴수록 어떻게 보내도 빈 스텝이 생긴다.

Ego 를 정지 상태에서 **직진(조향 항상 0)** 으로 `+accel`(기본 0.8 m/s², `LONG_CMD_TYPE_ACCELERATION`,
main.py `_fill_longitudinal` 기본 모드와 같음) 을 MORAI 시간 `--drive-s`(기본 2 s) 동안 명령한 뒤
세운다. 매 런은 0.5 s 브레이크 유지 직후에 출발한다(모든 런이 같은 정지 상태에서 시작). 이를 송신
패턴별로 반복한다(기본 2 회, 교차 순서 `--order abba`: A B C, C B A … — 선형 드리프트가 상쇄된다.
`--order fixed` 는 A B C, A B C).

| 패턴          | 송신 방식                                                |
| ------------- | -------------------------------------------------------- |
| `locked1`     | WaitForTick 이 돌아올 때마다 1 회 (프레임 동기)           |
| `locked_half` | WaitForTick 2 회마다 1 회                                |
| `burst4`      | WaitForTick 마다 연달아 4 회                             |
| `free33`      | 33.3 Hz 타이머, 블로킹 (Autoware control_cmd 1 회 송신)  |
| `free100`     | 100 Hz 타이머, 블로킹 (main.py 재송신기)                 |
| `free200`     | 200 Hz 타이머, 비동기 파이프라인 (`--max-inflight` 상한) |

별도 채널의 상태 읽기 스레드가 `GetAllActorsState` 를 tight loop 로 돌며 **서로 다른 timestamp 마다**
한 행(MORAI µs·로컬 시각·속도(km/h→m/s, main.py 와 동일)·위치·가속도·vehicle_state·제어 모드)을 남기고,
송신 로그는 송신마다 로컬 시각·패턴·직전 WaitForTick 값·RPC 지연·상태를 남긴다.

### 안전 장치

- `--i-understand-this-moves-the-ego` 가 없으면 **실행을 거부**한다. 할 일을 배너로 보여 주고 3 초
  기다린다(`--no-countdown` 으로 생략). 카운트다운 중 Ctrl-C 는 아무것도 바꾸지 않고 끝난다.
- 사전 점검(읽기 전용): 연결, Ego 존재, |v| < 0.05 m/s, 실시간 모드(동기 모드면 거부), MORAI 시간이
  흐르는지, **원래 제어 모드가 KEYBOARD / GAME_WHEEL 인지**(AUTO 면 브릿지·Autoware 가 아직 보내고 있을
  수 있다 — 그들의 페달 0 ACCELERATION 명령은 레코드가 유휴처럼 보여 페달만으로는 못 가린다. 그래도
  넘겨받으려면 `--allow-any-original-mode`), 다른 클라이언트의 입력이 없는지(페달 또는 **조향 에코**
  |steer|·|front_wheel_angle| > 1e-3 인 레코드가 20% 이상이면 거부), **원래 제어 모드·기어 기록**.
  하나라도 실패하면 아무것도 바꾸지 않고 종료 코드 2.
- 준비: `SetVehicleControlMode(VEHICLE_CONTROL_AUTO_MODE)`, `SetVehicleGear(GEAR_MODE_D)` 후 되읽기(기어는
  설정 뒤에 받은 상태로만 인정). 각 호출 직전에 신호를 확인해, 중단 요청 뒤에는 더 바꾸지 않는다.
  0.5 s 브레이크 유지로 ControlVehicle 수락을 확인하고, 명령 없는 0.5 s 로 유휴 레코드를 실측한다. 이
  창에서 조향 에코가 보이면(AUTO 가 되자 다른 클라이언트가 보내기 시작) 중단한다.
- 런마다 봉투(envelope): 속도 > `--max-speed`(3.0 m/s), 런 시작점에서 평면 변위 > `--max-distance`(15 m),
  상태 읽기 실패 > 0.5 s, MORAI 시간 정지 > 1 s, 충돌 보고, **조향 에코**(이 도구는 조향 0 만 보내므로
  에코가 보이면 다른 클라이언트) → 그 런 중단 → 안전 정지 → 세션 중단. 조향 0 보장은 아무도 다른
  명령을 보내지 않을 때만 성립하므로 이 검사가 그것을 지킨다. 세션 누적 이동이
  `--max-total-distance`(60 m) 를 넘을 것 같으면 다음 런을 시작하지 않는다.
- 안전 정지: acceleration -3.0 m/s², 조향 0 을 100 Hz 로, |v| < 0.05 m/s 가 1 s 이어질 때까지(최대 10 s).
  브레이크 송신마다 수락/거부를 세어 출력·JSON 에 남긴다(`safe-stop OK (4.0 s), 0/400 brake sends
  accepted` 처럼). 수락이 0 이면 "스스로 멈췄을 뿐 브레이크가 먹지 않았다" 경고이며 다음 런을 시작하지
  않는다(프리 홀드·정착 유지도 같다).
- **복구 보장**: 정상 종료·예외·SIGINT/SIGTERM/SIGHUP/SIGQUIT(Ctrl-\\)/SIGTSTP(Ctrl-Z)(핸들러는 플래그만
  세움)·gRPC 실패 어느 경로든, 무엇이든 바꾼 뒤라면 RAII 가드가 안전 정지 → **원래 기어** → **원래 제어
  모드** 를 보내고 되읽어 결과를 출력한다. 직전 안전 정지가 상태 끊김·브레이크 전부 거부(MORAI 불통·명령 거부)로
  실패했으면 복구의 안전 정지는 2 s 로 줄이고 곧장 기어·모드를 시도한다. 복구 중 Ctrl-C 는 무시한다:
  보통 수 초, MORAI 불통이면 ~16 s(런 안전 정지 10 s + 2 s + 기어·모드 설정·되읽기 ~3.6 s), 최악(차가
  브레이크에도 서지 않음) ~25 s. 이 값은 `--deadline-ms` 에 비례해 늘어난다.
- 도구가 강제 종료(kill -9)되거나 죽으면 복구가 돌지 않는다 — **MORAI UI 에서 Ego 를 직접 KEYBOARD / P 로**
  돌린다(MORAI 는 명령을 유지하지 않아 AUTO/D 로 남은 Ego 는 유휴 레코드로 굴러간다).
- 상태를 바꾸는 RPC 는 전부 deadline 을 걸고 `Result.status == SUCCESS(1)` 를 확인한다.
  `test/test_actuate_scope.cpp` 가 소스를 스캔해 호출 가능한 RPC 를 ControlVehicle·SetVehicleControlMode·
  SetVehicleGear 와 읽기로 제한하고, 조향이 리터럴 0.0 인지 확인한다. 읽기 전용 가드
  (`test_probe_readonly`)는 계속 `morai_probe.cpp` 만 본다.

기본값이면 런 하나에 약 2.9 m, 12 런에 약 35 m 를 **직진**한다. 긴 빈 직선로에 Ego 를 두고,
브릿지(main.py)·Autoware·키보드 조작이 없는 상태에서 돌린다.

### 실행

워크스페이스 루트, 깨끗한 셸에서 (MORAI UI 에서 Hz 를 60 → 100 → 200 으로 바꿔 가며 한 번씩):

```bash
env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin TERM=xterm LANG=C.UTF-8 bash --noprofile --norc -c \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 run morai_grpc_vendor morai_actuate --i-understand-this-moves-the-ego --host 127.0.0.1 \
     --json ~/Data/cpp_iface/actuate/$(date +%Y%m%d-%H%M%S).json'
```

**MORAI Hz 는 실행 사이에 MORAI UI 에서 바꾼다**(60 → 100 → 200). 200 Hz 처럼 업데이트마다 항상 같은
스텝 수(예: 4)만 진행하면 timestamp 만으로는 스텝을 확정할 수 없으므로 `--physics-hz 200` 처럼 설정한
Hz 를 함께 준다(추정과 대조해 경고). 종료 코드: 0 정상,
1 사용법/거부, 2 연결·사전 점검 실패(변경 없음), 3 JSON 저장 실패, 4 준비 실패(복구됨),
5 중단(복구됨), 6 **복구 실패 — 즉시 MORAI 확인**.

### 결과 읽기

런마다 한 줄, 끝에 패턴별 표와 판정을 낸다.

- `ratio` — [0.5 s, drive_s] 구간 속도-MORAI 시간 최소제곱 기울기 / 명령 가속도.
- `idle%` — 드라이브 구간 업데이트 중 유휴 레코드(throttle≈0, brake≈유휴값) 비율.
- `sends/upd` — 업데이트 사이에 실제로 들어간 송신 수(송신 직전 알려진 MORAI timestamp 기준으로 귀속).
- `steps/upd` — timestamp 차이 / 물리 스텝(빈번한 델타 군집의 공약수, 최빈값이 아님). `upd/step` 은 그
  역수 평균(업데이트당 명령 하나가 구동할 수 있는 스텝 비율, 60 Hz ~0.83, 200 Hz ~0.25).
- `dv~k+-se` — 명령을 받은 업데이트의 속도 변화가 진행 스텝 수 k 에 비례하는 몫과 표준오차.
  **1 이면 업데이트당**(업데이트의 모든 스텝이 그 명령으로 구동), **0 이면 스텝당**(한 스텝만 구동).
  200 Hz 스텝당이면 평균 속도 변화가 작아 속도 잡음이 크게 부풀므로, 몫이 2 SE 를 넘어 반대 모델 쪽
  범위(업데이트당 표는 몫 ≥ 0.75 이고 몫-2SE > 0.35, 스텝당 표는 몫 ≤ 0.35 이고 몫+2SE < 0.75)를
  벗어날 때만 표로 친다. `a(k>=2)/a(k=1)` 은 60 Hz 에서 같은 것을 직관적으로 보인다.
- `a(0snd)/a(snd)` — 명령 없는 업데이트의 가속 / 명령 받은 업데이트의 가속. ~0 이면 유지(hold) 없음.
- `attribution` 줄 — 송신을 어느 업데이트에 귀속할지(지연 L: prev 뒤 (1+L) 번째 업데이트)를 locked_half·
  free33 처럼 명령 없는 업데이트가 섞인 런에서 L=0,1,2 별 가속 대비(물리 지연)·비유휴 레코드 대비
  (레코드 지연)로 추정해 분석 전체에 물리 지연을 쓴다. 명령이 한 업데이트 늦게 먹히면 L=0 귀속은
  locked_half·free33 을 뒤집어 "MORAI 가 명령을 유지한다" 로 오판한다. 두 지연이 다르면 경고한다. 같은 줄에
  WaitForTick 값이 상태 읽기가 본 ActorStates.timestamp 와 정확히 일치한 비율(시계 동일성·놓친 업데이트
  확인, 95% 미만이면 경고)을 낸다.
- 판정 `per_update` / `per_step` / `inconclusive` 와 해석. **물리 표(dv~k)가 있어야만** 판정한다. 유휴
  레코드 표는 확인하거나(`confirmed`) 반대여서 `inconclusive` 로 강등할 뿐이며, 양성 대조 — 명령을 받은
  k=1 업데이트의 레코드가 유휴로 보이는 비율 ≤ 10% — 를 통과해야 쓴다(레코드가 스텝 단위가 아니라
  "명령 도착" 에코이거나, 소비된 페달 0 명령도 brake 0.003 을 에코하면 이 표는 틀린 답을 낸다). k=1
  업데이트가 없는 200 Hz 에서는 쓰지 않는다. 모든 업데이트가 같은 k 만큼 진행하거나 dv~k 가 잡음 안이면
  `inconclusive` 와 함께 Hz 간 비교 지시를 낸다: locked1 ratio 를 60 Hz 실행과 비교 — 업데이트당이면 같고,
  스텝당이면 upd/step 비율만큼(60 Hz 의 ~0.83 대비) 떨어진다.

`--json` 에는 원자료(서로 다른 timestamp 마다의 상태 행, 모든 송신, WaitForTick), 런별 분석(브레이크 송신
수락 수 포함), 단계별 (throttle, brake) 레코드 분포, 물리 스텝 추정, 귀속 지연 추정, WaitForTick 일치율,
런 순서, 옵션, 복구 결과가 들어간다. 순수 계산부(`tools/actuate_analysis.*`, `tools/actuate_options.*`)는
`test/test_actuate_analysis.cpp`·`test/test_actuate_options.cpp` 가 합성 데이터로 검증한다(60 Hz 1/2 스텝·
200 Hz 3~5 스텝에서 업데이트당/스텝당 소비, 200 Hz 속도 잡음 σ 5·10 mm/s 에서 반대 모델로 판정하지
않음, 균일 k·"명령 도착" 에코·항상 유휴 레코드에서 유휴 표가 판정을 뒤집지 않음, 1 업데이트 지연 추정과
유지 오판 방지, locked_half 무유지, 송신 귀속, 정체, 단위 오류 검출, 판정·충돌 증거, 봉투(조향 에코
포함), 원래 모드 정책, 페달 0 브릿지의 조향 에코 검출, 확인 플래그·잘못된 값 거부, 런 순서).

<!-- 26.09.25 _ YSH _ 신규 절 — morai_actuate --lockstep (MORAI 동기 모드, ADS 가 MORAI 를 페이싱) -->
## lockstep mode — `morai_actuate --lockstep` (MORAI 동기 모드, Ego 를 움직인다)

**목적.** Autoware 인터페이스의 LOCKSTEP 옵션 근거: ADS 가 MORAI 를 페이싱한다. MORAI 를
`SYNCHRONOUS` 모드로 두고 이 도구가 `Simulation.Tick` 으로 한 스텝씩 진행시키며, Tick 마다 명령을
하나 넣는다. sim 시간은 실시간보다 느리게 흐를 수 있지만 모든 측정은 정확한 sim 시간이다. 실측으로
가려야 할 미지수 두 가지를 잰다.

1. 명령 없이 Tick 한 스텝에서 MORAI 가 **직전 입력을 유지(held)** 하는가, **유휴 레코드(idle)** 로
   떨어지는가 (실시간 모드에서는 유휴로 떨어진다)
2. `Tick(n>1)` 한 번 안에서 입력이 **n 프레임 내내 유지(persists)** 되는가, **첫 프레임만(first frame
   only)** 인가

### 확인된 사실 (로컬 MORAI R1.260701.H3 실측 + 서버 IL 분석)

- `SetSynchronousMode(SYNCHRONOUS, tick_period)` 는 실행 중에 먹지만 **Result 가 적용돼도 status 0
  'Unknown error'** 다 → Result 를 믿지 않고 `GetSynchronousMode`(type 만 준다, tick_period 는 항상 0)
  와 timestamp 동작으로 확인한다. tick_period 0 → 10 ms, 1..100 밖 → 20 ms. type/주기를 바꾸려면
  UNSPECIFIED 를 거쳐야 한다.
- 전환 직후 **~1 s 동안 gRPC 서버가 UNAVAILABLE** ('failed to connect to all addresses') → 최대 5 s
  백오프 재시도 (채널 재연결 백오프도 100~500 ms 로 줄였다).
- 동기 모드: Tick 전까지 세계 정지, sim 시간·프레임 수 리셋 (`ActorStates.timestamp` = 진입 후 sim 시간
  µs, **epoch 아님**; `sync_mode_tick_count` = frame_count). Ego 제어 모드는 서버가 **7
  (SYNCHRONOUS_MODE)** 로 강제하고 해제 때 되돌린다. `ControlVehicle` 은 모드 3·7 에서만 적용되고 그 밖에는
  서버 로그만 남기고 조용히 버린다.
- `Tick(n)`: n 프레임을 진행하고 상태 스냅샷을 갱신한 뒤 돌아온다(그 뒤 읽기는 새 프레임을 본다).
  `Tick(1)` 은 timestamp 를 정확히 tick_period 만큼(100/100) 올린다. RPC 한 번 ≈ 16.5 ms 벽시계(MORAI
  ~60 fps 루프) → lockstep 속도 ≈ tick_period / 16.5 ms (10 ms 면 ~0.6 배속). 바쁨·일시정지·권한 없음이면
  frame_count 를 그대로 둔 채 즉시 돌아온다 → frame_count 증가로 검증. `SyncTimestamp.elapsed_time` 은
  sim 시간이 아니다.
- 큐에 쌓인 `ControlVehicle` 은 **다음 Tick 시작 시 한 번** 적용(여러 개면 마지막 것),
  `VehicleCtrlCmd.frame` 은 gRPC 경로에서 무시.
- 동기 모드의 `WaitForTick` 은 즉시 0 을 돌려준다 → lockstep 경로에서는 쓰지 않는다.
- `SetSynchronousMode(UNSPECIFIED, 0)` 은 Success 를 주고 즉시 실시간으로 돌아온다(epoch timestamp,
  60 Hz 스텝, 모드 KEYBOARD).
- **클라이언트가 동기 모드에서 죽으면 MORAI 는 얼어 있는 채로 남는다** (서버가 아무것도 하지 않는다)
  → `morai_sync_restore`.

### 가정·미확인 (도구는 어느 쪽이어도 안전하게 동작한다)

- **동기 모드에서 `SetVehicleGear` 가 언제 적용되는가** — 즉시인지, 다음 Tick 시작인지, 동기 모드 해제 때
  (큐에 남은 기어)인지 확인하지 않았다. 도구는 어느 쪽이든 기어를 Tick 뒤 최대 5 Tick 동안 되읽고, 복구의
  원래 기어(보통 P)는 **정지가 확인된 뒤에만** 보낸다(Tick 안전 정지 성공, 또는 실시간에서 새로 읽은 |v| <
  0.05 m/s). 가짜 MORAI 는 비관적으로 "다음 Tick 시작 + 해제 때 큐에 남은 기어 적용" 으로 모델링한다.
- 명령 없는 Tick 에서 입력 유지 여부, `Tick(n>1)` 안에서 입력 유지 여부 — 이 모드가 재는 두 미지수.
- 바쁨·권한 없음 Tick 의 `SyncTimestamp` 내용(기본값 frame_count 0 일 수 있음) — 판정은 sim 시간
  (`ActorStates.timestamp`)이 기준이라 어느 쪽이든 "진행 안 함" 으로 센다.
- 동기 모드 유휴 레코드의 brake 값 (실시간 AUTO 무명령은 1.0 을 보고) — 첫 명령 전에 잰 값을 그대로 쓴다.

### 패턴

모두 정지에서 `+accel`(기본 0.8 m/s², `LONG_CMD_TYPE_ACCELERATION`) 을 sim 시간 `--drive-s`(기본 2 s)
동안, 조향은 항상 0. 끝나면 Tick 으로 브레이크해 세우고 정착. 매 런은 0.5 s 브레이크 유지 직후 출발.
`--repeats`(기본 2) `--order abba`.

| 패턴          | 한 스텝                                                        | 답하는 것                          |
| ------------- | -------------------------------------------------------------- | ---------------------------------- |
| `sync1`       | ControlVehicle → Tick(1) → 읽기                                | 기준: 명령 받은 1 프레임의 가속    |
| `sync_drop2`  | 짝수 스텝만 ControlVehicle, 홀수 스텝은 명령 없이 Tick(1)      | 명령 없는 스텝: 유지 vs 유휴       |
| `sync_tick3`  | ControlVehicle 한 번 → Tick(3) → 읽기                          | Tick(3) 안에서 입력이 유지되는가   |
| `sync_tick3x` | ControlVehicle 한 번 → Tick(1) ×3 (2·3번째는 명령 없음)       | 유지 vs 유휴 + sync_tick3 과 비교  |

스텝마다 frame_count, elapsed_time, `ActorStates.timestamp`, `sync_mode_tick_count`, 속도(km/h→m/s,
main.py 와 같음), 위치, vehicle_state 레코드(throttle/brake/steer/기어), Tick 벽시계 ms, 앞에 명령이
있었는지를 남긴다(`--json` 의 `raw.steps`).

### 분석 (gRPC 비의존, `tools/lockstep_analysis.*`, gtest `test/test_lockstep_analysis.cpp`)

- (a) **dt**: 연속 Tick 마다 frame_count +n, timestamp +n × tick_period (±1 µs). 위반은 종류별
  (`not_advanced`, `frame_mismatch`, `dt_mismatch`, `jumped`, `tick_failed`, `no_read`)로 세고 예시를 낸다.
- (b) **ratio**: 런마다 [0.5 s, 끝] 속도-sim 시간 최소제곱 기울기 / 명령 가속도.
- (c) **유지 판정**(`sync_drop2`, `sync_tick3x`) — **1차: 패턴 수준 q = ratio(패턴) / ratio(sync1)**.
  패턴은 `cmd_every` Tick 에 한 번 명령한다. MORAI 가 입력을 유지하면 모든 Tick 이 구동돼 q ≈ 1, 버리면
  명령 받은 Tick 만 구동돼 q ≈ 1/cmd_every (구름 저항만큼 약간 아래: 가짜 MORAI 0.465 / 0.287). 띠는
  h0 = 1/cmd_every 로 옮긴 같은 띠: **held** q ≥ h0 + 0.75(1-h0) 이고 q-2SE > h0 + 0.35(1-h0), **idle**
  q ≤ h0 + 0.35(1-h0) 이고 q+2SE < h0 + 0.75(1-h0) (drop2: 0.875 / 0.675, tick3x: 0.833 / 0.567).
  ratio 의 SE 는 런 간 산포(sd/√n)와 런 안 최소제곱 기울기 SE 중 큰 쪽, 런끼리는 독립이라 델타법이 맞다.
  **액추에이터·파워트레인 지연(수 Tick)** 은 실제 가속을 이웃 Tick 에 번지게 해 Tick 단위 비교를 속이지만
  드라이브 전체 평균은 바꾸지 못한다 → q 는 지연에 속지 않는다.
  **2차: Tick 단위 share** = 명령 없는 Tick(1) 가속 / 명령 받은 Tick(1) 가속. 지연은 share 를 1(유지) 쪽으로만
  끌어당긴다(τ 30 ms 에서 버려진 입력도 share 0.94). 그래서 share 는 (1) **late** 검출(명령 받은 Tick 은 가속하지
  않는데 명령 없는 Tick 이 가속 = 명령이 한 Tick 늦게 먹음), (2) **conflict** 검출(q 는 유지인데 share 는 유휴 —
  지연으로 설명되지 않음), (3) `sync1` 이 없을 때의 대체(그때는 지연으로 생기지 않는 "idle" 만 믿고 "held" 는
  미정)에만 쓴다. q 가 유휴인데 share 가 유지면 판정은 **idle** 이고 "지연" 설명을 붙인다. share 의 델타법 SE 는
  이웃 dv 가 속도 표본을 공유하는 상관을 무시해 1.2~1.5 배 작으므로 share 띠는 **3 SE** 여유를 요구한다.
- (d) **Tick(3) 판정**: m = ratio(`sync_tick3`) / ratio(`sync1`) (패턴 수준, 지연에 속지 않음; `sync1` 이
  없으면 Tick 단위 대체 — 모든 패턴의 명령 Tick(1) 기준). 유지면 m ≈ 1, 첫 프레임만이면 m ≈ 1/3 (유휴 감속만큼
  약간 아래). 같은 띠를 [1/3, 1] 로 옮겨 **persists** m ≥ 0.833 이고 m-2SE > 0.567, **first_frame_only**
  m ≤ 0.567 이고 m+2SE < 0.833. `sync_tick3x` 도 같은 척도(m_chain)로 내서 "Tick(3) 이 Tick(1)×3 과
  같게/다르게 동작" 을 적는다.
- (e) **lockstep 속도**: sim 초 / 벽시계 초(드라이브 구간), Tick(1)·Tick(n) RPC p50/p90/max.
- (f) **유휴 레코드**: 명령 없는 Tick(1) 의 레코드가 유휴(throttle≈0, brake≈셋업에서 잰 값)인 비율.
  양성 대조(명령 받은 Tick(1) 레코드 유휴 ≤ 10%)를 통과해야 표를 낸다. 물리 판정과 반대면 경고. 유휴 서명은
  **기어 D 직후, 이 세션의 첫 ControlVehicle 전**에 잰다(붙들 입력이 없으므로 유지된 명령이 유휴로 잡히지
  않는다). 그래서 실시간 규칙(brake 0..0.05)과 달리 brake 1.0 까지 유휴 레코드로 받는다. `Tick(n)` 뒤에 읽은
  레코드(그 Tick 의 마지막 프레임)도 같은 방식으로 표를 내어(유휴 = 첫 프레임만, 명령 = 유지) (d) 와 반대면
  경고한다.

요약 표에는 패턴마다 ratio ± SE, 런 간 sd, `vs sync1`(= q, m) 이 나오고, 끝에 한 줄 해석이 나온다. 예:
`sync1 ratio ≈ 0.90; uncommanded steps: idle (input dropped - command every Tick); Tick(3): only the
first frame is driven; lockstep speed ≈ 0.60× real time; dt exact in 3744/3744 Ticks` (가짜 MORAI e2e 출력;
τ 50 ms 지연을 넣어도 같은 줄 — Tick 단위 share 는 0.97 로 "유지처럼" 보이지만 판정은 q 0.465 로 idle).

### 안전 장치 (실시간 모드와 같은 설계 + 동기 모드 전용)

- `--i-understand-this-moves-the-ego` **와** `--lockstep` 이 둘 다 있어야 한다. 배너 + 3 초 카운트다운
  (카운트다운 중 Ctrl-C 는 아무것도 바꾸지 않는다).
- 사전 점검(읽기 전용, 실시간): **`GetSynchronousMode` 가 UNSPECIFIED** 여야 한다(아니면 다른
  클라이언트가 쓰는 중이거나 이전 실행이 죽은 것 — 아무도 안 쓰면 `morai_sync_restore` 먼저). 그 밖은
  실시간 모드와 같다: 시간이 흐르는지, 다른 입력(페달·조향 에코) 없음, |v| < 0.05 m/s, 원래 제어 모드
  KEYBOARD / GAME_WHEEL(`--allow-any-original-mode`), **원래 제어 모드·기어 기록**. 실패하면 아무것도
  바꾸지 않고 종료 코드 2.
- 진입 검증: Result 는 기록만, `GetSynchronousMode == SYNCHRONOUS`(5 s 재시도), Tick 없이 0.3 s 이상
  timestamp 정지, `Tick(1)` 두 번이 frame_count +1 / timestamp 정확히 +tick_period(±1 µs), 제어 모드가
  7(또는 3). 하나라도 아니면 중단 → 복구 (종료 코드 4). 기어 D 는 다음 Tick 에 적용되므로 Tick 뒤
  되읽는다(5 Tick 안에). `SetVehicleControlMode` 는 동기 모드에서 쓰지 않는다(복구 때만).
- 봉투: 속도 > `--max-speed`, 런 시작점에서 > `--max-distance`, 상태 읽기 실패 > 0.5 s(벽시계), 충돌,
  조향 에코. **정지는 Tick 단위로 본다** — Tick 뒤 timestamp 가 3 번 연속 그대로면(바쁨·일시정지·권한 없음;
  frame_count 가 무엇이든 — 기본값 0 이 와도), Tick RPC 가 3 번 연속 실패하면, **최근 5 Tick 중 3 번**이
  실패·정지면(실패·성공이 번갈아도), timestamp 가 1 s 넘게 튀면(누가 UI 에서 동기 모드를 끔) 중단. 실패한
  Tick 다음 Tick 은 마지막 정상 Tick 과 비교한다. 첫 런에서 Ego 가 거의 안 움직이면(명령이 조용히 버려짐)
  남은 런을 돌리지 않는다.
- 안전 정지(lockstep): 브레이크(-3.0 m/s²) + `Tick(1)` 반복, |v| < 0.05 m/s 가 **sim 1.0 s** 이어질 때까지.
  상한은 max(2000 Tick, sim 20 s) — 10 ms 에서 2000 Tick, 1 ms 에서 20000 Tick(2000 Tick = sim 2 s 로는
  10 m/s 에서 서고 1 s 유지하기에 모자람) — 과 벽시계 120 s(1 ms·10 m/s 의 정상 정지 ~78 s 보다 넉넉;
  Tick 이 기어가듯 느린 MORAI 가 복구를 붙잡지 못하게). Tick 이 실패·정지하면(이 안전 정지의 Tick 으로 새로
  판정) 포기하고 복구로 넘어가며, 복구는 동기 모드를 풀자마자 실시간으로 브레이크한다. 앞선 안전 정지가
  실패했으면 복구의 Tick 안전 정지는 벽시계 10 s 로 짧게 한다.
- **복구(RAII + 신호 플래그, 모든 종료 경로: 정상·예외·중단·SIGINT/TERM/HUP/QUIT/TSTP)**, 단계마다 출력,
  **단계마다 따로 예외를 잡는다**(앞 단계가 예외를 내도 동기 모드 해제는 건너뛰지 않는다; 그래도 예외가 밖으로
  나오면 배너 + 종료 코드 6):
  1. 안전 정지(Tick) → 2. 원래 기어 — **1 이 정지를 확인했을 때만** 보내고 Tick 뒤 되읽기 (움직이는 채로 P 를
  넣지 않는다; 아니면 6 으로 미룸) → 3. `SetSynchronousMode(UNSPECIFIED, 0)` 을 `GetSynchronousMode` 가
  UNSPECIFIED 가 될 때까지 최대 10 s 재시도(Result 무시; 이미 UNSPECIFIED 면 보내지 않음) → 3b. 풀리자마자
  한 번 읽어 Ego 가 아직 움직이면(Tick 안전 정지 실패) **1 s 확인보다 먼저** 실시간 안전 정지(`AUTO_MODE` +
  -3.0 m/s² 100 Hz, |v| < 0.05 m/s 1 s, 최대 10 s, 시간이 1.5 s 멈춰 있으면 포기 — 원래 모드 KEYBOARD 에서는
  ControlVehicle 이 무시되므로) → 4. timestamp 가 벽시계 속도(0.5~1.5 배)로 흐르고 epoch 기반인지 1 s 확인 →
  4b. 그래도 움직이면 실시간 안전 정지 → 5. 제어 모드가 원래 값인지(서버가 되돌린다; 아니면
  `SetVehicleControlMode(원래)` 후 되읽기 — **동기 모드가 아닐 때만**: 실시간이 확인됐거나 이 복구가 3b 에서
  스스로 AUTO 로 바꿨을 때. 아니면 읽고 보고만 하고 `morai_sync_restore`/UI 에 맡긴다) → 6. 기어 재확인 —
  설정은 **실시간이 확인되고 새로 읽은 |v| < 0.05 m/s 일 때만**(아니면 "NOT sent: Ego still moves") →
  7. 정지 확인(0.5 s 동안 3 번 이상 읽어 모두 |v| < 0.05 m/s). 복구 성공 = 실시간 확인 + 모드 + 기어 + 7 의
  정지(안전 정지의 성패가 아니라 마지막 상태로 판정). 복구 중 Ctrl-C 는 무시한다(안전 정지·재시도에 벽시계
  상한이 있다). 3·4 가 실패하면 종료 코드 **6** 과 함께 stdout·stderr 에
  **`MORAI may be frozen in synchronous mode — run morai_sync_restore`** 를 크게 찍는다.
- 도구가 kill -9 되거나 죽으면 복구가 돌지 않고 **MORAI 는 동기 모드로 얼어 있다** → 아래
  `morai_sync_restore` (또는 MORAI UI Time Manager).
- 소스 범위: `SetSynchronousMode`·`Tick` 을 부르는 것은 `tools/actuate_lockstep.cpp`(이 모드)와
  `tools/morai_sync_restore.cpp` 뿐이다. `test/test_actuate_scope.cpp` 가 tools/ 전체를 스캔해 확인하고,
  lockstep 경로의 허용 RPC(SetSynchronousMode, Tick, ControlVehicle 조향 0·ACCELERATION 만,
  SetVehicleGear, 복구용 SetVehicleControlMode, 읽기)와 `WaitForTick` 부재, 상태 변경 호출의 deadline 을
  검사한다. 실시간 경로(`morai_actuate.cpp`)는 `--lockstep` 이면 `run_lockstep()` 으로 넘길 뿐이고 여전히
  동기 모드 RPC 를 부르지 않는다. `morai_probe` 읽기 전용 가드는 그대로다.

기본값이면 런 하나에 약 2.9 m, 8 런(4 패턴 × 2)에 약 23 m 를 **직진**한다. 벽시계로는 약 1 분(Tick 당
~16.5 ms). 긴 빈 직선로에서, 브릿지(main.py)·Autoware·키보드 조작이 없고 **다른 누구도 동기 모드를 쓰지
않을 때** 돌린다.

### 실행

워크스페이스 루트, 깨끗한 셸에서:

```bash
env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin TERM=xterm LANG=C.UTF-8 bash --noprofile --norc -c \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 run morai_grpc_vendor morai_actuate --i-understand-this-moves-the-ego --lockstep \
     --host 127.0.0.1 --tick-period-ms 10 \
     --json ~/Data/cpp_iface/lockstep/$(date +%Y%m%d-%H%M%S).json'
```

옵션(lockstep 전용): `--tick-period-ms`(1..100, 기본 10 — MORAI 범위 밖 값은 서버가 20 ms 로 바꾸므로
거부), `--tick-deadline-ms`(Tick RPC deadline, 기본 1000), `--patterns`(sync1,sync_drop2,sync_tick3,
sync_tick3x 중에서). `--drive-s`·`--settle-s`·`--idle-probe-s` 는 sim 초다. 실시간 전용
`--physics-hz`·`--reader-hz`·`--wft-deadline-ms`·`--max-inflight` 는 `--lockstep` 과 함께 주면 거부하고,
lockstep 전용 옵션은 `--lockstep` 없이 주면 거부한다. 종료 코드는 실시간 모드와 같다(0 정상, 1 사용법/거부,
2 연결·사전 점검 실패(변경 없음), 3 JSON 저장 실패, 4 진입·준비 실패(복구됨), 5 중단(복구됨), **6 복구 실패
— MORAI 가 얼어 있을 수 있다, `morai_sync_restore`**).

<!-- 26.09.25 _ YSH _ 신규 절 — 동기 모드 비상 복구 도구 morai_sync_restore -->
## morai_sync_restore — 얼어 버린 MORAI 를 실시간으로 되돌리기

lockstep 클라이언트(`morai_actuate --lockstep` 또는 다른 무엇이든)가 동기 모드에서 죽으면(kill -9,
크래시, 원격 PC 전원) MORAI 는 스스로 아무것도 하지 않고 **얼어 있는 채로** 남는다. 이 도구가 되돌린다.

- **멱등**: 이미 실시간이면 `SetSynchronousMode` 를 보내지 않는다(읽기만 한다). 확인 플래그가 필요 없다 —
  되돌리기만 하고 **Ego 를 움직이는 호출(ControlVehicle, Tick, 기어)은 전혀 없다** (`test_actuate_scope`
  가 허용 RPC 를 검사: Get/SetSynchronousMode(UNSPECIFIED 만), GetAllActorsState, Get/SetVehicleControlMode
  (KEYBOARD 만)).
- 동작과 출력(단계마다 무엇을 했는지 찍고, 끝에 `did:` 한 줄):
  1. `GetSynchronousMode` (서버가 재시작 중이면 UNAVAILABLE — `--timeout-s` 동안 백오프 재시도)
  2. UNSPECIFIED 가 아니면 `SetSynchronousMode(UNSPECIFIED, 0)` 을 `GetSynchronousMode` 가 UNSPECIFIED 를
     읽을 때까지 `--timeout-s`(기본 10 s) 재시도. Result 는 출력만 하고 무시한다
  3. timestamp 가 벽시계 속도(0.5~1.5 배)로 흐르고 epoch 기반인지 `--verify-s`(기본 1 s) 확인
     (`ActorStates.timestamp` 만 보므로 `--ego` 가 틀려도 판정된다 — Ego 를 못 찾으면 4 에서만 알린다)
  4. Ego 제어 모드·기어·속도 출력. `--keyboard` 면 `SetVehicleControlMode(KEYBOARD)` 후 되읽기
- 종료 코드: 0 실시간이고 벽시계 속도로 흐름(복구했거나 원래 정상), 1 사용법, 2 MORAI 응답 없음(아무것도
  안 바꿈), 3 `--timeout-s` 뒤에도 UNSPECIFIED 아님, 4 UNSPECIFIED 지만 시간이 벽시계 속도로 안 흐름(일시정지?),
  5 `--keyboard` 인데 KEYBOARD 로 안 읽힘.

```bash
# 워크스페이스 루트, 깨끗한 셸 — 원격 MORAI 면 --host 10.10.10.1
env -i HOME=$HOME PATH=/usr/local/bin:/usr/bin:/bin TERM=xterm LANG=C.UTF-8 bash --noprofile --norc -c \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 run morai_grpc_vendor morai_sync_restore --host 127.0.0.1 --keyboard'
```

**그래도 MORAI 가 얼어 있으면** (종료 코드 3·4): MORAI UI 의 **Time Manager** 에서 동기 모드를 끄고
(실시간/Normal), 일시정지가 걸려 있으면 푼다. 그다음 Ego 의 제어 모드(KEYBOARD)와 기어(P)를 UI 에서 확인한다.
모드가 7(SYNCHRONOUS_MODE)로 남아 있으면 `--keyboard` 로 다시 실행하거나 UI 에서 바꾼다.

<!-- 26.09.25 _ YSH _ 신규 절 — 가짜 MORAI 서버 (노드 종단 시험용) -->

## fake_morai_server — 가짜 MORAI (시험 전용, 설치하지 않음)

`test/fake/fake_morai_server.cpp`. `BUILD_TESTING` 빌드에서만 `build/morai_grpc_vendor/fake_morai_server` 로 생기고
설치·ctest 대상이 아니다. 서버 스텁(생성 코드)이 필요해 이 패키지에 둔다. **포트 7789 는 거부한다.** 쓰는 곳은
`autoware_morai_interface/test/e2e/run_fake_morai_e2e.sh` (노드 전체 종단 시험, 그 README "가짜 MORAI 종단 시험").

흉내내는 것 — 로컬 MORAI 실측(morai-time-facts)에서 확인된 동작만:

| 항목 | 가짜의 동작 |
| --- | --- |
| 업데이트 | 50 회/s (20 ms 틱). `--skip-every N`(기본 97) 번째마다 한 틱을 건너뛰어 timestamp 가 2 프레임(40 ms) 뛴다 |
| `ActorStates.timestamp` | 시작 epoch µs + 20 ms × 틱 (벽시계와 같이 간다) |
| `WaitForTick` | 업데이트마다 한 번 돌아온다. 값은 다른 시계(steady µs + 상수), 반복 없음. 멈춤 동안은 돌아오지 않는다 |
| `GetAllActorsState` | NPC 2(원을 도는 차 `NPC_Car_1`, 오가는 보행자 `NPC_Ped_1`, 필터 준수) + `Ego`(vehicle_state 포함), 한 번에 |
| Ego 동역학 | 마지막 ControlVehicle 을 업데이트 사이에 유지해 적분. AUTO(3)·SYNCHRONOUS(7) + 기어 D/R 에서만. 조향은 타이어각 rad (±40° 클램프), 자전거 모델(wheel_base 3.0) |
| 에코 | `front_wheel_angle = −조향/최대`(Unity 부호, 정규화), `angular_velocity.y = −yaw rate`(world rad/s, 축 (−z, x, −y)) |
| 그 밖 | `GetVehicleSpec`(3.0 m, 40°), Get/Set 제어 모드(`--lie-mode-result` 면 Result 가 거짓 실패 — 되읽기 시험), `SetVehicleGear`, `SetTransform`(30 ms 뒤 적용), `SetVehicleTailLight`, `GetActorState`, `GetSynchronousMode`, `GetTimestamp`(프레임 시각) |

기록: RPC 별 개수·마지막 시각, 업데이트 사이 ControlVehicle 개수 분포(`hist`), `--events PATH` 면 JSONL(업데이트마다
위치·속도·timestamp, 모든 명령·설정 RPC 의 값).

제어 포트(`--control-port`, 기본 gRPC 포트 + 1, 127.0.0.1): 한 줄 명령 → 한 줄 JSON.

| 명령 | 효과 |
| --- | --- |
| `stats` | 위 기록 전부 (JSON) |
| `freeze S` | S 초 동안 업데이트 멈춤 — timestamp·상태 그대로, WaitForTick 안 돌아옴 (물리도 멈춤) |
| `drop S` | 서버를 닫고(연결 끊김) S 초 뒤 같은 포트로 다시 연다. 세계 상태는 이어진다 |
| `zero S` | S 초 동안 Ego 위치·자세·속도 전부 0 (`ego_is_valid` 무효 — 미스폰·리셋) |
| `mode N` / `gear N` | MORAI 쪽에서 제어 모드·기어를 바꾼다 (운영자 조작·재기동 흉내) |
| `quit` | 종료 |

```bash
build/morai_grpc_vendor/fake_morai_server --port 17871 --events /tmp/fake.jsonl --lie-mode-result &
build/morai_grpc_vendor/morai_probe monitor --port 17871 --seconds 3   # 읽기 전용으로 붙어 보기
python3 -c 'import socket;f=socket.create_connection(("127.0.0.1",17872)).makefile("rw");f.write("stats\n");f.flush();print(f.readline())'
```
