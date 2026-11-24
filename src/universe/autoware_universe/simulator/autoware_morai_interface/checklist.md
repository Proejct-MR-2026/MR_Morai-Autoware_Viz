# checklist — MORAI 차량 인터페이스 패키지화

구 브리지 `Morai_Autoware_Bridge/scripts/main.py` (345줄, 워크스페이스 밖 수동 실행) 를
정식 Autoware 패키지로 승격하는 작업.

## Phase 1 — 벤더 격리 패키지

- [x] `morai/actor/actor.proto` transitive closure 산정 (15개 / 1406줄, protoc 로 검증)
- [x] proto 15개 바이트 그대로 벤더링 + `PROVENANCE.md` (upstream URL·SHA·라이선스 상태)
- [x] `--cpp_out` 만으로 빌드 성립 확인 (apt 설치 전, 코드생성·install·export 설계 검증)
- [x] pre-commit 제외 규칙 (13/15 파일이 개행 없이 끝나 fixer 가 재작성함)
- [x] `--grpc_out` + `ActorClient` (PIMPL, deadline, StatusCode SUCCESS=1 판정)
- [x] 격리 경계 검증 — `*.grpc.pb.h` 미설치, grpcpp 없이 헤더 `-Werror` 컴파일

## Phase 2 — 인터페이스 패키지

- [x] 노드 골격 — 파라미터, 재시도 타이머, 콜백 그룹 2개 (`-Werror` 통과 확인)
- [x] 상태 방향 (`StatusReporter` + `morai_to_autoware`) + 테스트 14
- [x] 명령 방향 (`CommandForwarder` + `autoware_to_morai`) + 테스트 11
- [x] 램프 병합 (`LampCommandCache`, hazard 우선) + 테스트 8
- [x] `control_mode_request` 서비스 (engage 가 시뮬을 AUTO_MODE 로) + 테스트 4

## Phase 3 — 배선

- [x] `vehicle_interface.launch.xml` 스텁 교체 + remap 11개 + `component_container_mt`
- [x] `e2e_simulator.launch.xml` 의 `launch_vehicle_interface` → true
- [x] `max_steer_angle` 0.60 placeholder → 0.632 (실측 풀락값)
- [x] 미사용 `morai_msgs` 삭제 (시뮬과 통신 안 되는 패키지였음)

## Phase 4 — 문서 / 환경

- [x] `autoware_morai_interface/README.md` (I/O, 부호 규약, 파라미터, 검증 절차)
- [x] `morai_grpc_vendor/README.md` + `PROVENANCE.md`
- [x] `ansible/roles/grpc` + `install_dev_env.yaml` 등록
- [x] `context-notes.md` (결정 근거)

## Phase 5 — 검증

- [x] 가짜 gRPC 서버로 종단간 검증 (remap·주기·변환값·명령·서비스·게이트)
- [x] 전체 워크스페이스 빌드
- [x] 전체 테스트
- [x] pre-commit 전체
- [x] code-review

## Phase 6 — sim 필요 (사용자 복귀 후)

- [ ] 조향 부호 4단계 검증 (README "검증" 절)
- [ ] 속도 단위 실측 (36 km/h → 10 m/s)
- [ ] 기어 P→R→N→D 순환
- [ ] 제어모드 키보드↔자동 전환
- [ ] 램프 좌/우/비상
- [ ] `max_steer_angle` 부팅 로그와 yaml 대조
- [ ] engage 후 폐루프 주행 한 바퀴
- [ ] 완주 확인 후 `main.py` 은퇴 처리

<!-- 26.09.24 _ YSH _ COLCON_IGNORE 2개(이 패키지, morai_grpc_vendor) 제거. gRPC C++ 1.30.2 / protobuf 3.12.4 설치 확인(dpkg). launch_vehicle_interface=false 는 유지 — 런타임 변화 없음. 이후 계획: ~/.claude/plans/zesty-sprouting-book.md -->
- [x] 26.09.24 _ YSH _ `COLCON_IGNORE` 제거 → MA 에서 빌드·테스트 (feat/morai-cpp-interface)

<!-- 26.09.25 _ YSH _ 신규 절 — 프레임 고정 엔진 (Task B, feat/cpp-engine). 상세는 context-notes.md "26.09.25 — 프레임 고정 엔진" -->

## Phase 7 — 프레임 고정 엔진 (26.09.25, main.py 대체 GT 모드)

- [x] `StatusReporter` / `CommandForwarder` 삭제 → `Engine` 스레드 (WaitForTick 트리거, 업데이트당 스냅샷·ControlVehicle 1 회)
- [x] `ILink` 추상 + `MoraiLinkClient` (링크 재생성), 가짜 링크로 엔진 gtest 31 개
- [x] 조향 명령 라디안 (÷max 제거), 조향 결손 보정·종방향 모드·stale 대체 명령 (main.py 식) + 골든
- [x] heading_rate·twist 를 world rad/s → body 회전 (`morai_coordinates.py` 골든)
- [x] GT: kinematic_state + TF(같은 stamp), TrackedObjects(uuid5 바이트 골든), /initialpose 텔레포트(스냅샷 확인), 측위 DIRECT 초기화
- [x] 기어 송신기 (main.py `_GearSender` 의미 + Result·되읽기), 모드 요청 지연 응답, 램프는 작업 스레드
- [x] 판정: Ego 없음/전 필드 0/중복/역행 건너뜀, 신선도·시계 오프셋·폴백·재연결 진단 (`/diagnostics`)
- [x] 파라미터 전부 코드 기본값 (main.py 이름 + `morai.*`), 런타임 변경은 main.py 가 콜백에서 읽던 것만
- [x] 단독 실행 파일 MultiThreadedExecutor, `launch/morai_interface_gt.launch.xml`, `config/morai_interface.gt.param.yaml`
- [x] 중복 writer 가드 (main.py 와 동시 실행 시 송신 차단), 종료 감속
- [x] 빌드(-Werror) · gtest 111 · 가짜 MORAI 로 노드 전체 스모크
- [ ] 로컬 MORAI + Autoware GT 모드 폐루프 (사용자) — `/diagnostics` update_rate ≈ sends_per_s ≈ 50, send_fail 0
- [ ] 조향 부호 4단계 재확인 (README "검증") — 명령 스케일이 바뀌었으므로 필수
<!-- 26.09.25 _ YSH _ 리뷰 반영: 진단 이름은 노드 이름(morai_bridge) 앞머리, 서브 PC 전 probe 확인 추가 -->
<!-- 26.09.25 _ YSH _ 최종 점검: morai_probe 옵션은 --host (--address 없음). 원문 첫 줄:
- [ ] 서브 PC MORAI + chrony — 먼저 `morai_probe all --address <IP>`(읽기 전용)로 timestamp ~50/s·WaitForTick 확인,
-->
- [ ] 서브 PC MORAI + chrony — 먼저 `morai_probe --host <IP> all`(읽기 전용)로 timestamp ~50/s·WaitForTick 확인,
      그다음 `morai_bridge: clock` 중앙값·`morai_bridge: link` 왕복 p50(>5 ms 면 WARN), 필요하면 keepalive 값 결정
<!-- - [ ] 서브 PC MORAI + chrony — `morai_interface: clock` 중앙값, 필요하면 keepalive 값 결정 -->
- [ ] lockstep(Tick) 드라이버 (다른 작업 트리에서) → `Engine::on_update` 에 연결

<!-- 26.09.25 _ YSH _ 신규 절 — 엔진 리뷰 반영 (상세는 context-notes.md "26.09.25 — 엔진 리뷰 반영") -->

## Phase 7a — 엔진 리뷰 반영 (26.09.25)

- [x] GT launch 노드 이름 `morai_bridge` (daq_publisher·mission_server·scenario_pose 호환), `ros2 run` 은 `-r __node:=morai_bridge`
- [x] 중복 writer 가드: 자기 publisher 하나만 제외 (cyclonedds Humble 은 get_gid ≠ graph GID — 실측), 1 s 주기, 첫 판정 전 송신 보류,
      차단 순간 ~0.3 s 감속, `duplicate_writer_guard` 파라미터 — domain 91 에서 같은 이름 두 인스턴스로 확인
- [x] timestamp 0·멈춤 → 내용 판정(수신 stamp), 역행 3 표본/0.5 s 새 기준
- [x] 막지 않는 WaitForTick → 폴백 (헛돌기 없음), 폴백 재확인 짧은 deadline·연속 3 회
- [x] Ego 1 s 없음 → 연결 단계 대신 복귀 때 스펙만 재조회 (모드 재지정 없음)
- [x] ControlVehicle deadline = max(설정, 4×왕복 p50) 상한 1 s (main.py cmd_timeout_for_rtt), 왕복 > 5 ms WARN
- [x] 모드 조회 0.5 s 넘게 실패 → ControlModeReport 생략, 재연결 때 모드 초기화 + 첫 조회 성공 때 AUTO 재지정 판단,
      요청 모드는 RPC 전에 기록, 모드 요청 전용 작업 스레드
- [x] 대소문자 대체 매치 Ego 를 actors 에서 제외 (자차 장애물 방지), `/clock` = 메시지 stamp, |오프셋| > 1 h 진단 ERROR
- [x] `actor_hold_s` 예약(효과 없음)으로 문서화, main.py 전용 파라미터 기동 WARN, README "main.py 와 다른 점" 보강
- [x] 빌드(-Werror) · gtest 136 · 가짜 MORAI 스모크(업데이트당 1 회 250/250)

<!-- 26.09.25 _ YSH _ 신규 절 — 가짜 MORAI 종단 시험 (Task C, 상세는 context-notes.md "26.09.25 — 가짜 MORAI 종단 시험") -->

## Phase 7b — 가짜 MORAI 종단 시험 (26.09.25)

- [x] `morai_grpc_vendor/test/fake/fake_morai_server.cpp` — 50 upd/s, epoch µs timestamp(가끔 2 프레임), WaitForTick(다른 시계),
      Ego+NPC 2, 명령 유지 적분, 스펙·모드·기어·텔레포트·램프, RPC 계수·JSONL, 제어 포트(freeze/drop/zero/mode) — 설치 안 함, 7789 거부
- [x] `test/e2e/run_fake_morai_e2e.sh` + `fake_morai_e2e.py` — 노드 전체를 Autoware 역할로 돌려 20 항목 판정 (20/20 PASS)
- [x] 찾은 결함 수정: 일시정지 중 WaitForTick deadline 을 실패로 세어 1 s 멈춤에 10 s 폴링 폴백 → 업데이트가 왔을 때만 센다
- [x] 찾은 결함 수정: 서버 재기동 뒤 명령 채널이 옛 연결로 READY → AUTO 재지정 'Socket closed' 1 회 실패로 영영 안 함
      → UNAVAILABLE 끊김도 채널 재생성 + 전송 실패한 재지정은 모드 조회마다 최대 3 회
- [x] gtest 136 → 140 (위 두 결함 회귀 시험)
- [ ] 로컬 MORAI + Autoware GT 폐루프 (사용자) — 그다음 서브 PC (위 Phase 7 항목)

<!-- 26.09.25 _ YSH _ 신규 절 — 명령 트리거 A/B (feat/cmd-timer-mode, 상세는 context-notes.md "26.09.25 — 명령 트리거 A/B") -->

## Phase 7c — 명령 트리거 A/B (26.09.25)

- [x] `command_trigger` (update 기본 | timer, 런타임) + `cmd_resend_hz` (100, 10~400, 정수 허용, 런타임) — 두 param yaml·스키마·GT launch 인자
- [x] timer = 전용 명령 스레드, main.py `_CmdResender` 규칙(절대 deadline, 늦으면 기준 재설정·late, 동기 1 개, 몰아 보내지 않음), 엔진 루프는 송신 안 함
- [x] 같은 명령 경로(`send_command(speed, …)`): 조향 라디안·보정(최신 표본 속도)·stale·NaN·forward·가드 감속·종료 감속
- [x] 연결 단계·스펙/첫 표본 전 무송신, 링크 끊김에 헛돌지 않음
- [x] 진단 `command`: command_trigger·cmd_resend_hz·sends_per_update·send_gap p50/p99/max·cmd_timer_late, 10 초 `제어 재송신` 줄 `목표 100 Hz`·지연재설정
- [x] gtest 140 → 156, 가짜 MORAI e2e 두 모드 21/21 (update 1.000/업데이트, timer 100.00/s·2.026/업데이트)
<!-- 26.09.25 _ YSH _ launch 인자 제거 반영, GT yaml 기본을 timer 100 Hz 로. 원문:
- [ ] 로컬 MORAI 기본 50 Hz + Autoware 폐루프 A/B (사용자): update vs `command_trigger:=timer cmd_resend_hz:=100` — MORAI 적용 throttle 0↔on 빈도·제동 페달·정지 위치
-->
<!-- 26.09.26 _ YSH _ A/B 결과 반영. 원문:
- [ ] 로컬 MORAI 기본 50 Hz + Autoware 폐루프 A/B (사용자): timer 100 Hz(GT yaml 기본) vs update(`ros2 param set /morai_bridge command_trigger update`) — MORAI 적용 throttle 0↔on 빈도·제동 페달·정지 위치
-->
- [x] 로컬 MORAI 기본 50 Hz + Autoware 폐루프 A/B (사용자): update(A, 20260925-232301) 목표 초과 15.9 m·실현 0.46 배 vs timer 100 Hz(C, 20260925-235746) 1.35 m·0.67 배. Autoware 입력 주기는 둘 다 41.5 Hz 로 같았다 — 차이는 MORAI 로 보내는 명령 횟수. C 는 저속 강제동에서 MORAI 내부 브레이크↔스로틀 180 ms 떨림

## 26.09.26 _ YSH _ 발행 주기 통일 (CARLA·simple_planning_simulator 방식)

- [x] 제어 모드 조회를 엔진 루프에서 전용 스레드(10 Hz)로 — 엔진 루프 안 동기 조회가 MORAI 다음 프레임까지 막혀 5 업데이트마다 1 개를 놓쳤다 (41.5 Hz). 세대(mode_epoch_)로 조회 도중 재연결·AUTO 설정과의 경쟁을 막고, AUTO 재지정은 새 조회 결과마다 엔진 스레드가 판단
- [x] 차량 리포트 전부(velocity·steering·gear·control_mode·turn·hazard)와 kinematic_state·TF 를 업데이트마다 같은 stamp 로, TrackedObjects 는 MORAI 시각 100 ms 스케줄(10 Hz)
- [x] gtest 156 → 163 (objects 10 Hz — 업데이트율 4 가지·무작위 흔들림·정체 뒤 몰림 없음·재기준, 조회 중 끊김·AUTO 재지정과의 경쟁, 연결 단계 조회 없음, 느린 모드 조회가 업데이트를 잃지 않음), colcon test 436/436, 가짜 MORAI e2e timer·update 각 21/21 (8 토픽 = 업데이트율·같은 stamp, objects 9.99~10.00 Hz)
- [x] 반박 리뷰 (3 방향 + 확인) 반영: 조회 스레드의 실행 중 확인을 세대 읽기와 같은 락 안으로, objects 최소 간격 80 ms 보장(늦은 표본은 격자를 민다), 시각 재기준·판정 전환 때 objects 스케줄 새로
- [ ] 실제 MORAI 녹화 (사용자): 상태 토픽이 MORAI 업데이트율(~50 Hz)·MORAI 고유 timestamp 대비 발행 ≥ 97 %·objects 10 Hz·control_mode_age ≤ ~150 ms
- [ ] 명령 주기 비교 (사용자 운전, 같은 코스): `ros2 param set /morai_bridge cmd_resend_hz 50|100|150|200` — 실현 비율·목표 초과·저속 떨림·횡오차

## 26.09.26 _ YSH _ DAQ 를 브리지 노드 안으로 (daq_publisher.py 대체)

- [x] 변환 3 개 (to_daq_vehicle_state·to_daq_acceleration·to_daq_tracked_object) — daq_publisher.py 와 비트 동일 (golden)
- [x] DaqSampler 전용 스레드 100 Hz ZOH, 엔진 훅 on_sample, 게이트 /daq/run_state (latched, 이름 맞는 idle 만), 런 요약 /daq/run_summary, 자동 종료, 다른 writer 양보
- [x] 반박 리뷰 반영: 이름 다른 idle 무시·구독 depth 10, pre-shutdown 콜백으로 'shutdown' 요약 전달, 기동 첫 판정 전 조용한 양보
- [x] gtest 467, 가짜 MORAI e2e timer·update 각 27/27 (D0~D5), cue_server 순서 녹화 bag → daq_export·daq_verify DAQ 항목 통과
- [x] Bridge `feat/daq-in-node` 87eff15: cue_server 녹화 구간 게이트·run JSON `daq`, daq_publisher 중복 거부, DAQ.md
- [ ] 실제 MORAI + cue_sim 한 런 (사용자 승인 후): daq_verify PASS, DAQ_ON flowing, DAQ_OFF state_hz ~100

