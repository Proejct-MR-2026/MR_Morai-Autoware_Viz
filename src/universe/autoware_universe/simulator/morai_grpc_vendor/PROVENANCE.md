# Provenance of `proto/`

Everything under `proto/` is **third-party material reproduced verbatim**. It is not
Autoware code and is not covered by the Apache-2.0 license that applies to the rest of
this package.

## Upstream

<!-- 26.09.24 _ YSH _ Simulation/Simulator closure 9개 추가 반입 행 추가 (sensor 7개는 이전 커밋에서 들어왔으나 이 문서에 누락돼 있었음) -->
| | |
|---|---|
| Repository | `http://172.16.1.20:8081/morai-interface/morai_standard_proto` |
| Commit | `f38b760bb2671afc2f2e565a7fa531e358b10e47` |
| Commit date | 2023-11-03 17:08:21 +0900 |
| Vendored on | 2026-08-10 |
| Vendored from | `Morai_Autoware_Bridge/scripts/lib/grpc/morai_standard_proto` (git submodule of the MORAI bridge repo) |
| Extended on | 2026-09-24 — +9 files (Simulation / Simulator closure), same bridge copy. That directory is plain files since bridge commit `9c079f9` (2026-08-10) and unchanged since; the 22 files already here were re-checked byte-identical against it with `cmp`. The upstream SHA itself could not be re-verified (no submodule metadata left in the bridge copy). |

## License status

<!-- 26.09.24 _ YSH _ 파일 수 15 -> 31 (추가 9개도 저작권·라이선스 표기 없음, grep 확인)
**No license file exists in the upstream repository**, and none of the 15 vendored
`.proto` files carries a copyright or license notice.
-->
**No license file exists in the upstream repository**, and none of the 31 vendored
`.proto` files carries a copyright or license notice. The upstream README describes the
repository as MORAI's internal interface definitions, shared with named customers.

The files are therefore treated as **proprietary, all rights reserved by MORAI**, and are
declared in `package.xml` as `LicenseRef-MORAI-Proprietary`. They are vendored here only
so that this workspace can talk to a MORAI simulator that the user is licensed to run.

Consequences that the rest of this package is designed around:

- The bytes are **not modified**. `.pre-commit-config.yaml` at the repository root
  excludes this directory so that `end-of-file-fixer` and friends cannot rewrite them.
  (Several files, e.g. `common/type.proto`, deliberately end without a trailing newline —
  that is upstream's state, not an error.)
- Generated code is **not committed**. It is produced into the colcon build directory.
- The generated gRPC stubs are **not installed**, and `actor_client.hpp` is a PIMPL, so no
  MORAI-derived header leaks into `autoware_morai_interface`.
<!-- 26.09.24 _ YSH _ morai_probe 도 같은 경계 안에 있음을 명시 -->
- The read-only tool `morai_probe` (`tools/`) links the Simulation / Simulator / Actor
  stubs **privately inside this package**; it is an executable, not an exported target.

<!-- 26.09.24 _ YSH _ 루트 .pre-commit-config.yaml 주석의 파일 수가 낡았음을 기록 (VEN 밖 파일이라 이번 단계에서는 고치지 않음) -->
Known stale comment: the exclude comment in the repository-root `.pre-commit-config.yaml`
still says end-of-file-fixer "would silently change all 15"; there are now 31 files. The
exclude pattern itself (`^src/universe/autoware_universe/simulator/morai_grpc_vendor/proto/`)
covers all 31, including `simulation/` and `simulator/`, so only the number is out of date.

## Why a copy and not a git submodule

The upstream host `172.16.1.20` is a **private LAN address**. A submodule would be
unclonable from any machine outside that network — CI, a colleague's laptop, a fresh
install — and `git submodule update` would hard-fail. A dangling pointer to an
unreachable host is worse provenance than a copy plus this file. The autoware tree is
also a single flat git repository with no existing submodules.

<!-- 26.09.24 _ YSH _ 범위를 Actor 단독 -> Actor·Sensor·Simulation·Simulator 4개 서비스 closure 로 갱신 (15 -> 31개, 1406 -> 2068줄)
## Scope: why 15 files and not 50

Only the `Actor` service is used. The 15 files here are the exact transitive import
closure of `morai/actor/actor.proto`, verified with:

```bash
cd proto
protoc -I. --descriptor_set_out=/dev/null --include_imports morai/actor/actor.proto   # exit 0
```

Manifest (1406 lines total):

```
morai/actor/actor.proto                        service Actor
morai/actor/actor_control.proto                VehicleCtrlCmd
morai/actor/actor_enum.proto                   GearMode, VehicleControlMode, VehicleTurnSignal, LongCmdType
morai/actor/actor_get.proto                    ActorState, VehicleState, VehicleSpec, CurVehicleControlMode
morai/actor/actor_network.proto                (closure only)
morai/actor/actor_set.proto                    VehicleGear, VehicleTailLight, VehicleControlModeParam
morai/actor/actor_spawn.proto                  (closure only)
morai/common/enum.proto                        StatusCode, ObjectType
morai/common/object_identifier.proto           ObjectIdentifier
morai/common/object_info.proto                 ObjectInfo
morai/common/resource_identifier.proto         (closure only)
morai/common/result.proto                      Result
morai/common/type.proto                        Vector3, Transform, StringValue, ...
morai/infrastructure/infrastructure_enum.proto (closure only)
morai/map/link_info.proto                      (closure only)
```
-->
## Scope: why 31 files and not 50

Four services are used: `Actor`, `Sensor`, `Simulation` and `Simulator` (the last two
only by the read-only `morai_probe` tool, for GetSynchronousMode / WaitForTick /
GetTimestamp). The 31 files here are the exact transitive import closure of those four
service files, verified with:

```bash
cd proto
protoc -I. --descriptor_set_out=/dev/null --include_imports \
  morai/actor/actor.proto morai/sensor/sensor.proto \
  morai/simulation/simulation.proto morai/simulator/simulator.proto   # exit 0
```

Note the upstream package name `morai_sim_api.simualtor` (sic) in `simulator/*.proto`;
the generated C++ namespace is therefore `morai_sim_api::simualtor`.

Manifest (2068 lines total):

```text
morai/actor/actor.proto                        service Actor
morai/actor/actor_control.proto                VehicleCtrlCmd
morai/actor/actor_enum.proto                   GearMode, VehicleControlMode, VehicleTurnSignal, LongCmdType
morai/actor/actor_get.proto                    ActorState, ActorStates, VehicleState, VehicleSpec, CurVehicleControlMode
morai/actor/actor_network.proto                (closure only)
morai/actor/actor_set.proto                    VehicleGear, VehicleTailLight, VehicleControlModeParam
morai/actor/actor_spawn.proto                  (closure only)
morai/common/enum.proto                        StatusCode, ObjectType
morai/common/object_identifier.proto           ObjectIdentifier
morai/common/object_info.proto                 ObjectInfo
morai/common/resource_identifier.proto         (closure only)
morai/common/result.proto                      Result
morai/common/type.proto                        Vector3, Transform, StringValue, Int64Value, Empty, ...
morai/infrastructure/infrastructure_enum.proto (closure only)
morai/map/link_info.proto                      (closure only)
morai/sensor/add_sensor_param.proto            AddSensorParam, AddSensorParamList
morai/sensor/sensor.proto                      service Sensor
morai/sensor/sensor_data.proto                 SensorData, ObjectStatus, GroundTruth
morai/sensor/sensor_data_save_config.proto     SensorDataSaveConfig
morai/sensor/sensor_enum.proto                 SensorType, GTShapeType, GTObstacleObjType
morai/sensor/sensor_info.proto                 SensorInfo, SensorInfos
morai/sensor/sensor_settings.proto             SensorSetting, SensorSettingList, ...
morai/simulation/resim.proto                   (closure only)
morai/simulation/simulation.proto              service Simulation
morai/simulation/simulation_enum.proto         SyncModeType, SimulationModeType, ...
morai/simulation/start_param.proto             (closure only)
morai/simulation/sync_mode.proto               SyncMode, SyncTimestamp
morai/simulator/available_object.proto         (closure only)
morai/simulator/category_obstacles.proto       (closure only)
morai/simulator/rendering.proto                (closure only)
morai/simulator/simulator.proto                service Simulator
```

## Re-vendoring

From a machine with access to the MORAI LAN:

```bash
git clone http://172.16.1.20:8081/morai-interface/morai_standard_proto /tmp/msp
cd /tmp/msp && git checkout <new-sha>

DST=<this-package>/proto
# 26.09.24 _ YSH _ 목록을 31개 전체로 확장 (sensor 7 + simulation 5 + simulator 4 추가), 복사 후 cmp 로 바이트 동일 확인
# for f in actor/actor.proto actor/actor_control.proto actor/actor_enum.proto \
#          actor/actor_get.proto actor/actor_network.proto actor/actor_set.proto \
#          actor/actor_spawn.proto common/enum.proto common/object_identifier.proto \
#          common/object_info.proto common/resource_identifier.proto common/result.proto \
#          common/type.proto infrastructure/infrastructure_enum.proto map/link_info.proto; do
#   cp -p "/tmp/msp/morai/$f" "$DST/morai/$f"
# done
for f in actor/actor.proto actor/actor_control.proto actor/actor_enum.proto \
         actor/actor_get.proto actor/actor_network.proto actor/actor_set.proto \
         actor/actor_spawn.proto common/enum.proto common/object_identifier.proto \
         common/object_info.proto common/resource_identifier.proto common/result.proto \
         common/type.proto infrastructure/infrastructure_enum.proto map/link_info.proto \
         sensor/add_sensor_param.proto sensor/sensor.proto sensor/sensor_data.proto \
         sensor/sensor_data_save_config.proto sensor/sensor_enum.proto \
         sensor/sensor_info.proto sensor/sensor_settings.proto \
         simulation/resim.proto simulation/simulation.proto simulation/simulation_enum.proto \
         simulation/start_param.proto simulation/sync_mode.proto \
         simulator/available_object.proto simulator/category_obstacles.proto \
         simulator/rendering.proto simulator/simulator.proto; do
  mkdir -p "$(dirname "$DST/morai/$f")"
  cp -p "/tmp/msp/morai/$f" "$DST/morai/$f" && cmp "/tmp/msp/morai/$f" "$DST/morai/$f"
done
```

Then re-run the closure check above — if it fails, upstream added an import and the file
list in this document and in `CMakeLists.txt` must both be extended. Update the commit
SHA and date in the table at the top.
