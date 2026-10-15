# 2026-10-15 한 PC 전체 시나리오 리허설 기록 (방식 B, 로봇 없음)

`bash scripts/field/full-scenario-rehearsal/run.sh <install> <결과 폴더> 195` 를 26회 돌린 기록이다. 로봇 Pi·라이다·도크·OAK-D 감지기는 `sim_world.py`(2D 모의)이고, slam_toolbox·AMCL·start_align·Nav2 두 벌·실행기·watchdog·조정기·운영 노드·도킹 노드·알림음은 실제 노드다. **모의 입력 결과이며 실기 결과가 아니다.** 환경: ROS 2 Jazzy, 이 PC 1대(22 코어, 메모리 31 GB, RTX 4070 Laptop), 로컬 Discovery Server 127.0.0.1, `ROS_DOMAIN_ID=195`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, `--no-rviz`.

## 결과 표

| 단계 | 결과 | 모의/실제 | 회차 |
| --- | --- | --- | --- |
| preflight, 단계별 기동·준비 판정, 실패 시 단계·사유 출력 | 확인 | 실제 실행기 | 19 ~ 26 (22회차는 `[robots] not ready within 120 s` 로 단계·사유를 내고 멈춤) |
| TB-6 출발 자세 기록, SLAM(실시간 지도) | 확인 | 실제 slam_toolbox, 모의 라이다 | 19 ~ 24 |
| TB-5 seed(정답에 +0.08 m, −0.06 m, +0.05 rad 오차) → start_align ALIGNED | 확인 | 실제 AMCL·start_align | 19 ~ 24 |
| Nav2 두 벌 active, 두 로봇 arm, 합성 DISASTER_SIGN 1회 → 조정기 DISASTER | 확인 | 실제 | 19 ~ 24 |
| TB-6 frontier 자동 탐색 → `EXPLORATION_COMPLETE`(시간 제한 아님) | 확인 | 실제 조정기·Nav2, 모의 주행 | 21, 23 ~ 25 |
| TB-6 출발 자세로 복귀 → 도킹 요청 → `DOCKED` → `MISSION_FINISHED` | 확인 | 실제 조정기·도킹 노드, **모의 Create3 dock** | 21 (생존자 없음 분기) |
| 인형 관측 → 실제 target_locator 투영 → TargetLocation·보고 PERSISTED | 확인 | 모의 감지, 실제 투영·보고 | 19, 23 ~ 25 |
| 교통 관제: TB-5 출동 전 TB-6 취소 → `TRAFFIC_REARM` → 비켜서기 → TB-5 출동 | 확인 | 실제 조정기·실행기 | 25 |
| 막힌 접근점 → `APPROACH_POINT_MOVED`(0.80 m 유지) | 확인 | 실제 조정기 | 24, 25 |
| TB-5 접근 → 알림음 1회(`FOUND_ALERT_PLAYED`) → 15 s 대기 → 출구(TB-5 출발 자리) `AT_EXIT` 대기, 그 뒤 TB-5 작업 없음 | 확인 | 실제 조정기·Nav2·알림 노드(이 PC 스피커), 모의 주행 | 24, 25 |
| 한 회차에서 처음부터 인형 발견·TB-5 출구 대기·TB-6 도킹까지 `SUCCESS` | **미확인** — 구간별로는 위 줄들에서 모두 확인했으나 한 회차에 다 이어진 적은 없다(25·26회차 TB-6 복귀 실패) | 실제 실행기 | 25, 26 |
| Ctrl+C·실패 시 두 로봇 안전 정지 명령과 프로세스 정리, 남은 프로세스 없음 | 확인 (22회차 전에는 손자 프로세스가 남았음 → 아래 13) | 실제 | 20 ~ 24 |

회차 요약: 20 = frontier 기한 초과 뒤 TB-6 정지 고정(아래 12), 21 = `SUCCESS — NO_SURVIVOR_FOUND`(카메라가 인형을 못 봄, 아래 14), 22 = 앞 회차 잔여 노드로 TB-6 Nav2 활성화 실패(13), 23 = 접근점이 상자 안(15), 24 = 마지막 frontier FAILED 뒤 TB-6 정지 고정(16), 25 = TB-6 복귀 FAILED(아래 남은 위험 첫 줄), 26 = 탐색 경로가 달라 카메라가 인형을 못 봄(14) → TB-5 출발 자리 대기, TB-6 복귀가 1 m 옆 TB-5 앞에서 collision_monitor 감속·「Failed to make progress」 반복 끝에 EXPIRED → `FAILED — SCOUT_RETURN_FAILED`. 리허설은 사용자 결정으로 26회에서 멈췄다(2026-10-15).

## 찾아서 고친 것 (원인 → 조치)

1. slam_toolbox 는 서 있는 TB-6 의 위치를 첫 스캔에만 낸다 → 출발 자세 기록기(`live_map`)를 SLAM 앞에 띄움.
2. TB-5 Nav2 는 AMCL `map→odom` 없이 활성화되지 않음 → TB-5 위치 추정을 로봇 launch 앞에.
3. `start_align` 이 FAILED 를 기록하는 순간 죽음(rclpy: 한 호출 위치에서 로그 심각도 변경 불가) → 줄을 나눔. 방식 A 의 start_align 에도 있던 결함. 같은 패턴인 조정기 WATCH/MAP_FREEZE 응답 로그도 고침.
4. LOCALHOST 단순 discovery 에서 노드 약 30개·서비스 600개 규모일 때 일부 노드 쌍이 서로를 못 찾음(조정기 시계 서비스 ↔ 실행기) → 리허설은 로컬 Discovery Server, 실행기는 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 요구.
5. Ctrl+C 때 rclpy 신호 처리가 context 를 먼저 닫아 안전 정지 서비스 호출이 실패 → 실행기가 SIGINT 를 직접 처리.
6. 서 있는 TB-6 의 첫 스캔 공분산은 1.0 m²(자리표시) → MapState UNKNOWN, 첫 탐색 목표 없음 → 방식 B 에서만 slam_toolbox 가 정지 중에도 1 s 마다 스캔 처리(이동 임계 0), 마지막 SLAM 위치 반복 노드, TB-6 공분산 나이 900 s([제안]).
7. 교통 관제: (a) 멈춰 있는 TB-5 옆 frontier 를 영원히 기다림 → 그 frontier 를 건너뜀, (b) 비켜서기 지점이 Nav2 도착 허용오차(0.25 m) 안쪽이라 117회 반복 → 비켜서기 지점을 0.25 m 더 멀리.
8. RETURN_HOME(TB-5 출구·TB-6 복귀) 에 frame_ref 가 비어 실행기가 `INVALID_INPUT: native frame` 으로 거절 → 조정기가 목적지 지도 기준을 채움.
9. TB-6 의 SLAM 지도에 TB-5 몸체가 장애물로 남아 TB-5 출발 칸에서 경로가 없음 → TB-5 용 지도 중계(`self_clear_map`)와 조정기 TB-5 경로 검사에서 TB-5 출발 자리·현재 위치 주변 0.3 m 만 비움.
10. (모의 쪽) 빈 점구름 필드 누락으로 collision_monitor 종료, 관측 QoS 불일치, 라이다 해상도 → 고침. 실행기에 Nav2 lifecycle 감시 추가.
11. (19회차) 교통 관제가 TB-6 작업을 취소하면 실행기가 STOP_LATCHED 로 남아(자동 재 arm 은 COMPLETED 뒤만, OPS-231) 비켜서기를 못 받고 TB-5 가 `TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD` 에서 멈춤 → 조정기가 자기가 취소한 작업이 정지 확인된 CANCELED 로 끝나면 `<ns>/rescue_command_adapter/task_arm` 을 1 s 간격 최대 10회 부름(`TRAFFIC_REARM`). 운영자 정지·FAILED·리스 만료 뒤에는 하지 않는다.
12. (20회차) TB-6 frontier 작업이 기한을 넘겨(모의 벽에 걸려 Nav2 복구 실패) EXPIRED 로 끝난 뒤 같은 이유로 7분간 작업 없음 → `EXPLORATION_TIMEOUT` → 11 의 재 arm 을 TB-6 frontier 의 정지 확인된 EXPIRED 에도 적용.
13. (22회차) 실행기 정리가 launch 리더만 보고 끝나 그룹 안의 `/robot6` lifecycle_manager 가 남았고, 다음 회차에서 같은 이름의 관리자가 둘이 되어 TB-6 Nav2 가 활성화되지 않음 → 정리는 프로세스 그룹 전체가 사라질 때까지 INT → TERM → KILL. 리허설 스크립트는 같은 `ROS_DOMAIN_ID` 의 잔여 프로세스가 있으면 시작하지 않음.
14. (21회차) 8 m × 6 m 모의 세계에서 라이다가 문간에서 먼 방을 다 봐 frontier 가 사라졌고(`EXPLORATION_COMPLETE`), 카메라(2.5 m, ±0.6 rad)는 인형에 가까이 간 적이 없어 `NO_SURVIVOR_FOUND` 로 끝남 → 모의 인형을 첫 방(world `[3.0, 4.8]`)으로 옮김. **코드로 고친 것이 아니다**: 「라이다 지도 완료 = 카메라가 다 봤음」이 아니라는 현장 위험이다(아래 남은 위험).
15. (23회차) 인형 옆 상자 때문에 TB-5→인형 직선 위 접근점(0.8 m 앞)이 상자 안 → `SUPPORT_APPROACH_NOT_FREE` 로 영원히 대기 → 직선 점이 막히면 대상 둘레 16 방향의 같은 거리 점 중 빈 점(TB-5 에 가장 가까운)을 고름(`APPROACH_POINT_MOVED`). 하나도 없을 때만 기존처럼 보류.

16. (24회차) 마지막 frontier 가 FAILED(정지 확인)로 끝나 TB-6 가 STOP_LATCHED → `EXPLORATION_COMPLETE` 뒤 RETURN_HOME 이 나가지 못함 → frontier 는 어느 끝이든 시도한 것으로 버리므로(`_frontier_done`) EXPIRED·FAILED·UNREACHABLE 모두 재 arm. 접근·복귀 실패 뒤에는 여전히 손으로 `task_arm`.

## 부하 (21회차, 30 s 간격 표본, `--no-rviz`)

| 항목 | 값 | 비고 |
| --- | --- | --- |
| CPU | 사용 10 ~ 22 %(유휴 78 ~ 90 %), load1 4.3 ~ 8.2 | 22 코어. 데스크톱·다른 작업 포함 |
| 메모리 | 사용 13.3 ~ 14.3 GB / 31 GB | 시작 전 약 13 GB(다른 작업 포함) |
| GPU | 0 %, 13 MiB | 리허설은 감지기가 모의라 YOLO 가 돌지 않는다 |
| 경로 제어기 주기 | `/robot6/controller_server` 「Control loop missed its desired rate of 20 Hz … 9.04 Hz」 경고(20회차) | 순간 부하. 실기에서 다시 볼 것 |

YOLO GPU 사용(코드·설정으로 확인, 실행 미확인): OAK-D 감지(`rescue_perception/camera_utils.py`)는 `model.predict(..., device=values['device'])` 이고 기본값 `device='0'`(GPU 0). 이 PC 의 torch 2.14.1+cu130 에서 `torch.cuda.is_available()` 은 True [측정, 2026-10-15]. 웹캠 노드(`webcam_node.py`)는 `device` 를 넘기지 않아 ultralytics 기본 선택을 따른다. 실제 감지 노드를 띄운 상태의 GPU 사용률은 이 기록에 없다(실기 미확인).

## 도메인·discovery 점검

- 목적: 리허설 그래프가 다른 도메인과 섞이는지. 23회차(도메인 195, 로컬 Discovery Server 127.0.0.1:11889)가 도는 동안 같은 서버에 super client 로 붙어 `ros2 topic list` 를 도메인 195 와 196 에서 봤다 → **둘 다 같은 204개 토픽**이 보였다 [측정, 2026-10-15]. Discovery Server 에 함께 붙으면 `ROS_DOMAIN_ID` 만으로는 목록이 나뉘지 않는다는 뜻이다. 현장에서 그래프를 나누는 것은 Discovery Server(주소·포트)이고, 다른 팀·다른 시험과는 서버를 따로 쓴다.
- 실제 로봇 그래프(도메인 6, 현장 Discovery Server)와는 리허설이 현장 설정을 지우고 127.0.0.1 서버만 쓰므로 만나지 않는다(`run.sh`).
- LOCALHOST 단순 discovery 회차, 다른 도메인 전체 회차: **미실행**(2026-10-15 사용자 판단으로 생략). LOCALHOST 는 위 4 의 탐색 누락이 이미 기록되어 있다.

## 알림음

- 단위 시험으로 확인: 발견 완료(APPROACH_COMPLETE)에서만 1회, 중복 이벤트에서 반복 없음, 재생기 없음·실패 시 `FOUND_ALERT_NOT_PLAYED` 로그만 남기고 계속, `enabled:=false` 는 `FOUND_ALERT_DISABLED` 로그만(`tests/test_full_scenario.py`).
- 리허설: 24·25회차에서 TB-5 접근 완료 때 `FOUND_EVENT` → `FOUND_ALERT_PLAYED` 각 1회(같은 회차에 반복 없음). `--no-alert` 회차·장치 없는 회차는 미실행(단위 시험만).

## 남은 위험

- **TB-6 복귀 자리 옆의 TB-5**(26회차): 모의 배치에서 TB-5 출발 자리(=QR 없는 출구)가 TB-6 출발 자리에서 1.0 m 다. TB-5 가 거기 서 있으면 TB-6 복귀 끝에 collision_monitor 감속 구역에 걸려 진행이 막힐 수 있다(21회차는 통과, 26회차는 기한 초과). 현장에서는 TB-5 출구를 TB-6 출발 자리에서 더 멀리 두거나(절차서 사전 준비 3) 프로파일 `home_positions` 로 따로 준다. 0.6 m 기준(`traffic_min_separation_m`)만으로는 부족할 수 있다 [추정].
- **TB-6 가 장애물에 붙어 못 빠져나옴**(25회차): 도달 못 한 frontier(상자 옆 0.3 m) 뒤 TB-6 가 상자에 맞닿아 멈췄고, 복귀 RETURN_HOME 에서 Nav2 가 「collision ahead」·spin 「Collision Ahead」·「Controller patience exceeded」를 반복하다 FAILED → 실행기 `FAILED — SCOUT_RETURN_FAILED`(거짓 성공 없음). 이번 작업에서는 Nav2 복구 동작(뒤로 빠지기 등)을 고치지 않았다. 모의 세계의 충돌 처리(맞닿으면 그 자리에 멈춤)가 실제와 다를 수 있고, 실기에서는 Create3 범퍼·실제 주행으로 다시 봐야 한다.
- **카메라 탐색 범위**(14): AutoSLAM 완료는 라이다 지도의 frontier 소진이다. 넓은 공간에서 라이다가 먼 곳까지 보면 카메라가 가 보지 않은 곳이 남고, 그 자리의 인형은 못 찾은 채 `NO_SURVIVOR_FOUND` 로 끝날 수 있다. 실기에서 인형 위치를 바꿔 가며 확인해야 한다.
- 접근점 대체(15)는 직선 거리로 TB-5 에 가장 가까운 빈 점을 고른다. 벽 너머의 점일 수 있고, 그 경우 Nav2 경로가 길어지거나 실패한다(`APPROACH_FAILED_<사유>` 로 드러남).
- 자동 재 arm(11·12)은 조정기가 취소했거나 기한을 준 작업에만 한다. 같은 순간 운영자가 `task_stop` 을 누르면 두 경우가 같은 CANCELED 로 보여 구분되지 않는다(짧은 경합 창). 실행기의 arm 조건(정지 증거·Nav2 준비)은 그대로 적용된다.
- loop closure: `map_correction` 이 큰 보정을 조정기에 알리고 기록하지만, 이미 보낸 목표를 다시 계산하지는 않는다(리허설에서 보정 0회).
- 리허설은 모의 주행·모의 도크·모의 감지기다. 실제 라이다로 start_align 수렴, 실제 Create3 도킹, OAK-D 감지·GPU, Wi-Fi 대역폭은 확인하지 않았다(절차서 「실기에서 처음 확인할 것」).
