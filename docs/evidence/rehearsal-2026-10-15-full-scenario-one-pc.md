# 2026-10-15 한 PC 전체 시나리오 리허설 기록 (방식 B, 로봇 없음) — 진행 중

`bash scripts/field/full-scenario-rehearsal/run.sh <install> <결과 폴더> 195` 를 19회 돌린 기록이다. 로봇 Pi·라이다·도크·OAK-D 감지기는 `sim_world.py`(2D 모의)이고, slam_toolbox·AMCL·start_align·Nav2 두 벌·실행기·watchdog·조정기·운영 노드·도킹 노드·알림음은 실제 노드다. **모의 입력 결과이며 실기 결과가 아니다.** 환경: ROS 2 Jazzy, 이 PC 1대, 로컬 Discovery Server 127.0.0.1, `ROS_DOMAIN_ID=195`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.

## 현재까지 확인 (19회차 기준)

| 단계 | 상태 |
| --- | --- |
| preflight, 단계별 기동·준비 판정, 실패 시 단계·사유 출력 | 확인 |
| TB-6 출발 자세 기록, SLAM(실시간 지도) | 확인 (SLAM lifecycle 응답 유실 시 단계 재시작 1회로 회복) |
| TB-5 seed(정답에 +0.08 m, −0.06 m, +0.05 rad 오차) → start_align ALIGNED | 확인 (약 7 s, 오차 보정) |
| Nav2 두 벌 active, 두 로봇 arm, 합성 DISASTER_SIGN 1회 → 조정기 DISASTER | 확인 |
| TB-6 frontier 자동 탐색, 다른 방까지 이동 | 확인 |
| 인형 관측 → 실제 target_locator 투영 → 조정기 TargetLocation·보고 PERSISTED | 확인 |
| TB-5 출동 → 접근 → 알림음 → 출구 대기 → TB-6 복귀 → 도킹 → FINISHED | **미확인**: 교통 관제가 TB-6 을 취소한 뒤 TB-6 재 arm 이 없어 멈춤(아래 8) |
| Ctrl+C·실패 시 두 로봇 안전 정지 명령과 프로세스 정리, 남은 프로세스 없음 | 확인 (`[보냄]` 줄, `남은 프로세스 그룹: 없음`) |

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

## 남은 일 (다음 회차)

- 8 다음 문제: 교통 관제가 TB-6 작업을 취소하면 실행기가 STOP_LATCHED 가 되어 비켜서기 작업을 못 받는다(`TRAFFIC_SUPPORT_WAITING_SCOUT_YIELD` 에서 멈춤, 19회차). 조정기가 교통 때문에 직접 취소한 TB-6 작업에 한해, 정지 확인된 CANCELED 뒤 기존 `task_arm` 을 다시 부르는 수정이 필요하다(미구현).
- 그 뒤 TB-5 접근·알림음·출구 대기, TB-6 탐색 완료·복귀·도킹, FINISHED 를 끝까지 확인하고 이 문서를 결과 표로 바꾼다.
- 부하(CPU·GPU·메모리) 기록, LOCALHOST(비교용)·다른 도메인 회차.
