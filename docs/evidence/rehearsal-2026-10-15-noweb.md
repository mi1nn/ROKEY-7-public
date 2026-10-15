# 웹 없는 트랙 한 PC 리허설 기록 (2026-10-15)

## 한눈에

웹 관제 화면 없이 터미널 도구 두 개(`scripts/field/noweb/operator_command.py`, `status_watch.py`)만으로 한 회차를 처음부터 초기화까지 돌릴 수 있는지 봤다(실습 기록 B-5, 재범 지시 「실습에 관한 작업 모두」). 종류는 E3(이 저장소의 실제 노드, 가짜 로봇)이다. 실기는 NOT_RUN 이다.

| 회차 | 코드 | 결과 | watchdog 기동 래치 | 실행기 하트비트 가장 긴 간격(TB-6 / TB-5) |
| --- | --- | --- | --- | --- |
| 1 | main 559f87c + 이 PR 의 probe | 8단계 모두 PASSED | 0 | 재지 않음 |
| 2 | 같음 | 8단계 모두 PASSED | 0 | 0.122 s / 0.390 s |
| 3 | 같음 | 8단계 모두 PASSED | 0 | 0.074 s / 0.063 s |
| 4 | 같음 | 8단계 모두 PASSED | 0 | 0.101 s / 0.060 s |
| 5 | 같음 | 8단계 모두 PASSED | 0 | 0.065 s / 0.055 s |

- 로봇은 그래프에 없었다. 가짜 Nav2 는 목표를 모두 거절하므로 아무것도 움직이지 않는다.
- 3·4·5회차는 처음에 도메인 번호를 233~235 로 줘서 DDS 가 시작하지 못했다(Fast DDS 는 232 이하만 받는다). 41~43 으로 다시 돌린 결과만 적었다.

## 순서와 본 것

`REHEARSAL_PROBE=noweb bash scripts/field/wiring-rehearsal/run.sh <install> <out> SEARCHING <port> <domain>` 로 돌렸다. 배선 점검이 먼저 돌고(모두 PASSED), 그 뒤 `noweb_probe.py` 가 아래 순서로 진행한다. `status_watch.py` 는 처음부터 끝까지 떠 있으면서 `<out>/status.log` 에 1초마다 한 줄씩 쓴다.

| 단계 | 한 일 | 1회차에서 본 것 |
| --- | --- | --- |
| 1 | 두 실행기 상태 수신 | 둘 다 `NOT_READY`, `STOP_UNCONFIRMED`(기동 직후) |
| 2 | `refresh_state`, 두 실행기 `task_arm` | 둘 다 `True ARMED` |
| 3 | `operator_command.py DECLARE_DISASTER` | 종료 코드 0, 답 `accepted: true`. 상태 줄에 `임무 DISASTER/DISASTER 이유=SEARCH_QUEUED` |
| 4 | TB-6 작업 수락 | `TaskAdmission` `ACCEPTED` |
| 5 | 가짜 Nav2 가 목표 거절 | TB-6 `FINISHED`, `NOT_READY`, `stop_reason=GOAL_REJECTED` |
| 6 | 가짜 로봇 도킹(`dock_status` 를 docked 로) | 두 로봇 `DOCKED`, `STOPPED`, 작업 없음 |
| 7 | `operator_command.py RESET_RUN --note …` | 종료 코드 0, 답 `accepted: true`, 새 `run_id` |
| 8 | 초기화 확인 줄 | 상태 줄 끝에 `초기화 확인 조정기=아직 robot_a=아직 robot_b=아직` |

3단계와 8단계의 실제 상태 줄(1회차)은 아래와 같다.

```
19:50:06 회차 a640d26b | 임무 DISASTER/DISASTER 이유=SEARCH_QUEUED (0초 전) | robot_a(SCOUT) 준비=READY 이동=STOPPED 도킹=UNDOCKED 자세=있음 작업=없음 배터리=- (0초 전) | robot_b(SUPPORT) 준비=READY 이동=STOPPED 도킹=UNDOCKED 자세=없음 작업=없음 배터리=- (0초 전) | 지도 r0 (3초 전) | 지도 보관 꺼짐 — 고정 여부 모름 | 대상 없음
19:50:13 회차 212f80bd | 임무 기록 없음 | robot_a(SCOUT) 준비=NOT_READY 이동=STOPPED 도킹=DOCKED 자세=있음 작업=없음 배터리=- (0초 전) | robot_b(SUPPORT) 준비=NOT_READY 이동=STOPPED 도킹=DOCKED 자세=없음 작업=없음 배터리=- (0초 전) | 지도 상태 없음 | 지도 보관 꺼짐 — 고정 여부 모름 | 대상 없음 | 초기화 확인 조정기=아직 robot_a=아직 robot_b=아직
```

## 이 리허설에서 알게 된 것

- **초기화는 두 로봇이 도킹해 있어야 받아들여진다.** 운영 API 는 도킹하지 않은 로봇이 있으면 `ROBOT_NOT_DOCKED` 로 거절한다. 그래서 probe 는 6단계에서 가짜 로봇을 먼저 도킹시킨다. 현장에서도 `RESET_RUN` 앞에 두 로봇을 도킹시켜야 한다(런북 8단계와 같음).
- **초기화 직후 「초기화 확인」은 모두 「아직」이다.** 이 줄은 초기화 뒤 조정기와 각 로봇 실행기가 다시 켜졌는지를 운영 DB 로 본다. 이 리허설은 다시 켜기를 하지 않았으므로 「아직」이 맞다. 다시 켠 뒤 「다시 켜짐」으로 바뀌는지는 보지 않았다.
- 상태 줄의 `지도 보관 꺼짐 — 고정 여부 모름` 은 리허설의 운영 launch 가 지도 보관을 켜지 않아서다. 현장 운영 launch 에서는 다르게 나와야 한다.
- TB-5 는 고정 지도를 받기 전이라 `자세=없음` 이 맞다.

## watchdog 기동 래치

#342 리허설에서 watchdog 이 기동 직후 `EXECUTOR_HEARTBEAT_STALE` 로 래치되는 일이 22회 중 4회 있었다. 그때는 실행기 하트비트가 0.5 s 넘게 끊겼다. 이번에는 watchdog 동작을 바꾸지 않고 빈도와 하트비트 간격만 쟀다.

- 이 리허설 5회: 래치 0회.
- 같은 날 따로 잰 임무 리허설 22회(main 268f0e8 + 거절 로그 11회, #342 판 11회): 래치 0회, 실패 0회.
- 하트비트 간격은 실행기가 하트비트에 찍은 시각(`std_msgs/Header.stamp`) 사이의 간격이다. 받는 쪽 지연과 상관없이 실행기가 멈춘 시간을 보여 준다. 가장 긴 간격은 아래와 같다.
  - 임무 리허설 기준판 11회: 0.335 s(10회차)
  - 임무 리허설 #342 판 11회: 0.133 s
  - 이 리허설 4회: 0.390 s(2회차 TB-5, 첫 하트비트 0.2 s 뒤)
- 모두 watchdog 기준 0.5 s 아래였다. 가장 긴 간격은 대개 기동 직후 1초 안에 나왔다.
- 한계: 기록기는 리허설 시작 1초 뒤에 붙고, Discovery 로 연결되기 전에 나간 첫 하트비트 몇 개는 놓칠 수 있다. 래치가 났던 4회의 끊김(약 1.7 s)이 이번에 다시 나지 않은 이유는 모른다. 전체 시험을 같은 PC 에서 함께 돌린 시간대(임무 리허설 중간)에도 래치는 없었다.

## 무엇이 진짜이고 무엇이 가짜인가

| 부분 | 무엇 |
| --- | --- |
| 진짜 | 이 저장소의 운영(웹 없이, 운영 API 127.0.0.1:18765), 조정기, 두 로봇 실행기와 watchdog, 지도 고정, 통로 노드. `operator_command.py`, `status_watch.py` 는 저장소 그대로 |
| 가짜 | 로봇 Pi 토픽(odom, dock_status, scan, 카메라), TB-6 지도와 TF(평면), TB-5 AMCL 자세와 출발 정렬 `ALIGNED`, Nav2(목표를 모두 거절), lifecycle 관리자(active 로 답함). `scripts/field/wiring-rehearsal/feed.py`, `fake_graph.py` |
| 네트워크 | 회차마다 `fast-discovery-server` 하나(127.0.0.1), `ROS_DISCOVERY_SERVER` 를 지우고 그 주소만 넣음, 도메인 10 이상 |

## 이 PR 이 바꾼 것

- `scripts/field/wiring-rehearsal/noweb_probe.py` 를 새로 만들었다. `run.sh` 에 `REHEARSAL_PROBE=noweb` 를 더했다.
- `feed.py` 의 가짜 `dock_status` 는 기본이 언도킹이고, `docked` 를 켜면 도킹으로 바뀐다. 다른 probe 는 그대로 언도킹이다.

## 하지 않은 것

- 실기(E4) NOT_RUN.
- 초기화 뒤 조정기와 실행기를 다시 켜서 「다시 켜짐」을 확인하는 단계, 그리고 두 번째 회차.
- 지원 출동(`DISPATCH_SUPPORT`), 재확인(`REQUEST_RECHECK`), 구간 계속(`CONTINUE_SEGMENT`). 가짜 Nav2 가 목표를 거절해서 대상과 출동이 생기지 않는다.
- RViz(`two_robots.rviz`) 화면.
