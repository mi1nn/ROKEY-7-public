# 가제보 실습 스물한째 회차 (full-1016-095154, 2026-10-16 09:51 ~ 10:02, sim1 새 arena)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드: main `1c51d0e`(20회차 `a051a6c` 뒤 #467·#469 관제 웹, #471 터널 얇은 덮개). `src/`·`config/` diff 0줄.
- 재범 「실습 바로 시작하자」(09:4x). 첫 기동(09:46)은 run_full `언도킹 robot6 실패` 로 스스로 멈췄고, 두 번째(09:51)에 떴다.
- 화면: 재범 지시(「왼쪽 창에는 가제보 오른쪽 창에는 rviz」)로 run_full 을 `SIM_RVIZ=off` 로 띄웠다. 이 PC 화면 왼쪽에 Gazebo 화면 클라이언트, 오른쪽에 두 로봇 RViz(`logs/two_robots_screen.rviz`, 창 X 960)를 따로 띄웠다. Wayland 화면이라 이 회차는 화면 녹화가 없고, RTF 기록도 빠졌다.
- 관제 웹은 Tailscale 주소로 띄웠다(재범이 LOQ 에서 접속).
- TB-6 `ARMED` 09:53:45. 재난 선포 09:54:25(`accepted: true`).

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1 | 통과 | |
| 2 | 통과 | 경계 4개를 보냈다(첫 작업은 아래). TB-6 은 터널 1 을 지나 서쪽 끝 정답 위치 (2.9, 4.4) 근처까지 갔다 |
| 3 | **인형 없음** | `FAKE_TARGET published` 0. a116 경계 goal 이 (−1.07, 2.17)(터널 1 앞)으로 잡혀 거기서 4/4 회전했다. 15 ~ 18·20회차에 인형을 본 (−3.0, 1.97) 자리에서는 회전하지 않았다 |
| 4 | 실행 안 됨 | 인형 없음. 2차 고정 순간 TB-5 실행기 중단 0(`Assertion` 0), TB-5 `ALIGNED`(10:01:25) |
| (TB-6 복귀) | **실패(기한)** | 09:59:24 `Begin navigating from current location (-4.44, 2.10) to (-0.00, -0.00)` → 10:01:23 `CommandLease rejected: DEADLINE_EXPIRED … task expired` → 조정기 `RESULT … EXPIRED reason=DEADLINE_EXPIRED` → `SCOUT_RETURN_FAILED`. 그때 정답 위치 TB-6 (1.931, 1.104), 출발 자리 (0.880, 0.721)까지 약 1.1 m |

## 첫 탐색 작업 바로 취소

- 09:54:28.04 TB-6 실행기 `SEND … GoalPose(… x=-0.968, y=2.016 …)` → 09:54:28.08 `Begin navigating from current location (-0.35, -0.00) to (-0.97, 2.02)` → 09:54:28.13 `CANCEL … handle=True`.
- 조정기는 09:54:29 `RESULT … FAILED reason=GUARD_INVALID` 였다. 실행기 로그에 그 사이 `task guard rejected …` 이유 줄은 없다(`logs/robot_robot_a.log`).
- TB-6 RobotPose 는 09:54:27.52 에 처음 `VALID` 였다(`track/pose_gap.txt`).

## 그 밖의 자료

- 넘김: `AMCL_NOMOTION_REQUESTED`(09:59:13, 2.3 s) 뒤 위치가 나왔다. `frozen_loc_a.log` 에 `createLaserObject` 1줄이 있다(C22 없음).
- RETURN_HOME 은 09:57:57 ~ 09:59:24 사이에 generation 8 ~ 13 을 보냈고, 13 이 Nav2 까지 갔다.
- 프로세스별 부하는 `track/proc_load.txt` 다.

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 복귀는 길이 길어(서쪽 끝 → 터널 1 → 집) 작업 기한 안에 못 들어온 것으로 보인다. 20회차는 (−2.76, 2.00)에서 출발해 49 s 에 도착했다.
- 인형 미관측은 19회차와 같이 경계 목표 자리가 달라져 인형 쪽으로 돌지 않은 탓으로 보인다.
