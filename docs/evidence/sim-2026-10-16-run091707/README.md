# 가제보 실습 스무째 회차 (full-1016-091707, 2026-10-16 09:17 ~ 09:24, sim1 새 arena)

측정·로그 자료를 먼저 적는다. 가제보 실습의 해석은 맨 끝 「참고: 가제보 실습 의견」에만 둔다(재범 지시 2026-10-16).

- 빌드·world: main `a051a6c`. 19회차 뒤 들어간 것은 #462(새 arena 머지), #470(고정 지도 TB-5 몸 자리), #472(TB-5 실행기 Pose 중단 고침)다. `src/`·`config/` diff 0줄.
- 기동: TB-6 `ARMED`(09:19:03), 손으로 한 조작 없음. 재난 선포는 재범 위임으로 09:19:32(`accepted: true`).
- 녹화: 09:19:32 ~ 09:23:43.

## 단계별 결과

| 단계 | 결과 | 시각 · 값 |
| --- | --- | --- |
| 1 | 통과 | |
| 2 | 통과 | 경계 2개 `COMPLETED`, 터널 1 통과 |
| 3 | 통과(장부 기준) | 09:21:07 `FAKE_TARGET published … map=(-2.501, 0.320) … dist=1.76`. 장부 `target_states` 1, `overhead_passages` 2(`logs/ledger_targets_passages.json`) |
| 4 | **실패** | TB-5 `ARMED`(09:22:21) 뒤 `task not issued: SUPPORT_APPROACH_NOT_FREE` 3회(09:22:27 ~ 09:23:17) |
| (TB-6 복귀) | **처음 성공** | 09:22:27 `Begin navigating from current location (-2.76, 2.00) to (-0.00, -0.00)` → 09:23:16 `Reached the goal!` → 조정기 09:23:17 `RESULT … outcome=COMPLETED reason=NAVIGATION_SUCCEEDED`. 정답 위치 TB-6 (0.954, 0.464), 출발 자리 (0.880, 0.721) 근처 |

## 고쳐진 것 (근거)

- #472: 2차 고정(09:21:11) 순간 TB-5 실행기에 `robot_b map is not the frozen map; not loaded` 줄은 나왔다. 그러나 `Assertion` 0, `task_input-14]: process has died` 0 이었다(`logs/robot_robot_b.log`). 그 뒤 `MAP_STATE robot_b … rev=2 quality=VALID`, RobotPose `VALID mission=(-0.420, 1.774)`(09:21:11.97), `START_ALIGN_ALIGNED`(09:21:18) 순서로 이어졌다. 16·17·18회차는 같은 순간 중단이었다.
- #463: TB-6 RETURN_HOME 이 끝까지 갔다(위 표). 14 ~ 19회차는 모두 `localization uncertain`·`Unable to get start pose`·C22 로 실패했다.

## 남은 막힘

- TB-5 접근 지점 `SUPPORT_APPROACH_NOT_FREE`: 15 ~ 20회차에서 TB-5 가 arm 된 회차 모두에서 났다. 계산과 근거는 15회차 자료 README 「TB-5 출동 막힘」에 있다. 인형 (−2.501, 0.320)과 TB-5 (−0.42, 1.77) 사이 직선 0.8 m 점이 흰 상자 안이다.

## 파일

| 파일 | 무엇 |
| --- | --- |
| `logs/*.log` | 회차 폴더 원문(색 코드 지움, 경로·PC 이름 바꿈) |
| `logs/ledger_targets_passages.json` | 장부 target·터널 행 |
| `maps/`, `track/` | 1·2차 고정 지도, 정답 위치·RobotPose·RTF |
| `video/` | 녹화 줄인 판, 녹화 시작 기준 223 s(TB-6 복귀 도착) 장면 |

## 참고: 가제보 실습 의견 (정확하지 않을 수 있음)

아래는 위 자료를 보고 짐작한 것이다. 확인하지 않은 해석이라 틀릴 수 있다.

- 이 빌드에서 4단계를 막는 것은 TB-5 접근 지점 고르기 하나로 보인다. 이것이 풀리면 네 단계를 모두 지날 수 있을 것 같다.
