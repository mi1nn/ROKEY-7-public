# 시뮬레이션 기록: PassageStop 이 도크 앞에서 TB-6 을 세움 (2026-10-15)

## 한눈에

한 PC 시뮬레이션 회차(런북 3~7단계, 현장 파일 값 그대로)에서 재난 선포 뒤 TB-6 의 탐색 작업이 120초 동안 움직이지 못하고 만료됐다. 깊이 장애물 정지 `PassageStop`(현장 파일 `robot.passage.*`)이 **TB-6 앞 0.9~1.0 m 의 TB-5 도크**를 장애물로 잡았고, collision_monitor 의 `stop` 은 회전까지 0 으로 만들어 TB-6 이 돌아서 빠져나가지도 못했다.

- 높이 띠가 0.05~0.46 m 라서 도크(약 0.1 m)뿐 아니라 실습장 MDF 벽(0.41 m [측정, 재범 2026-10-15])도 앞 0.4~1.0 m 안에 오면 같은 정지가 난다.
- 시뮬레이션 쪽 오류가 아니라 현장 값의 설계 결함으로 본다(작전 결정 2026-10-15). 실기 NOT_RUN 항목이라 실습장에서도 같은 일이 날 수 있다. 설계 수정은 통합-로봇 자리, 값 변경은 등급 4(재범 승인).
- 이 회차 앞부분의 만료 한 번은 원인이 다르다(2절). 시뮬레이션 중계의 시각 짝 문제였고, 고친 뒤의 재현만 3절에 적는다.

## 1. 구성

- 코드: main `97f2cef` 빌드 + 시뮬레이션 작업 브랜치(`scripts/sim/` 만, `src/` 무수정). 현장 파일 `config/field/search_area.field.json` 값을 그대로 썼다.
- 한 PC: Gazebo arena world(평면도) + TB-6·TB-5(시뮬레이션 도메인) → `scripts/sim/wall_clock_relay.py`(벽시계·프레임·카메라 이름) → 우리 노드(현장 도메인): 운영·관제 웹, `tb6_slam`, `robot.launch` 두 벌, `map_freeze`, `frozen_localization`(TB-5), `passage_points`·`overhead_passage` 두 벌, 조정기.
- 두 로봇은 도면 자리의 도크에서 출발했고, 런북대로 지도 고정 노드를 띄운 뒤 언도킹했다.
- 현장 파일 passage 값: `passage.min_height_m` 0.05, `passage.max_height_m` 0.46, `passage.front_m` [0.4, 1.0], `passage.half_width_m` 0.2, `passage.min_points` 5, action `stop`(`field_launch.nav2_with_passage`).

## 2. 원인이 다른 첫 만료 (시뮬레이션 중계, 고침)

- 22:14 `SEND SEARCH_AREA task=27359eb2…` → 22:16 `RESULT … outcome=EXPIRED`.
- 이때 collision_monitor 는 `Robot to stop due to invalid source` 였다. depth_image_proc 이 깊이와 camera_info 를 exact stamp 로 짝짓는데, 중계가 메시지마다 따로 벽시계를 찍어 짝이 0쌍이었다(`Synchronized pairs: 0`). 깊이 점이 없으니 collision_monitor 가 정지했다.
- 같은 시뮬레이션 stamp 에 같은 벽시계 stamp 를 주도록 중계를 고친 뒤 짝 8쌍/10초, 깊이 점 약 2 Hz. 이 만료는 현장 값과 관계없다.

## 3. 재현 (중계를 고친 뒤)

### 3.1 로그 (시각은 벽시계 ns, 같은 회차)

조정기:

```text
SEND SEARCH_AREA task=8afa658d-ac2a-4f76-95da-442bb94dd0cc robot=robot_a generation=1
ADMISSION 8afa658d-ac2a-4f76-95da-442bb94dd0cc -> True
GOAL task=8afa658d-ac2a-4f76-95da-442bb94dd0cc accepted=True (ACTIVE only from task_admission)
RESULT task=8afa658d-ac2a-4f76-95da-442bb94dd0cc outcome=EXPIRED released=True
```

TB-6 실행기(`robot.launch`, task_input):

```text
[1792070234.866875802] SEND aeeef9b0-4a42-4feb-ae0f-aed8f6aff778 GoalPose(frame_id='map', x=-0.075, y=1.376, ...)
[1792070342.989819121] PASSAGE_BLOCKED polygon='PassageStop' action=1
[1792070345.698751683] PASSAGE_RELEASED polygon='invalid source' action=1
[1792070345.779838044] PASSAGE_BLOCKED polygon='PassageStop' action=1
[1792070353.236514595] CommandLease rejected: DEADLINE_EXPIRED: task expired or within clock uncertainty
[1792070354.866975309] CANCEL aeeef9b0-4a42-4feb-ae0f-aed8f6aff778 handle=True
```

- `SEND` 에서 `CANCEL` 까지 120.0 s = 작업 시간(`coordinator.task_duration_s` 120). 그동안 TB-6 Nav2 는 `Failed to make progress` 를 냈다.
- 같은 시각 명령 사슬을 셌다(10초): `cmd_vel_smoothed` 97건 → `cmd_vel_watchdog_in` 1건(0, 0) → `cmd_vel` 0건. collision_monitor 뒤에서 끊겼다.
- 정지가 풀린 3 s(`PASSAGE_RELEASED polygon='invalid source'`)는 깊이 점이 늦게 온 순간이다. 그때도 「출처 없음」 정지라 움직이지 않았다.

### 3.2 위치 (확인용 Gazebo 정답 위치, 로봇 노드에는 주지 않음)

| | arena x (m) | arena y (m) | yaw |
| --- | --- | --- | --- |
| TB-6 (만료 직후) | 1.712 | 0.671 | -29.6° |
| TB-5 | 2.700 | 0.722 | 89.5° |
| TB-5 도크(출발 자리 + 앞 0.157 m) | 2.70 | 0.213 | — |

- TB-6 에서 TB-5 도크까지: 거리 약 1.09 m, 진행 방향 기준 약 +5° [계산]. 도크 앞면은 약 0.9~1.0 m 앞이다.
- arena 원점은 로봇 구역 남서쪽 모서리, x 동쪽, y 북쪽(평면도 1 unit = 1 cm).

### 3.3 깊이 점 한 장면 (`/robot6/passage_points`, 같은 시각)

```text
frame oakd_rgb_camera_optical_frame n 49176
base_link min/max [ 0.39 -0.45  0.  ] [1.82 1.04 0.41]
정지 상자(x 0.4~1.0, |y| ≤ 0.2) 안, 높이 0.05~0.46 m: 1343점, 가장 가까운 x 0.91 m
```

- `min_points` 5 를 크게 넘는다. 도크 높이(약 0.1 m)는 띠 안이다.
- 높이 상한 0.46 m 이 벽(0.41 m)보다 높아, 벽이 앞 1 m 안에 오면 같은 정지가 난다. 실습장 로봇 구역 폭은 2.01 m 다.

## 4. 해석

- `PassageStop` 은 터널(머리 위 통로) 앞에서 높이를 판단할 때까지 멈추려는 장치다([field_launch.py](../../src/rescue_bringup/rescue_bringup/field_launch.py) `nav2_with_passage`, OPS-137·OPS-189). 높이 띠 아래쪽이 라이다 평면(0.23 m) 아래까지 내려가 있어, 라이다가 이미 보는 낮은 장애물과 벽도 정지 대상이 된다.
- collision_monitor `stop` 은 선속도와 각속도를 모두 0 으로 만든다. 정지 상자에 점이 남아 있는 한 회전도 못 하므로, 경로 계획이 돌아가는 길을 내도 빠져나가지 못한다.
- 실행기는 작업 시간이 다할 때까지 기다리고(만료), 조정기는 다음 탐색 경계를 고른다. 같은 자리면 같은 정지가 되풀이된다.

## 5. 후속

| 무엇 | 누가 | 상태 |
| --- | --- | --- |
| PassageStop 높이 띠·동작(stop/회전 허용) 설계 | 통합-로봇 | 미정 |
| 현장 파일 값 변경 | 재범 승인(등급 4) | 미정 |
| 실습장에서 같은 상황 확인(도크·벽 앞 1 m) | 실기 | NOT_RUN |
| 시뮬레이션 선택 인자 `SIM_PASSAGE=off`(사본만, 기동 로그에 「현장 값과 다름」) | 가제보 실습 | 시뮬레이션 실행 스크립트 PR 에서 |

## 검증 범위

- 확인한 것: 위 로그 원문, 명령 사슬 개수, 깊이 점 한 장면, Gazebo 정답 위치(확인용).
- 확인하지 않은 것: 실기, 다른 위치·자세에서의 재현 횟수, 정지 상자 안 점이 정확히 도크인지의 점별 대조(위치 계산으로만 맞춤).
