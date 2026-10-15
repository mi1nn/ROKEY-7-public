# 웹 없이 하는 회차의 도구

관제 웹 없이 RViz 와 터미널만으로 실기 회차를 돌릴 때 쓴다(재범 결정 2026-10-15). 순서는 [현장 실행 안내 「웹 없이 하기」](../../../docs/process/field-runbook.md#웹-없이-하기) 절에 있다.

| 파일 | 하는 일 | 어디서 |
| --- | --- | --- |
| `operator_command.py` | 관제 웹 버튼 대신 운영 API 로 운영자 명령과 초기화를 보낸다 | 운영 PC |
| `status_watch.py` | 관제 웹 카드 대신 운영 DB 를 읽어 1초마다 상태 한 줄을 쓴다 | 운영 PC |
| `two_robots.rviz` | 두 로봇의 지도, 경로, 위치를 보는 RViz 화면 | RViz 를 띄울 PC |
| `tf_merge.py` | 두 로봇의 TF 와 라이다를 프레임에 이름공간을 붙여 `/tf` 하나로 합친다. `two_robots.rviz` 가 두 로봇을 같이 그리게 한다 | RViz 를 띄울 PC |
| `target_markers.py` | 인형(대상)과 출구 QR 위치를 RViz 가 그릴 수 있는 표시(MarkerArray)로 바꿔 낸다 | RViz 를 띄울 PC |

네 도구 모두 로봇을 움직이지 않는다. 앞의 두 도구는 표준 라이브러리와 이 저장소의 `rescue_operations` 만 쓰고 ROS 가 필요 없다. `target_markers.py` 는 ROS 와 이 저장소의 `rescue_interfaces` 가 필요하다. PC 이름과 주소는 인자로 받는다.

## `operator_command.py`

```bash
python3 scripts/field/noweb/operator_command.py --port <api 포트> --operator <운영자 이름> DECLARE_DISASTER
python3 scripts/field/noweb/operator_command.py --port <api 포트> --operator <운영자 이름> RESET_RUN --note <메모>
python3 scripts/field/noweb/operator_command.py --port <api 포트> --operator <운영자 이름> REQUEST_RECHECK --target-id <대상 UUID>
```

- 포트는 운영 launch 로그 `rescue_db_writer ... api=http://127.0.0.1:<port>` 의 값이다.
- 보낼 때마다 새 `command_id` 를 만든다. 다시 돌리면 새 명령이다.
- 답 JSON 을 그대로 찍는다. 끝 코드는 0 접수, 1 거절, 2 보내지 않음(인자나 DB 값 문제), 3 답 없음이다. 3 이면 명령이 들어갔는지 모른다.
- `DISPATCH_SUPPORT`·`CONTINUE_SEGMENT` 의 revision 은 손으로 넣지 않는다. `--db <site_state.sqlite3>` 로 운영 DB 의 지금 값을 읽어 채운다.

## `status_watch.py`

```bash
python3 scripts/field/noweb/status_watch.py --db <site_state.sqlite3> --log <status.log>
```

- 운영 DB 를 읽기 전용으로 연다. 파일을 만들거나 쓰지 않는다.
- 한 줄에 회차, 임무(mode/phase, 조정기 이유, 지원 출동 구간), 로봇별 준비·이동·도킹·자세·작업·배터리, 실시간 지도와 고정 지도, 대상, 초기화 뒤 재기동 확인이 들어간다.
- 시각이 있는 값에는 「N초 전」이 붙고, 5초가 넘으면 「낡음」이 붙는다. DB 에 없는 값은 「없음」으로 찍는다.
- 고정 지도 단계는 운영 launch 에 `map_topic`·`map_dir` 를 주었을 때만 보인다. 주지 않았으면 「지도 보관 꺼짐 — 고정 여부 모름」이 찍힌다.
- Nav2 상태는 운영 DB 에 없다. 로봇 담당 PC 로그에서 본다.

## `two_robots.rviz` 와 `tf_merge.py`

두 터미널에서 띄운다. 둘 다 1단계 「터미널 준비」(Discovery 설정)를 먼저 넣는다. 시뮬레이션도 같은 파일을 쓴다.

```bash
python3 scripts/field/noweb/tf_merge.py --robots robot6 robot5
rviz2 -d scripts/field/noweb/two_robots.rviz
```

- **두 로봇을 한 창에 그린다(재범 「제일 중요」, 2026-10-16).** 두 로봇은 TF 를 각자 `/robot6/tf`·`/robot5/tf` 에 같은 프레임 이름(`odom`, `base_link` …)으로 낸다. `tf_merge.py` 가 로봇 프레임마다 이름공간을 앞에 붙인다(`robot6/odom`, `robot5/base_link`). 공유 프레임 `map` 과 이미 접두가 있는 프레임은 그대로 둔다. 그렇게 `/tf`·`/tf_static` 하나로 합친다.
- 라이다는 `/rviz/<ns>/scan` 으로 프레임 이름만 바꿔 다시 낸다. 로봇 모델은 RViz `TF Prefix` 로 맞춘다.
- `tf_merge.py` 는 RViz 를 띄우는 PC 에서만 돌린다. `/tf` 를 읽는 다른 노드 옆에서 돌리지 않는다. 우리 노드는 모두 `<ns>/tf` 를 읽는다.
- 고정 프레임은 `map` 이다. 그리는 것은 다음과 같다.
  - TB-6 SLAM 지도(`/robot6/map` 과 갱신), 고정 지도(`/rescue/map/frozen`)
  - 두 로봇의 모델·라이다·경로, TB-5 AMCL 자세
  - 인형과 출구 표시(`target_markers.py` 를 띄웠을 때)
- 카메라와 목표 지정 도구는 넣지 않았다. 이 화면에서는 아무것도 발행하지 않는다.
- **TB-5 는 지도 고정 전에도 출발 자리에 그려진다(재범 지적 2026-10-16 08:00).** TB-5 는 고정 지도를 받아 AMCL 이 켜져야 `map → odom` 을 낸다. 그 전에는 `tf_merge.py` 가 임시 `map → robot5/odom` 을 `/tf` 에 낸다. 값은 현장 파일의 TB-5 출발 값(`robots.robot_b.initial_pose_xy_yaw`, TB-6 출발 자세 기준, AMCL 시작값과 같은 값)을 TB-6 출발 자세(`/rescue/map/start_pose/robot_a`)에 더한 자리다.
  - TB-6 출발 자세와 TB-5 의 `odom → base_link` 가 둘 다 와야 그린다. 출발 자세는 TB-6 지도 고정 노드가 회차에 한 번 낸다. 그 전에는 TB-5 가 안 보인다.
  - TB-5 자신의 `map → odom` 이 한 번이라도 오면 임시 값을 다시 내지 않는다. 로그에 `TF_PROVISIONAL_OFF` 가 남는다. 초기화 뒤에는 `tf_merge.py` 를 다시 띄운다.
  - 임시 값은 RViz 용 `/tf` 에만 나간다. 로봇 쪽 `/robot5/tf` 에는 아무것도 보내지 않는다.
  - RViz 는 TF 로 모델 모양을 바꿀 수 없다. 그래서 임시 자리라는 표시는 로그(`TF_PROVISIONAL`)뿐이다.
  - 다른 현장 파일은 `--field <파일>`, 다른 로봇은 `--provisional <ns>=<로봇 ID>` 로 준다. `--provisional ''` 이면 끈다. 출발 값이 없는 현장 파일이면 시작하지 않는다.
- 저장소에서는 파일 문법(YAML, 토픽 이름)만 시험한다(`tests/test_field_noweb_rviz.py`). 실제 화면은 실기에서 아직 보지 않았다(NOT_RUN).

## 한 PC 리허설

`REHEARSAL_PROBE=noweb bash scripts/field/wiring-rehearsal/run.sh <install> <out> SEARCHING <port> <domain>` 은 가짜 로봇 위에서 이 두 도구로 한 회차를 돈다. 순서는 DECLARE_DISASTER → 상태 줄 「임무 DISASTER」 → TB-6 목표(가짜 Nav2 가 거절) → 도킹 → RESET_RUN → 「초기화 확인」 줄이다(`wiring-rehearsal/noweb_probe.py`). 2026-10-15 기록은 [웹 없는 트랙 리허설](../../../docs/evidence/rehearsal-2026-10-15-noweb.md)에 있다.

## `target_markers.py`

```bash
python3 scripts/field/noweb/target_markers.py
```

- 조정기의 대상 목록(`/rescue/targets`), 인형 위치(`/rescue/target_locations`), 출구 위치(`/rescue/exit_location`)를 구독한다. 이 메시지들은 우리 전용 메시지라 RViz 가 바로 그리지 못한다.
- RViz 표시(`visualization_msgs/MarkerArray`)로 바꿔 `/rescue/noweb/markers` 에 낸다. `two_robots.rviz` 의 「Targets and exits」가 이 토픽을 그린다.
- 대상은 공 모양이고 단계마다 색이 다르다. 후보(CANDIDATE)는 노랑, 확정(CONFIRMED)은 주황, 구조(RESCUED)는 초록, 버림(DISMISSED)은 회색이다.
- 조정기 대상에 아직 들어가지 않은 인형 위치는 작은 노란 공(CANDIDATE)으로 그린다.
- 출구는 파란 상자다. 글자는 QR 내용이다.
- 글자에는 짧은 ID 와 단계를 붙인다. RViz 글꼴에 한글이 없을 수 있어 글자는 영어로 쓴다 [추정, 확인 안 함].
- 메시지에 실린 유효 시각(`valid_until_ns`)이 지나면 흐리게 그리고 글자에 `STALE <초>s` 를 붙인다. 나이는 메시지 시각과 이 PC 시계로 잰다. 보낸 PC 와의 시계 차이는 빼지 않는다.
- 지도 기준 좌표계(`mission_frame`)가 비어 있는 메시지는 그리지 않고 경고 로그를 남긴다.
- 구독만 한다. 로봇에 아무것도 보내지 않고 영상도 받지 않는다. `--period`(기본 1초)마다 다시 그린다.
- 메시지에서 표시로 바꾸는 함수는 ROS 없이 시험한다(`tests/test_field_noweb_target_markers.py`). 이 저장소 PC 의 로컬 ROS(Jazzy)에서 가짜 메시지로 표시가 나오는 것을 봤다. 실기와 실제 RViz 화면은 NOT_RUN 이다.
