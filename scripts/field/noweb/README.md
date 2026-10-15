# 웹 없이 하는 회차의 도구

관제 웹 없이 RViz 와 터미널만으로 실기 회차를 돌릴 때 쓴다(재범 결정 2026-10-15). 순서는 [현장 실행 안내 「웹 없이 하기」](../../../docs/process/field-runbook.md#웹-없이-하기) 절에 있다.

| 파일 | 하는 일 | 어디서 |
| --- | --- | --- |
| `operator_command.py` | 관제 웹 버튼 대신 운영 API 로 운영자 명령과 초기화를 보낸다 | 운영 PC |
| `status_watch.py` | 관제 웹 카드 대신 운영 DB 를 읽어 1초마다 상태 한 줄을 쓴다 | 운영 PC |
| `two_robots.rviz` | 두 로봇의 지도, 경로, 위치를 보는 RViz 화면 | RViz 를 띄울 PC |

세 도구 모두 로봇을 움직이지 않는다. 앞의 두 도구는 표준 라이브러리와 이 저장소의 `rescue_operations` 만 쓰고 ROS 가 필요 없다. PC 이름과 주소는 인자로 받는다.

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

## `two_robots.rviz`

```bash
rviz2 -d scripts/field/noweb/two_robots.rviz --ros-args -r /tf:=/robot6/tf -r /tf_static:=/robot6/tf_static
```

- 고정 프레임은 `map` 이다. TB-6 SLAM 지도(`/robot6/map` 과 갱신), 고정 지도(`/rescue/map/frozen`), TB-6 모델, TB-6 라이다, TB-6 경로, TB-5 AMCL 자세, TB-5 경로를 그린다.
- 카메라와 목표 지정 도구는 넣지 않았다. 이 화면에서는 아무것도 발행하지 않는다.
- **RViz 하나는 TF 를 하나만 읽는다.** 위 명령은 TB-6 의 TF 를 읽으므로 TB-5 의 모델과 라이다는 그리지 못한다. TB-5 는 AMCL 자세 화살표와 경로로만 보인다.
- **1차 고정 전에는 TB-5 가 안 보이는 것이 정상이다.** TB-5 는 고정 지도를 받아야 위치 추정을 시작한다.
- 저장소에서는 파일 문법(YAML, 토픽 이름)만 시험한다(`tests/test_field_noweb_rviz.py`). 실제 화면은 실기에서 아직 보지 않았다(NOT_RUN).

## 한 PC 리허설

`REHEARSAL_PROBE=noweb bash scripts/field/wiring-rehearsal/run.sh <install> <out> SEARCHING <port> <domain>` 은 가짜 로봇 위에서 이 두 도구로 한 회차를 돈다. 순서는 DECLARE_DISASTER → 상태 줄 「임무 DISASTER」 → TB-6 목표(가짜 Nav2 가 거절) → 도킹 → RESET_RUN → 「초기화 확인」 줄이다(`wiring-rehearsal/noweb_probe.py`). 2026-10-15 기록은 [웹 없는 트랙 리허설](../../../docs/evidence/rehearsal-2026-10-15-noweb.md)에 있다.
