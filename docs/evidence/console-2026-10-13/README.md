# 관제 웹 화면·시험 기록 (2026-10-13, E2)

관제 웹(`rescue_console`)을 로봇 없이 띄운 화면과 시험 기록이다. **합성 데이터**로 띄운 화면이다. 현장 회차나 로봇 상태가 아니다.

- 코드: main `bd4bfa5` 와 `src/` 가 같은 커밋(이 PR 의 첫 커밋 `5b3b153`)에서 실행했다.
- 환경: 클라우드 작업 컨테이너, Python 3.13, Flask 3.1.3(가상 환경), Chromium(Playwright) 1440×900.
- 로봇·ROS·operations 는 띄우지 않았다. 그래서 명령 버튼은 「operations 주소 없음」으로 꺼져 있다(정상 동작).

## 파일

| 파일 | 무엇 |
| --- | --- |
| [01-login.png](01-login.png) | 로그인 화면 |
| [02-dashboard.png](02-dashboard.png) | 로그인 뒤 관제 화면(1초 폴링 2~3회 뒤) |
| [console-tests-flask.log](console-tests-flask.log) | `tests/test_console_*.py` 198개, Flask 설치 상태, 원문 출력. 경로는 `<REPO>`·`<TMP>` 로 바꿨다 |

## 만든 방법

1. `python -m rescue_bringup.offline_demo config/deployment.example.json --output <TMP>/run` — 오프라인 리허설 장부(exit 0, `E2 SYNTHETIC PASS: 14 checks`).
2. `RESCUE_CONSOLE_ADMIN_PASSWORD=<임시> python -m rescue_console.accounts init <TMP>/accounts.sqlite3 --admin admin`
3. 상태 픽스처 [`src/rescue_console/examples/status.example.json`](../../../src/rescue_console/examples/status.example.json) 을 복사하고 `*_ns` 시각 11개를 「가장 늦은 시각 = 지금 − 5 s」가 되도록 같은 양만큼 옮겼다. 값의 관계(순서·간격)는 그대로다. 사건 기록의 2023년 줄은 1번 장부의 시각이라 옮기지 않았다.
4. `RESCUE_CONSOLE_SECRET=<임시> python -m rescue_console <TMP>/run/operations.sqlite3 --accounts <TMP>/accounts.sqlite3 --status <옮긴 픽스처> --map assets/maps/pre_quake/map.yaml --robot-name robot_a=TB-6 --robot-name robot_b=TB-5 --support-dispatch AUTO --host 127.0.0.1 --port 8080`
5. Playwright 로 `/login` 을 찍고, admin 으로 로그인한 뒤 2.5 s 기다려 전체 화면을 찍었다.

## 화면에서 읽을 것

- TB-6 카드의 「낡음: 13.0s 동안 새 상태 없음」: 픽스처는 갱신되지 않으므로 시간이 지나면 낡음으로 보인다. 상태가 낡으면 숨기지 않고 경고하는 동작이다(AGENTS.md 7절).
- 「TB-5 자동 출동」: `--support-dispatch AUTO` 이면 출동 승인 버튼 대신 상태만 보인다.
- 꺼진 버튼마다 이유가 적혀 있다(「구출 판정 전」, 「Q-22 재개 관문 전」 등). 정지 버튼은 늘 꺼져 있다 — 현장 비상 정지 장치를 쓴다.
- 웹캠 칸은 `--camera` 를 주지 않아 안내 문구가 보인다.

## 아직 없는 것

- 실제 operations·조정기에 연결한 화면(명령 접수·거절 표시), 웹캠 영상이 나오는 화면, 현장 실시간 SLAM 지도(`--live-map`) 화면: 현장 회차에서 찍는다. NOT_RUN.
- 영상(동영상): 없음.
