---
title: Fast DDS Discovery Server 조사와 채택안
status: draft
updated: 2026-10-08
---

# Fast DDS Discovery Server 조사와 채택안 (작전, 2026-10-08)

> 배경: 튜터가 멀티 로봇 구성에 Discovery Server 사용을 지시함 (원문 미확인, 전달: 사용자, 2026-10-08).
> 결론 먼저. 출처 표기: [문서] [측정] [추정] [미확인]. 측정은 이 PC(Jazzy, Fast DDS 2.14.6, `rmw_fastrtps_cpp`) 로컬 전용 서버(127.0.0.1:11899, 도메인 77/78)에서 했고 로봇은 쓰지 않았다.

## 0. 결론

> [!NOTE]
> 2026-10-10 현장 확인 전 권고다. 운영 PC(PC4 역할)는 **중앙 Discovery Server(서버 ID 0, 11811)** 를 손으로 띄워 쓰고 있고 클라이언트 목록에 자기 서버만 있다는 측정이 있다([운영 PC 현황 기록](../process/ops-host-record.md), PR #49, 웹마스터). 로봇 쪽 설정(온보드 서버·오프보드 서버 지정)은 [미확인]이라, 아래 2번(권고안 A)과 현장 구성 중 어느 쪽이 팀 결정인지는 로봇 설정을 읽은 뒤 정한다(실기·배포 → 작전).

1. **이미 쓰고 있다.** P4의 robot3는 Pi 안에서 Discovery Server(포트 11811, 서버 ID 3, 도메인 3)를 돌리고, 제어 PC는 `ROS_DISCOVERY_SERVER=;;;<robot3>:11811` 로 붙어 있었다 [측정, P4 DEC-003·hardware-spec]. 튜터 지시는 "로봇 2대를 서버 2개로 한 PC에서 같이 보라"는 멀티 로봇 구성으로 읽힌다 [추정].
2. **매뉴얼 방식(A)으로 가면 로봇 설정을 바꾸지 않아도 된다.** 로봇마다 자기 Pi 서버(ID 고유), PC는 두 서버를 목록에 적고, 네임스페이스 `/robotN` 으로 구분한다 [문서: TB4 Discovery Server]. 서버 ID는 이미 로봇 번호(3, 6)로 고유하다 [측정 robot3 / 미확인 robot6].
3. **도메인은 Discovery Server에서 무시된다.** Fast DDS 문서가 "DDS Domain 개념은 적용되지 않는다"고 쓰고 [문서], 로컬 측정에서도 도메인 78 클라이언트가 도메인 77 발행자를 같은 서버로 보고 데이터까지 받았다 [측정 C·D]. 따라서 P4 DEC-010의 "도메인 통일(A) vs 로봇별 도메인(B)" 구도는 Discovery Server에서는 의미가 없다: **PC 한 대가 서버 목록만으로 두 로봇을 동시에 본다.** 실기 확인은 필요하다(로봇 Pi의 Fast DDS 버전 [미확인]).
4. 서버는 **발견**만 맡는다. 서버가 죽어도 이미 맺어진 연결은 계속 데이터를 주고받고(10초간 56건 수신 [측정 H2]), 새 노드만 아무것도 못 찾는다 [측정 H3·H4]. 서버가 같은 ID·포트로 다시 뜨면 2초 안에 기존 노드가 다시 보인다 [측정 H5, localhost 값].
5. 위험은 세 가지다: (a) 서버 IP를 아는 다른 PC는 **도메인과 무관하게** 우리 로봇에 붙는다(P4 10-06 `/robot3/map` 발행자 2개 사고의 구조적 원인) (b) Pi 서버 CPU 부하(P4 92% [측정], 원인은 [추정]) (c) PC의 `.bashrc`에 서버 설정이 남아 있으면 **오프라인 시험 노드끼리도 서로 못 찾는다** [측정 F·H3].

## 1. Discovery Server가 무엇인가

- Fast DDS 전용 발견 방식. 기본 Simple Discovery는 멀티캐스트로 서로를 찾지만, Discovery Server는 서버 하나(여기서는 로봇 Pi)에 유니캐스트로 등록하고 서버가 "누가 어떤 토픽을 쓰는지" 중계한다 [문서: Fast DDS DS, Wi-Fi use case].
- Wi-Fi에서 멀티캐스트가 불안정하고, 기준 트래픽이 적어 여러 로봇으로 확장하기 좋고, Create 3가 Wi-Fi에 붙을 필요가 없어진다 [문서: TB4 networking]. 반대로 `rmw_fastrtps_cpp` 전용이고 CycloneDDS와 섞을 수 없으며, Simple Discovery와 혼용도 안 된다 [문서].
- 참가자 종류: SERVER / CLIENT(자기와 맞는 토픽 정보만 받음) / SUPER_CLIENT(전부 받음, 조회 도구용) / BACKUP(서버 상태를 파일에 보존) [문서: Fast DDS 2.14].
- 서버 v2 필터: 토픽이 겹치지 않는 노드는 서로를 아예 모른다. 그래서 `ros2 topic list` 같은 조회 도구는 SUPER_CLIENT여야 전체가 보인다 [문서, 측정 A·B].

## 2. TurtleBot 4 구성 (Jazzy, turtlebot4_setup ≥ 1.0.4)

| 구성 요소 | 내용 | 출처 |
|---|---|---|
| Pi 온보드 서버 | `turtlebot4-setup` → ROS Setup → Discovery Server: Enabled, Onboard Port(11811), **Onboard Server ID(0~255, 로봇마다 고유)**, Offboard Server IP/Port/ID(선택) | [문서: TB4 setup tool] |
| Create 3 | USB-C 이더넷 `192.168.186.3:<port>` 의 **온보드 서버에만** 클라이언트로 붙는다(오프보드는 Pi만). 설정은 Pi가 USB-C로 보내고 Create 3가 재부팅한다 | [문서: conf.py, changelog 1.0.4, setup tool] |
| 오프보드 서버 | Pi 노드들이 추가로 붙는 두 번째 서버(예: 관제 PC). 로봇끼리 직접 통신이 필요할 때만 의미가 있다 | [문서: setup tool] |
| PC 설정 | `configure_discovery.sh` 가 `/etc/turtlebot4_discovery/setup.bash` 를 쓴다: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `[ -t 0 ] && ROS_SUPER_CLIENT=True || False`(대화형 터미널만), `ROS_DOMAIN_ID`, `ROS_DISCOVERY_SERVER="ip:port;ip:port;"`. XML은 더 이상 쓰지 않는다 | [문서: configure_discovery.sh jazzy] |
| 적용 뒤 | `source ~/.bashrc; ros2 daemon stop; ros2 daemon start`. 네트워크 변경·로봇 재부팅 뒤에도 같다 | [문서] |
| 매뉴얼 2대 예시 | robot1: `/robot1`, ID 0 / robot2: `/robot2`, ID 1 / PC: `ROS_DOMAIN_ID=0`, `ROS_DISCOVERY_SERVER="192.168.131.5:11811;192.168.131.6:11811;"` | [문서: TB4 Discovery Server] |

`ROS_DISCOVERY_SERVER` 규칙: 세미콜론으로 나눈 **자리 번호가 서버 ID**다. ID 3 서버 하나만 쓰면 `;;;ip:11811`, ID 3과 6이면 `;;;ip3:11811;;;ip6:11811` [문서: ROS 2 DS tutorial, 측정: P4 제어 PC 값]. 매뉴얼은 PC의 `ROS_DOMAIN_ID`를 로봇과 맞추라고 쓰지만(0 기본) [문서], 아래 측정처럼 발견과 데이터 모두 도메인을 보지 않는다.

## 3. 로컬 측정 (2026-10-08, 이 PC, 로봇 없음)

서버 `fastdds discovery -i 0 -l 127.0.0.1 -p 11899`, 발행자 `ros2 topic pub /ds_probe`(도메인 77, 서버 클라이언트), `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. 측정 스크립트는 실기·배포 레인이 `scripts/field/` 로 옮긴다(작성 시점에는 작전 PC 의 임시 폴더).

| # | 조건 | 결과 |
|---|---|---|
| A | 클라이언트, 도메인 77, `ROS_SUPER_CLIENT=False`, `topic list --no-daemon` | `/ds_probe` **안 보임** (자기 토픽만) |
| B | 같은 조건, `ROS_SUPER_CLIENT=True` | `/ds_probe` 보임 |
| C | **도메인 78**, SUPER_CLIENT=True | `/ds_probe` 보임 → 발견은 도메인 무시 |
| D | 도메인 78, SUPER_CLIENT=False, `echo --once` | `data: hi` 수신 → 데이터도 도메인 무시 |
| E | 도메인 77, SUPER_CLIENT=False, `echo --once` | 수신 (일반 노드는 SUPER_CLIENT 없이도 통신) |
| F | 서버 설정 없이 Simple Discovery, 도메인 77 | `/ds_probe` 안 보임 → 서버 클라이언트는 멀티캐스트에 안 나온다 |
| H0~H2 | 구독 중 서버 kill | 죽기 전 24건, 죽은 뒤 10초간 **56건 계속 수신** (5 Hz) |
| H3·H4 | 서버 죽은 상태에서 새 클라이언트 | 목록 비고 `echo --once` 12초 timeout → 새 노드는 발견 불가 |
| H5·H6 | 같은 ID·포트로 서버 재시작 | 2초 뒤 기존 발행자 다시 보임, 새 구독자 수신 |
| H7 | 기존 구독자 | 서버 죽음·재시작 내내 수신 유지 (총 199건) |
| - | `fastdds discovery` 래퍼 kill | 자식 `fast-discovery-server` 가 남는다. P4 "서비스 재시작으로는 서버가 안 내려감, PID 그대로" 와 같은 현상. 끌 때는 자식 PID를 지정 |

## 4. 운영에서 걸리는 것 (P4 기록 + 측정)

| 증상 | 원인 | 대응 |
|---|---|---|
| `ros2 topic list` 가 비거나 적게 보임 | 비대화형 셸은 SUPER_CLIENT 아님 / 데몬이 재부팅 전 목록을 들고 있음 [측정 P4 10-06 20:56] | 조회는 `ROS_SUPER_CLIENT=True`, 로봇 재부팅·네트워크 변경 뒤 `ros2 daemon stop; ros2 daemon start`. 생존 판정은 목록이 아니라 수신 시각(이미 설계) |
| 마스터에 Create 3 토픽이 안 옴 | Pi `fast-discovery-server` CPU 92% [측정 P4 10-06], 참가자 수 때문이라는 해석은 [추정] | 카메라 구독 하나, 안 쓰는 노드·RViz 끄기. AC-01에서 참가자 수 대 CPU를 다시 잰다. SUPER_CLIENT를 터미널에만 쓰는 이유도 이것 [문서 changelog 1.0.4] |
| 다른 PC가 같은 로봇에 SLAM을 띄움 | 서버 IP만 알면 도메인과 무관하게 붙는다 [측정 C·D] | 시작 전 `/robotN/map`, `/robotN/cmd_vel` 발행자 수 확인(P4 규칙). 서버 목록을 적는 PC를 정해 둔다 |
| Nav2 가 안 뜸 | [미확인]. 강사 안내는 `ROS_SUPER_CLIENT=False` 뒤 재실행 | 런처는 비대화형이라 원래 False. 터미널에서 `ros2 launch` 하면 True가 상속되므로 명시적으로 False |
| 오프라인 시험에서 가짜 로봇과 노드가 서로 못 찾음 | `.bashrc` 의 `ROS_DISCOVERY_SERVER` 가 상속돼 모두 "로봇 서버의 클라이언트"가 된다. 서버가 안 닿으면 아무도 못 찾는다 [측정 F·H3] | 시험 스크립트에서 `unset ROS_DISCOVERY_SERVER` 하고 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 또는 로컬 서버를 띄우고 그 주소를 쓴다. 팀 `AGENTS.md` 2절 규칙에 한 줄 추가 대상 |
| 서버가 죽음 | 기존 연결 유지, 새 노드만 막힘 [측정 H] | 재시작 뒤 자동 재등록(2초, localhost). Pi 재부팅의 "약 2분"은 부팅·노드 기동 시간이다 [추정] |

## 5. 토폴로지 후보

| 안 | 구성 | 장점 | 단점 | 로봇 설정 변경 |
|---|---|---|---|---|
| **A 매뉴얼** (권고) | 로봇마다 Pi 온보드 서버(ID 3, 6). 로봇 담당 PC는 자기 로봇 서버, 관제 PC는 두 서버 모두 목록에 적음. `/robot3`, `/robot6` | 매뉴얼 그대로. 로봇은 Wi-Fi가 끊겨도 Pi–Create 3 가 돈다. 로봇 설정 그대로 | 로봇끼리 직접 토픽 없음(서버가 안 이어짐). 협업은 PC의 coordinator를 거친다 — 현재 C-08 설계와 같다 | **없음** (서버 ID가 이미 고유하면) |
| A+오프보드 | A에 더해 두 Pi가 관제 PC의 서버(ID 예: 10)에도 붙음 | 한 그래프. 로봇 간 직접 토픽 가능 | 관제 PC 서버가 죽으면 새 발견 불가. Pi 설정 변경(등급 4) 2대 | 2대 |
| B 로봇별 도메인 분리 | P4 DEC-010 B | - | Discovery Server에서는 도메인이 격리를 주지 않는다 [측정 C·D]. 격리는 서버 목록이 한다 | - |

권고: **A.** F-01(수색 분담)과 구출 보고는 coordinator/operations가 PC에서 중계하므로 로봇 간 직접 토픽이 필요 없다. 오프보드는 실기에서 로봇 간 직접 경로가 필요해질 때 열어 둔다.

## 6. 우리 저장소에 반영할 것

1. `rescue_bringup` profile v2.1 에 호스트별 `network` 블록: `domain_id`, `discovery_servers[{server_id, host, port}]`, `namespace`, `super_client=false`. 코드는 값을 읽어 기록·비교만 하고 발견 성공을 가정하지 않는다 (development.md 미구현 후보 그대로).
2. 실행 전 점검(E3): 프로세스의 실제 `ROS_DISCOVERY_SERVER`·`RMW`·`ROS_SUPER_CLIENT` 를 프로파일과 비교, 로봇 서버 포트 도달 여부, `/robotN/cmd_vel`·`/robotN/map` 발행자 수, 최근 수신 시각. P4 `preflight.py` 의 `discovery_host()` 를 가져온다.
3. 오프라인 시험 규칙: `scripts/run_tests.py` 와 가짜 로봇은 `ROS_DISCOVERY_SERVER` 를 지우고 시작한다.
4. AC-01·CT-08 의 A0 실험 목록을 구체화: (a) 관제 PC가 두 서버 목록으로 두 로봇을 동시에 보는가(도메인 다른 채로) (b) 참가자 수 대 Pi 서버 CPU (c) Pi 재부팅 뒤 재등록 시간 (d) 두 PC가 같은 서버에 붙을 때 발행자 충돌 감지.
5. `docs/scope.md` 에 Q 추가: 튜터 지시 원문, 두 로봇의 현재 서버 ID·도메인·네임스페이스, PC 4대의 서버 목록.

## 7. 사용자 확인 필요

- 튜터가 말한 구성의 원문(Notion 'Multi Robot' 두 쪽, pD5 p.55-56). 오프보드 서버까지 요구하는지.
- robot6 의 현재 서버 ID·도메인·네임스페이스 (robot3 는 ID 3·도메인 3·`/robot3` [측정 P4]).
- A안 채택 여부. 채택하면 PC 쪽 설정만 바꾸고 로봇 설정 변경은 0건이다.

## 출처

- TB4 Discovery Server: https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html
- TB4 Networking: https://turtlebot.github.io/turtlebot4-user-manual/setup/networking.html
- TB4 setup tool: https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_setup.html
- turtlebot4_setup jazzy `configure_discovery.sh`, `conf.py`, CHANGELOG 1.0.4 (2024-07-02): https://github.com/turtlebot/turtlebot4_setup/tree/jazzy
- Create 3 Discovery Server: https://iroboteducation.github.io/create3_docs/setup/discovery-server/
- ROS 2 Jazzy Discovery Server tutorial: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html
- Fast DDS 2.14 Discovery Server: https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/discovery/discovery_server.html
- Fast DDS Wi-Fi use case: https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/use_cases/wifi/discovery_server_use_case.html
- Fast DDS ROS 2 DS: https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/ros2/discovery_server/ros2_discovery_server.html
- P4 기록: DEC-003, DEC-010, `mini/field/2026-10-06-practice.md`(ROKEY_P4 1a2d0a9), `docs/reference/2026-10-04-hardware-spec-hv03-vs-mu03.md`
