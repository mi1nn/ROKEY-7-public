---
title: Fast DDS Discovery Server 조사와 채택안
status: draft
updated: 2026-10-08
---

# Fast DDS Discovery Server 조사와 채택안 (작전, 2026-10-08 · 실기·배포 보강 같은 날)

> 배경: 튜터가 멀티 로봇 구성에 Discovery Server 사용을 지시함 (원문 미확인, 전달: 사용자, 2026-10-08).
> 결론 먼저. 출처 표기: [문서] [측정] [추정] [미확인]. 3절 측정은 작전 PC(Jazzy, Fast DDS 2.14.6, `rmw_fastrtps_cpp`) 로컬 전용 서버(127.0.0.1:11899, 도메인 77/78)에서, 3.1절 재측정은 실기·배포 PC 의 Jazzy 컨테이너에서 했다. 로봇은 쓰지 않았다.
> 실기·배포가 더한 것: 3.1 재측정, 8 PC 별 설정 절차와 되돌리기, 9 오프라인 시험 규칙, 10 실행 전 점검 `scripts/field/preflight.py`.

## 0. 결론

1. **이미 쓰고 있다.** P4의 robot3는 Pi 안에서 Discovery Server(포트 11811, 서버 ID 3, 도메인 3)를 돌리고, 제어 PC(hv-03)는 `ROS_DISCOVERY_SERVER=;;;<robot3>:11811` 로 붙어 있었다 [측정, P4 DEC-003·hardware-spec]. 튜터 지시는 "로봇 2대를 서버 2개로 한 PC에서 같이 보라"는 멀티 로봇 구성으로 읽힌다 [추정].
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

`ROS_DISCOVERY_SERVER` 규칙: 세미콜론으로 나눈 **자리 번호가 서버 ID**다. ID 3 서버 하나만 쓰면 `;;;ip:11811`, ID 3과 6이면 `;;;ip3:11811;;;ip6:11811` [문서: ROS 2 DS tutorial, 측정: P4 hv-03 값]. 매뉴얼은 PC의 `ROS_DOMAIN_ID`를 로봇과 맞추라고 쓰지만(0 기본) [문서], 아래 측정처럼 발견과 데이터 모두 도메인을 보지 않는다.

## 3. 로컬 측정 (2026-10-08, 작전 PC, 로봇 없음)

서버 `fastdds discovery -i 0 -l 127.0.0.1 -p 11899`, 발행자 `ros2 topic pub /ds_probe`(도메인 77, 서버 클라이언트), `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. 작성 시점의 스크립트는 작전 PC 의 임시 폴더에 있었고 저장소에 없다. 같은 경우를 다시 재는 탐침은 3.1 절의 [`tests/ros/hosts_discovery_probe.py`](../../tests/ros/hosts_discovery_probe.py)다.

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

### 3.1 재측정 (2026-10-08, 실기·배포 PC, 로봇 없음)

이 PC 에는 ROS 가 설치돼 있지 않다 [측정: Ubuntu 26.04, `/opt/ros` 없음]. 그래서 `ros:jazzy-ros-base` 컨테이너(Fast DDS 2.14.6, `rmw_fastrtps_cpp` 기본값 [측정])를 `--network none` 으로 띄워 **loopback 밖으로는 패킷이 못 나가게** 하고 쟀다. 도메인 42, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, 서버 `127.0.0.1:11899`(ID 0). 탐침은 시작할 때 `ROS_DISCOVERY_SERVER` 가 남아 있으면 거절한다. 원문 로그: [hosts-discovery-probe](../evidence/field-2026-10-08-hosts-discovery-probe.log).

```sh
docker run --rm --network none -v <REPO>:/repo:ro -w /repo \
  -e ROS_DOMAIN_ID=42 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros:jazzy-ros-base \
  bash -c 'source /opt/ros/jazzy/setup.bash && python3 tests/ros/hosts_discovery_probe.py'
```

| # | 조건 | 기대 | 결과 [측정 2026-10-08] |
|---|---|---|---|
| A | 서버 있음, 일반 클라이언트(`ROS_SUPER_CLIENT=False`)가 목록 조회 | 안 보임 | 안 보임 — 3절 A 와 같음 |
| B | 서버 있음, super client 가 목록 조회 | 보임 | 보임 — 3절 B 와 같음 |
| C | 서버 있음, 일반 클라이언트가 구독(`echo --once`) | 수신 | 수신 — 서로를 찾는다 |
| F | 서버 있음, 발행자는 서버 클라이언트, 구독자는 Simple Discovery | 못 받음 | 12 초 안에 못 받음 — 3절 F 와 같음 |
| P | preflight 를 이 서버에 붙여 `/probe_robot/cmd_vel` 발행자 2개를 셈 | `FAILED`, 2 | `FAILED`, 2 — 중복 발행 감지 경로가 실제 그래프에서 동작 |
| R1 | 서버가 떠 있을 때 preflight 의 UDP 포트 점검 | `FAILED` 아님 | `no_reply` → `NOT_RUN` (UDP 는 침묵이 열림의 증명이 아님) |
| H | 서버 프로세스 그룹을 끈 뒤 새 클라이언트가 구독 | 못 받음 | 못 받음 — 3절 H3·H4 와 같음 |
| R2 | 서버를 끈 뒤 같은 UDP 점검 | `FAILED` | `refused` → `FAILED` |
| S | 서버 설정이 아무 데도 없음, 양쪽 Simple Discovery | 수신 | 수신 — 오프라인 시험의 기준 상태 |

9/9 일치, 종료 코드 0 [측정]. 3절의 C·D(도메인 무시)·H5~H7(재시작 뒤 복구)은 다시 재지 않았다 — 미실행. 서버는 프로세스 그룹으로 껐고 R2 의 `refused` 로 자식 `fast-discovery-server` 까지 내려간 것을 확인했다(3절 마지막 행의 "자식이 남는다" 문제를 피하는 방법).

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
2. 실행 전 점검(E3) — **10절에서 구현**(최근 수신 시각은 아직 없음): 프로세스의 실제 `ROS_DISCOVERY_SERVER`·`RMW`·`ROS_SUPER_CLIENT` 를 프로파일과 비교, 로봇 서버 포트 도달 여부, `/robotN/cmd_vel`·`/robotN/map` 발행자 수, 최근 수신 시각. P4 `preflight.py` 의 `discovery_host()` 를 가져온다.
3. 오프라인 시험 규칙 — **9절**: `scripts/run_tests.py` 와 가짜 로봇은 `ROS_DISCOVERY_SERVER` 를 지우고 시작한다.
4. AC-01·CT-08 의 A0 실험 목록을 구체화: (a) 관제 PC가 두 서버 목록으로 두 로봇을 동시에 보는가(도메인 다른 채로) (b) 참가자 수 대 Pi 서버 CPU (c) Pi 재부팅 뒤 재등록 시간 (d) 두 PC가 같은 서버에 붙을 때 발행자 충돌 감지.
5. `docs/scope.md` 에 Q 추가: 튜터 지시 원문, 두 로봇의 현재 서버 ID·도메인·네임스페이스, PC 4대의 서버 목록.

## 7. 사용자 확인 필요

- 튜터가 말한 구성의 원문(Notion 'Multi Robot' 두 쪽, pD5 p.55-56). 오프보드 서버까지 요구하는지.
- robot6 의 현재 서버 ID·도메인·네임스페이스 (robot3 는 ID 3·도메인 3·`/robot3` [측정 P4]).
- A안 채택 여부. 채택하면 PC 쪽 설정만 바꾸고 로봇 설정 변경은 0건이다.

## 8. PC 별 설정 절차와 되돌리기 (사람이 실행, 실기·배포 2026-10-08)

PC 의 셸 설정을 바꾸는 일은 [팀 작업 규칙](../../AGENTS.md) 6절의 설정 변경 절차(백업·전후 해시·사용자 승인 날짜)를 따른다. AI 세션은 아래 명령을 실행하지 않고, 사람이 실행한 뒤 결과(해시·preflight JSON)를 PR 이나 실행 기록에 붙인다. 로봇 Pi 는 바꾸지 않는다(토폴로지 A 는 로봇 쪽 변경이 0건이다).

**원칙: `.bashrc` 에 넣지 않는다 [제안].** TB4 의 `configure_discovery.sh` 는 `/etc/turtlebot4_discovery/setup.bash` 를 쓰고 `.bashrc` 에서 읽게 한다 [문서]. 그러면 모든 셸이 로봇 서버의 클라이언트가 되어 오프라인 시험 노드끼리도 서로 못 찾는다(3절 F·H, 3.1 절 F·H [측정]). 그래서 현장 터미널에서만 `source` 하는 파일 하나를 쓴다.

| 단계 | 명령(예시, `<...>` 는 Q-01 배정 뒤 값) | 남길 것 |
| --- | --- | --- |
| 0. 승인 | 사용자 승인 날짜를 실행 기록에 적는다 | 날짜 |
| 1. 지금 값 | `env \| grep -E '^(ROS_\|RMW_\|CYCLONEDDS_\|FASTRTPS_)' \| sort` · `python3 -I scripts/field/preflight.py --host-config config/hosts/<role>.example.json --json <TMP>/before.json` | 바꾸기 전 환경, preflight JSON |
| 2. 백업 | `cp ~/.bashrc ~/.bashrc.bak-<날짜>` · `sha256sum ~/.bashrc ~/.bashrc.bak-<날짜>` | 두 해시가 같은지 |
| 3. 파일 만들기 | `~/.config/rokey7/field-discovery.bash` 에 아래 내용 | 파일 해시 |
| 4. 현장 터미널에서만 | `source ~/.config/rokey7/field-discovery.bash` | — |
| 5. 확인 | `python3 -I scripts/field/preflight.py --host-config ... --json <TMP>/after.json` · `sha256sum ~/.bashrc`(그대로여야 함) | preflight JSON, `.bashrc` 해시 |

```sh
# ~/.config/rokey7/field-discovery.bash — 현장 터미널에서만 source 한다
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp      # Discovery Server 는 Fast DDS 전용 [문서]
unset CYCLONEDDS_URI
# 자리 번호 = 서버 ID. ID 1·2 라면 ";<robot_a-ip>:11811;<robot_b-ip>:11811" (2절 규칙)
export ROS_DISCOVERY_SERVER="<서버 목록>"
export ROS_SUPER_CLIENT=False                   # 조회만 앞에 붙여 쓴다: ROS_SUPER_CLIENT=True ros2 topic list
export ROS_DOMAIN_ID=<Q-01>                     # 서버는 도메인을 보지 않지만 매뉴얼대로 로봇과 맞춘다 [문서]
ros2 daemon stop; ros2 daemon start             # 낡은 목록을 버린다 (4절)
```

역할별 서버 목록은 [`config/hosts/`](../../config/hosts) 예시를 따른다: 로봇 담당 PC 두 대는 자기 로봇 서버 하나, 관제(central)와 인식(perception) PC 는 두 서버 모두 [제안]. 예시의 주소와 ID 는 자리표시자이고, 작전의 프로파일 v2.1 `network` 블록이 들어오면 그 값으로 바꾼다.

**되돌리기.** 이 방식은 `.bashrc` 를 바꾸지 않으므로 그 터미널을 닫으면 끝난다. 열린 터미널에서 되돌릴 때는 `unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT` 뒤 1단계에서 적은 `RMW_IMPLEMENTATION`·`CYCLONEDDS_URI`·`ROS_DOMAIN_ID` 를 다시 `export` 하고 `ros2 daemon stop`. TB4 의 `configure_discovery.sh` 를 이미 썼다면 `.bashrc` 를 2단계 백업으로 되돌리고(해시 확인) `/etc/turtlebot4_discovery/setup.bash` 를 지운다 — 이것은 sudo 라 사람이 한다.

이 PC 의 측정 [2026-10-08]: 셸에 CycloneDDS RMW 와 그 설정 파일, 로봇과 무관한 도메인 값이 들어 있다. 이 PC 를 현장에 쓰면 위 파일이 RMW 를 덮어써야 하고, 쓰지 않는 셸은 그대로 둔다.

## 9. 오프라인 시험 규칙 (로봇 없이 ROS 를 돌릴 때)

1. 시작하는 셸에서 `unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT` 를 하거나, 8절 파일을 source 하지 않은 새 터미널을 쓴다.
2. `ROS_DOMAIN_ID` 는 로봇과 겹치지 않는 값(지금은 10 이상, 팀 작업 규칙 6절), `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
3. 서버가 필요한 시험은 `127.0.0.1` 의 로컬 서버를 직접 띄우고 그 주소만 쓴다(3.1 절 탐침이 예). 끌 때는 자기가 띄운 프로세스 그룹이나 PID 로 끈다. `pkill -f` 금지.
4. 시험 스크립트는 시작할 때 1·2 를 확인하고 아니면 거절한다(`tests/ros/e3_*_probe.py`, `tests/ros/hosts_discovery_probe.py` 가 그렇게 한다). `scripts/run_tests.py` 는 ROS 를 쓰지 않아 해당 없음.

## 10. 실행 전 점검 `scripts/field/preflight.py`

자기 PC 의 실제 값을 **보고만** 한다. 파일·설정·로봇을 바꾸지 않는다. 그래프 조회 때 super client 참가자 하나가 잠깐 서버에 붙는 것이 유일한 흔적이다.

| 점검 | 판정 |
| --- | --- |
| `ROS_DISTRO`·`RMW_IMPLEMENTATION`·`ROS_DISCOVERY_SERVER`·`ROS_DOMAIN_ID` | 호스트 설정(`--host-config`)과 비교. 기대값이 없거나 예시 자리표시자면 `NOT_RUN`. 서버가 있는데 Fast DDS 가 아니면 설정이 없어도 `FAILED` |
| `ROS_SUPER_CLIENT`·`ROS_AUTOMATIC_DISCOVERY_RANGE` | 기록만 (조회 터미널은 True 가 정상이라 판정하지 않음) |
| 설치 패키지 `nav2_msgs`·`irobot_create_msgs`·`turtlebot4_*`·`rmw_fastrtps_cpp`, `fastdds` CLI | source 한 ament 경로에서 찾음. source 안 했으면 `NOT_RUN` |
| 서버 포트 | 빈 UDP 한 개. `refused`·`unreachable` 은 `FAILED`, 응답 없음은 `NOT_RUN`(UDP 는 열림을 증명 못함) |
| `/<ns>/cmd_vel`·`/<ns>/map` 발행자 수 | `ROS_SUPER_CLIENT=True` 를 자식 프로세스에만 주고 `ros2 topic list -v --no-daemon` 한 번. 2 이상이면 `FAILED`(2026-10-06 P4 중복 발행 사고). 그 네임스페이스가 안 보이면 `NOT_RUN` |

결과는 표준 출력 JSON, 사람이 읽는 표는 표준 오류. JSON 의 `host_facts` 에 OS 이름·Python 버전·`/opt/ros/<distro>` 목록을 판정 없이 남긴다(현장 PC 를 고를 때의 입력, Q-02·Q-15). 종료 코드 0 = 전부 `PASSED`, 2 = `FAILED` 하나 이상, 3 = 실패는 없고 `NOT_RUN` 이 있음. 아직 없는 것: 최근 수신 시각(생존 판정), 서버 CPU, 참가자 수 — 다음 PR(4-PC 기동 탐침, AC-01).

이 PC 에서 잰 값 [측정 2026-10-08]: 호스트 셸(Ubuntu 26.04.1, Python 3.14, `/opt/ros` 없음)은 ROS 미설치로 종료 코드 2([preflight-host](../evidence/field-2026-10-08-preflight-host.log)), Jazzy 컨테이너(`--network none`)는 `nav2_msgs`·`irobot_create_msgs` 미설치로 종료 코드 2([preflight-container](../evidence/field-2026-10-08-preflight-container.log)). 둘 다 서버·그래프는 `NOT_RUN`(로봇 없음).

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
