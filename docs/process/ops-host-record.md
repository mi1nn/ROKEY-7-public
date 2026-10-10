---
title: 운영 PC 현황 기록 — 관제 웹·DB·Discovery Server
status: record   # 측정한 현재 상태. 설계 결정이 아니다
updated: 2026-10-10
---

# 운영 PC 현황 기록 (웹마스터, 2026-10-10)

운영 PC(PC4 역할: operations·web·DB, [관제 웹·DB 구성 계획](../design/console-and-db-plan.md) 2절)에 **지금 실제로 무엇이 떠 있는지**를 적는다. 이 PC 의 로컬 상태를 아는 세션이 하나(웹마스터)뿐이라 여기에 남긴다. 값은 [측정, 2026-10-10] 이고, IP·PC 이름·개인 경로는 팀 [AGENTS.md](../../AGENTS.md) 9절대로 `<OPS_HOST_IP>` 같은 자리 이름으로 바꿨다.

## 0. 결론

1. **운영 PC 가 Fast DDS Discovery Server(서버 ID 0, UDP 11811)를 돌린다.** 두 로봇도 Discovery Server 로 설정되어 있다 — 로봇 쪽 구체 값은 [미확인](아래 3절).
2. **이 구성은 [network-discovery.md](../design/network-discovery.md) 0절의 권고안 A(로봇마다 Pi 온보드 서버, PC 는 두 서버를 목록에 적음)와 다르다.** 운영 PC 의 클라이언트 목록에는 자기 자신의 서버 하나만 있다. 어느 쪽이 팀 결정인지는 문서에 없다 [미확인].
3. **서버는 서비스가 아니라 터미널에서 손으로 띄운 프로세스다.** 그 터미널이 닫히거나 PC 가 재부팅되면 서버가 꺼지고, 새로 뜨는 노드는 아무것도 찾지 못한다(이미 맺은 연결은 유지, network-discovery.md 0절 4번). 바꾸지 않았다(2절 규칙).
4. **운영 PC 주소는 DHCP(자동)다.** 서버 주소가 IP 로 고정되어 있으므로 주소가 바뀌면 이 PC 와 로봇 쪽 설정이 함께 깨진다. 공유기 예약 여부는 [미확인].

## 1. 측정값 (2026-10-10, 읽기 전용 조회만)

| 항목 | 값 | 확인 방법 |
| --- | --- | --- |
| 서버 프로세스 | `fastdds discovery --server-id 0 --ip-address <OPS_HOST_IP> --port 11811` (자식 `fast-discovery-server`) | `ps` |
| 기동 | 2026-10-08 19:03, 사용자 터미널에서 직접 실행(부모가 대화형 bash, 출력이 그 터미널). systemd 유닛 없음 | `ps`·`/proc/<pid>/fd`·`systemctl list-units` |
| 수신 | UDP `0.0.0.0:11811` | `ss -lunp` |
| 서버 바이너리 버전 | Ubuntu `fastdds-tools` 2.11.2 (`/usr/bin/fastdds`) | `dpkg -l` |
| ROS 쪽 Fast DDS | `ros-jazzy-fastrtps` 2.14.6, `rmw_fastrtps_cpp` 8.4.4 | `dpkg -l` |
| 클라이언트 설정 | `/etc/turtlebot4_discovery/setup.bash`(TB4 `configure_discovery.sh` 형식): `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, 대화형 터미널만 `ROS_SUPER_CLIENT=True`, `ROS_DOMAIN_ID=6`, `ROS_DISCOVERY_SERVER="<OPS_HOST_IP>:11811;"` | 파일 읽기. sha256 앞 16자 `10d36e65ce000207` |
| 셸 | 사용자 `.bashrc` 가 위 파일을 `source` 하고 `ROS_DOMAIN_ID=6` 을 한 번 더 export | 파일 읽기 |
| 주소 할당 | 무선, IPv4 method `auto`(DHCP) | `nmcli` |

- 서버 바이너리(2.11.2)와 ROS 클라이언트(2.14.6)의 Fast DDS 버전이 다르다. 지금 동작에 문제가 있다는 관측은 없다. 문제가 생기면 먼저 볼 곳으로 적어 둔다 [측정 / 영향 미확인].
- 이 PC 의 셸은 기본으로 실제 로봇 그래프에 붙는다(`ROS_DISCOVERY_SERVER` 설정됨). 로봇을 움직이지 않는 시험은 팀 AGENTS.md 6절대로 다른 `ROS_DOMAIN_ID` 와 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 로, 그리고 network-discovery.md 0절 5(c)대로 `ROS_DISCOVERY_SERVER` 를 비우고 한다.

## 2. 규칙 (2026-10-10)

> [!WARNING]
> 원문 미확인 (전달: 사용자, 2026-10-10): 로봇의 Discovery Server 설정은 최대한 건드리지 않는다.

- 위 전달에 따라 **로봇 쪽 Discovery Server 설정은 바꾸지 않는다.** 운영 PC 쪽 서버 재시작·서비스 등록도 로봇 통신을 잠깐 끊으므로 같은 급으로 다룬다.
- 운영 PC 서버를 서비스로 바꾸는 작업은 **로봇이 쉬는 시간에만** 한다(사용자 지시 2026-10-10: "로봇이 쉬는 시간에 작업을 하자"). 그 전에 팀에 알리고, 팀 AGENTS.md 6절 설정 변경 절차(백업 → 해시 → 변경 → 해시 → 재시작 → 동작 확인 → 되돌리는 방법)를 따른다. 아직 하지 않았다.
- 그 작업 전까지 **서버를 띄운 터미널을 닫지 않는다.** 서버가 꺼졌으면 같은 ID·포트로 다시 띄운다(같은 명령, network-discovery.md 0절 4번: 2초 안에 기존 노드가 다시 보임 [측정, localhost 값]).

## 3. 모르는 것

| 무엇 | 왜 필요한가 | 확인할 사람 |
| --- | --- | --- |
| 두 로봇의 Discovery Server 설정(온보드 서버 ID, 오프보드 서버로 운영 PC 를 적었는지, 도메인) | 운영 PC 서버가 꺼질 때 로봇끼리·로봇과 다른 PC 가 여전히 서로 보이는지가 여기에 달렸다 | 로봇·지도, 실기·배포 (로봇 설정을 바꾸지 않고 읽기만) |
| 이 중앙 서버 구성이 팀 결정인가, 권고안 A 로 갈 것인가 | network-discovery.md 가 A 를 권고한 채로 남아 있다 | 실기·배포 → 작전 |
| 운영 PC 주소의 공유기 예약 | DHCP 로 바뀌면 서버 주소가 깨진다 | 사용자 |
| 다른 PC(고사양 2대 등)의 클라이언트 목록 | 같은 서버를 쓰는지 | 실기·배포 |

## 4. 같은 PC 의 다른 프로세스 (참고)

- 관제 웹(`rescue_console`, PR #37 브랜치)이 TCP 8080 에서 뜬다. 사용자 서비스로 등록되어 재부팅 뒤에도 뜨고, 기동할 때 실행 중인 git SHA 를 로그에 남긴다. ROS 를 쓰지 않아 Discovery Server 와 무관하다. 지금 데이터는 오프라인 리허설의 합성 장부다(E2).
- 운영 장부·현장 상태 DB 는 로컬 디스크(ext4)에 둔다. 네트워크 공유 금지는 [implementation-operations.md](../implementation/implementation-operations.md) 의 운영 경로 규칙을 따른다.
