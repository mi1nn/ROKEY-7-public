---
title: 용어집
status: current
updated: 2026-10-14
---

# 용어집

문서와 코드에 나오는 이름·약어·번호를 이 파일 한 곳에 모은다. 다른 문서는 이름을 처음 쓸 때 이 파일로 링크한다. 이름이 바뀌면 이 파일을 먼저 고친다.

## 로봇

| 지금 이름 | 코드 값 | 역할 | 이전 이름 (기록 문서에만 남음) |
| --- | --- | --- | --- |
| **TB-6** | `robot_a`, 네임스페이스 `/robot6` | 탐색 담당(SCOUT). SLAM 으로 지도를 만들며 인형과 출구 QR 을 찾는다 | AMR1, TB1, 로봇 A |
| **TB-5** | `robot_b`, 네임스페이스 `/robot5` | 구조 담당(SUPPORT). 구급상자를 싣고 인형 앞까지 가서 사람을 출구로 이끈다 | AMR2, TB2, 로봇 B |

두 로봇 모두 TurtleBot4(Create 3 + Raspberry Pi 4, OAK-D 카메라, 라이다)다 [문서].

## PC 역할

PC 는 맡은 일로 부른다. 실제 PC 이름과 주소는 저장소에 쓰지 않는다.

| 지금 이름 | 프로파일 값 | 하는 일 | 이전 이름 |
| --- | --- | --- | --- |
| **TB-6 담당 PC** | `compute_a` | TB-6 의 SLAM·Nav2·통로 판정·인형 탐지·지도 고정 | PC1, 고사양 PC1, robot_control_1 |
| **TB-5 담당 PC** | `compute_b` | TB-5 의 위치 추정(AMCL)·Nav2·통로 판정 | PC2, 고사양 PC2, robot_control_2 |
| **운영 PC** | `operations` + `coordination` | 임무 조정기, 현장 상태 DB, 보고 저장, 웹캠 판정, 영상 중계, 관제 웹 | PC4, central |
| **Discovery 서버 PC** | (프로파일에 없음) | 최종 구성에서 ROS 2 발견 서버만 돌린다. 시험 중에는 쓰지 않는다([network-discovery.md](design/network-discovery.md)) | — |

시험 중에는 TB-6 담당 PC 와 TB-5 담당 PC 를 한 PC 에서 돌리기도 한다. 이전 설계의 「웹캠 PC(PC3)」는 지금 운영 PC 에 합쳐졌다.

## 문서 이름

| 줄임말 | 풀어 쓴 이름 | 무엇을 적나 | 파일 |
| --- | --- | --- | --- |
| BRD | 사업 요구 정의서 | 무엇을 왜 만드는지, 시연 시나리오, 성능 목표(KPI) | [brd-v0.2.md](requirements/brd-v0.2.md) (v0.1·Part I 은 이전 판) |
| SRD | 시스템 요구 정의서 | 시스템이 지켜야 할 기능·성능·안전 요구 | [srd-v0.2.md](requirements/srd-v0.2.md) |
| SDD | 시스템 설계서 | 노드·토픽·메시지 연결 | [sdd-node-interface-spec-v0.1.md](design/sdd-node-interface-spec-v0.1.md) |
| scope | 범위와 결정 기록 | 사용자 요구, 지금 시나리오, 결정 대기열 | [scope.md](requirements/scope.md) |

## 번호 체계

| 번호 | 뜻 | 정의한 곳 |
| --- | --- | --- |
| RQ-nn | 사용자 요구 | [scope.md](requirements/scope.md) 「사용자 요구」 |
| Q-nn | 결정이 필요한 질문과 그 결정 | [scope.md](requirements/scope.md) 「미결정 사항」 |
| AC-nn | 인수 시나리오(무엇을 보면 완료로 볼지) | [scope.md](requirements/scope.md), BRD |
| KPI-nn | 성능 목표 | BRD |
| OI-nn | BRD 의 열린 질문 | BRD |
| C-nn | 노드 사이 계약(메시지 모양, 시각, 좌표, 명령 권한) | [contracts.md](design/contracts.md) |
| T-nn | 시험 전에 정할 수치(나이 한도, 시계 오차 등) | [acceptance-plan.md](process/acceptance-plan.md) |
| E1~E4 | 증거 층위. E1 문서, E2 로봇 없이 실행, E3 ROS 통합 시험, E4 실기 | [acceptance-plan.md](process/acceptance-plan.md) |
| 등급 1~4 | 머지 조건 등급. 4 는 사람이나 로봇에 닿는 설정 변경이라 재범 승인이 필요하다 | [AGENTS.md](../AGENTS.md) |
| OPS-nnn | 작전(통합 담당)이 PR 댓글과 지시에 붙이는 일련번호 | PR 댓글 |

## 지도와 위치

| 용어 | 뜻 |
| --- | --- |
| SLAM | 움직이며 지도를 만들고 그 지도에서 자기 위치를 잡는 방법. TB-6 만 한다 |
| AMCL | 이미 있는 지도에서 라이다로 자기 위치를 잡는 방법. TB-5 는 처음부터, TB-6 는 2차 고정 뒤에 쓴다 |
| 지도 고정 (map_freeze) | TB-6 의 SLAM 지도를 저장하고 해시를 붙여 두 로봇이 같은 지도를 쓰게 하는 일. 한 회차에 두 번 한다(인형 확정 때, 탐색 완료 때) |
| revision | 지도 판 번호. 고정할 때마다 하나씩 오른다. 판이 바뀌면 이전 판 기준 좌표를 그대로 쓰지 않는다 |
| frontier (탐색 경계) | 지도에서 이미 본 곳과 아직 못 본 곳의 경계. TB-6 는 가장 가까운 경계로 간다 |
| 출발 자리 | 두 로봇이 출발하는 바닥 표시 자리. TB-6 SLAM 지도의 원점이 된다 |

## 임무 상태와 끝

| 이름 | 뜻 |
| --- | --- |
| DISASTER | 웹캠이 지진 흔들림을 판정한 뒤의 재난 모드 |
| EXPLORATION_COMPLETE | TB-6 가 갈 탐색 경계가 더 없음 |
| MISSION_FINISHED | TB-5 가 출구에, TB-6 가 도킹 접근 지점에 도착해 임무를 마침 |
| NO_SURVIVOR_FOUND | 탐색을 마쳤는데 인형이 없음. TB-5 는 출발하지 않는다 |
| arm | 실행기의 출발 준비. arm 상태가 아니면 로봇은 새 임무를 받지 않는다 |

지금 흐름 전체는 [scope.md 「지금 시나리오」](requirements/scope.md)에 있다.
