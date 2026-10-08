---
title: 2 AMR·예정 4 PC 현장 실행 전 관문
status: proposed-field-runbook-not-run
updated: 2026-10-08
---

# 현장 실행 전 관문

현재 실행 가능한 통합 산출물은 [오프라인 E2 데모](implementation-integration.md)다. 현장 ROS launch, 실제 드라이버/SLAM/Nav2 adapters, 모델 추론과 센서 검증, 물리적 정지 확인은 아직 완성/실행하지 않았다. 아래는 현장에서 구현·측정할 순서이며 이번 PR에서 성공한 실기 절차가 아니다. 실행할 수 없는 ROS 명령을 예시로 만들어 넣지 않는다.

실제 AMR 2대 활용은 사용자 확정 요구다. 고사양 PC 2대·중간 사양 PC 2대는 예정 자원이고 네 대를 모두 쓰는 것은 현재 배포 기준안의 검증 조건이다. 발견→접근→보고와 4-PC 전체 사용의 최종 인수 의무는 [scope Q-08](scope.md)에 남긴다. 조건이 바뀌면 배치·프로파일·AC를 함께 수정한다.

## 1. 실행 기록과 장비 확인

현장 담당과 통합 담당이 두 로봇과 네 PC의 실제 식별자·CPU/GPU/RAM·OS·ROS/패키지/드라이버/펌웨어 버전·센서·네트워크를 기록한다. 고사양 PC별 SLAM/Nav2·추론, 중간 PC별 협업/관제 배치는 부하 측정 전 가설이다. 저장소의 example 이름을 실제 장비 확인으로 사용하지 않는다.

매 run에 run ID, 코드 전체 SHA·dirty diff, 모든 호스트의 배포 SHA, 프로파일/로봇별 parameter hash, 모델/데이터/보정 hash, 시계 상태, map session/revision·transform revision, 실제 명령, 원시 로그·영상·지도·관측/임무/보고 ID, 미실행 이유를 연결한다. 날짜만 같은 결과를 같은 실행으로 묶지 않는다. 비공개 원본을 확인하지 못하면 해당 출처 검증은 미확인으로 남긴다.

먼저 아래 **로컬 E2 명령**을 저장소 루트에서 실행할 수 있다. 새 output 경로를 사용한다.

```sh
export PYTHONPATH=src/rescue_bringup:src/rescue_robot:src/rescue_mapping:src/rescue_perception:src/rescue_coordination:src/rescue_operations:src/rescue_contracts
python3 -m rescue_bringup config/deployment.example.json
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 -m rescue_bringup.offline_demo config/deployment.example.json --output work/offline-run-001
```

이 명령은 호스트에 서비스를 배포하지 않는다. 설정의 `runtime_enabled=false`를 true로 편집하는 것만으로 실행이 활성화되는 구조도 아니다. 실제 launch/adapters를 구현하고 해당 검사를 통과한 뒤 문서에 검증한 실기 명령을 추가해야 한다.

## 2. E3 연결과 제어 권한

| 관문 | 준비/시험 | 다음 단계에 필요한 증거 |
| --- | --- | --- |
| 그래프·장치 소유 | 단일 그래프 A 또는 로봇별 domain+API B 선택. 호스트별 domain·Discovery Server 목록/고유 ID·ROS_SUPER_CLIENT·participant/구독자 수 기록 | 실제 메시지 수신 시각·요청/응답·재연결, namespace/TF buffer/명령 경로 분리. 목록 조회만으로 PASS 금지 |
| 준비/도킹 | 장치 driver·watchdog, 별도 권한의 UNDOCK/DOCK adapter 구현. power_saver/도킹 중 센서 발행 조건 확인 | 접수 후 무동작·timeout과 실제 dock status 확인. 센서 READY와 이동 허가 분리; 이동 전 fresh is_docked=false |
| 단독 명령 소유 | 주행·도킹 요청은 rescue_robot 실행기 하나가 발행. 탐색기는 후보만 제공 | competing goal/속도 발행자 없음. 수락 지연·늦은 terminal·cancel·lease/authority/boot fencing과 독립적인 최종 명령 억제 경로 |
| 시각/센서 | 원 sec/nanosec·clock domain·stamp 의미·오차/드라이버 지연 상한 측정. RGB/Depth/CameraInfo·정렬·encoding/단위 확인 | 0/미래/역순/UNKNOWN clock 거절, 올바른 원 시각 TF 조회, 수신 지연이 원 관측 수명을 연장하지 않는 기록 |

P4는 로봇별 domain과 서버를 기록했고 단일 그래프 A는 당시 제안이었다([DEC-010](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/decisions.md#L142-L151)). 다른 PC의 `/robot3/map` 발행자 2개는 [실습 L33](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L33), Discovery Server CPU 92%는 [실습 L34](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L34)다. L24–L32의 가짜 controller, 시계 점검, 언도크, 탐색 성공 행은 이 두 사고의 근거가 아니다. 이 값은 이번 네 PC의 재현 결과가 아니므로 새 구성에서 다시 측정한다.

## 3. 현장 수치·좌표·탐지 관문

[시험 계획의 T-01~14](acceptance-plan.md)를 단위·근거·정책 version/hash·결정 시각과 함께 `LOCKED_FOR_RUN`으로 확정한다. 현재 코드의 synthetic fixture 수치는 근거가 아니다. 합격 기준을 실행 결과에 맞춰 사후 낮추고 같은 회차를 PASS로 바꾸지 않는다. 값이 비어 있거나 stamp 의미/시계 오차가 미확인이면 그 값에 의존하는 자율 접근은 준비되지 않은 상태다.

독립 SLAM과 물리 기준점 측정부터 시작한다. 로봇별 TF·map 소유, A/B의 같은 기준점 오차, 다른 출발 방향, pose drift와 실제 backend의 loop correction 이벤트/posegraph 연결을 확인한다. 임의 revision 증가나 일반 map→odom 변화만으로 보정 감지 성공을 주장하지 않는다. 영향 관측을 재투영할 근거가 없으면 재관측으로 전환하며, A/B 원 관측별 provenance를 보존한다.

현재 mapping의 `auto_approach_eligible`은 false이며 운영 준비 provider가 없다. 실제 backend correction과 readiness/권한/시간/좌표를 연결한 adapter·실패 처리를 구현하고 검증하기 전에는 교차 로봇 접근을 허가하지 않는다. 좌표가 맞는 것과 명령을 보내도 되는 것은 별도 관문이다.

사람/시연 인형·촬영 세션을 분리한 평가 데이터로 모델 class·confidence·track 의미, Depth 결측·배경 혼입, 위치 오차/공분산, 가림·조명을 검증한다. 객체의 점 자체를 goal로 보내지 않고 자유 공간·footprint·불확실성·제동/보호 영역과 실제 경로를 평가한다. 전송 FPS·합성 타일·기존 car/dummy 점수는 이 평가를 대신하지 않는다.

## 4. 제한된 공간의 E4 실행 순서

CT-06·CT-07의 순수 코어 E2 부분은 [통합 리허설](implementation-integration.md)에서 수색 SEND 의도→취소 요청→terminal CANCELED→합성 정지 증거→새 generation→접근 admission→실제 TaskAdmission으로 배정 승격까지 연결했다. 자동 접근 SEND는 여전히 차단된다. 이 결과는 ROS action 취소/terminal 전달, 실제 odometry의 정지 판정, 접근 주행·재관측을 검증하지 않는다. 아래 E3/E4 관문은 별도로 구현하고 실행해야 한다.

현장 실행 담당은 합의한 속도·영역·정지 수단으로 먼저 한 로봇의 최종 명령 차단·fresh stop 관측을 검증하고, 두 로봇의 분리된 수색과 공통 좌표 관문을 이어간다. 이후 [AC-01~07](acceptance-plan.md)에 따라 현재 배포 기준안의 실제 네 PC 부하에서 역할을 교환한다.

1. 두 로봇 센서/TF/SLAM·명령 분리와 host별 부하·수신/처리 지연을 기록한다.
2. A 발견→B 수색 취소→정지 확인→새 generation·후보 경로→B 접근·fresh 재관측·이격/정지→보고 저장을 검증한다. A의 별도 수색 유지/재개 정책과 역할 교환도 확인한다.
3. 장애물 추가/제거를 센서→costmap→경로→SLAM 각 층에서 측정한다. 프로세스 생존만으로 변화 대응을 통과시키지 않는다.
4. E2/E3에서 먼저 검증한 관측 만료·SLAM 보정·센서/TF·lease·연결 단절·늦은 action·좁은 통로 점유 실패를 현장 조건 안에서 확인한다. 정지 요청/응답·terminal fence·실제 정지 증거와 재개 권한을 구분한다.
5. 접근 불가·미확인도 원인과 발견 증거를 보고한다. Q-07에서 정할 사진 정책과 실제 evidence bytes·receipt를 확인하고, 응답 유실/재시작 뒤 같은 report ID 복구를 검증한다.

`CONFIRMATION` 사진 정책은 재확인하지 못한 보고를 PENDING_EVIDENCE에 남긴다는 현재 구현 한계가 있다. 실패 보고의 완료까지 요구하는 운영에서는 정책을 그대로 채택하지 않고 사전에 보고 종류별 요구와 구현을 맞춘다. 사진 정책·운영 scheduler·원격 인증·증거 보존/용량은 E2 데모가 결정하지 않는다.

각 단계는 NOT_RUN/BLOCKED/FAILED/PASSED와 원문 증거를 남긴다. 실제 두 로봇 사용을 한 로봇 재생으로 대체하지 않으며, 네 PC 기준안 시험을 단일 프로세스의 네 host 이름으로 대체하지 않는다. 필수 관문이 미실행이면 해당 실기 완료 주장은 남겨 둔다.
