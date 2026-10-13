# ROKEY-7

AMR 2대가 재난 모사 환경을 수색하며 지도를 갱신하고, 구조 대상자를 발견하면 접근 가능 여부를 판단해 위치와 현장 정보를 보고하는 파이널 프로젝트다.

마스터 PC는 고사양 2대와 중간 사양 2대, 총 4대가 예정되어 있다. 현재 1차 설계 범위는 **대상 발견 → 안전한 접근 → 위치·관측 정보·접근 결과 보고**다. 사람 이송, 안전지대 안내, 구호물품 전달은 이 설계에서 제외한다. 로봇 2대 사용은 필수 요구이며, 발견·접근·보고의 최종 인수 범위와 PC 4대 전부의 사용 의무는 아직 확정하지 않았다([범위 Q-08](docs/requirements/scope.md)).

## 현재 단계

범위·구조·계약 설계에 이어 **배포 프로파일 검사, 로봇 명령/정지 판정, 3D 좌표·지도 revision, RGB-D 관측, 임무 배정/통로, 보고 영속 저장의 Python 코어**를 구현했다. 두 논리 로봇을 연결한 합성 입력 리허설도 실행할 수 있다. 공통 ROS 메시지·서비스·액션은 [rescue_interfaces](src/rescue_interfaces/README.md)에 1차 IDL을 정의했다. 실제 SLAM·Nav2·센서/모델 추론·ROS 노드·원격 통신 연결은 미구현이다. 자동 접근은 백엔드 미연동으로 차단한다. 원본 코드는 복사하지 않고 선택한 원리와 실패 경험을 고정 URL로 추적했다. **계획 PR 5~10의 E2 부분 구현이며 두 실제 AMR·4 PC 시연 완료가 아니다.**

| 문서 | 확인할 내용 |
| --- | --- |
| [프로젝트 범위](docs/requirements/scope.md) | 사용자 요구, 성공·실패 시나리오, 미결정 사항, 계획 PR 순서 |
| [원본 자산 선별](docs/process/source-selection.md) | 고정한 원본 커밋, 이식 후보, 바꿔야 할 전제, 제외 근거 |
| [레슨런](docs/process/lessons-learned.md) | 원본 코드·기록의 교훈과 새 프로젝트에서 확인할 시험 |
| [원본 목록 집계](docs/evidence/source-audit.json) | 고정 커밋의 추적 파일·패키지·시험 파일 집계 |
| [PR 진행과 검토 지침](docs/process/review-workflow.md) | 현재 검토 담당·인계·머지 조건, 계획 PR의 과거 운영 기록과 원본 URL |
| [디렉터리와 이식 경계](docs/design/repository-layout.md) | 패키지 책임·원본 후보의 목적지·설정/자산/시험 경계 |
| [구조와 배포](docs/design/architecture.md) | 2 AMR·4 PC의 책임·데이터 흐름·명령/TF·실행 관문 |
| [알고리즘 결정](docs/design/algorithm-decisions.md) | SLAM·좌표·탐색·인식·접근·통행의 선택 근거와 대안 |
| [AMR1 자율 탐색 후보](docs/implementation/amr1-exploration-candidates.md) | 무지도 환경의 탐색 알고리즘 5개, 구현·논문 근거, 비교 순서와 미검증 관문 |
| [인터페이스 계약](docs/design/contracts.md) | 관측·시간·좌표·임무·정지·보고·QoS/API의 의미 |
| [SDD 노드·인터페이스 검토안 v0.1](docs/design/sdd-node-interface-spec-v0.1.md) | 00 통합도: AMR1 지도 직접 공유·AMR2 자동 출동/사람 후행·현장 재확인 필수·역할별 종료 위치의 E2E 이후 PM 요구안. 3-PC 복제 00은 웹캠·관제·DB·Discovery·로컬 브라우저를 PC3에 통합. JSON·CSV·그림 정합 검사, 기존 4-PC 00·01~11쪽은 보존 |
| [시스템 전체 설계도 (draw.io 한 장)](docs/design/rokey7-system-overview.drawio) | 4 PC·2 AMR·웹캠을 기능 묶음 22개와 의미 단위 화살표 20개로 그린 한 장 요약도. 시나리오 순서 ①~⑦ 표시, 통합 노드·통신은 `rokey7-sdd.drawio` 00쪽, 상세는 01~11쪽 |
| [관제 웹·DB 구성 계획](docs/design/console-and-db-plan.md) | PC4 의 operations·web·DB 를 지금 코드 위에 올리는 순서: 승인 관문 2곳, 프로세스·담당, site_state 1.1 테이블, HTTP API, 화면, 10-10 ~ 10-13 순서 |
| [DB 스키마](docs/design/db-schema.md) | SQLite 파일 3개(운영 장부·현장 상태·콘솔 계정)의 표·열·멱등 규칙·쓰는 쪽, 현장 상태 1.1 추가분 [제안] |
| [상태 전이](docs/design/state-machines.md) | 로봇·대상·접근·보고·정지와 실패/재시작 처리 |
| [시험과 작업 분담](docs/process/acceptance-plan.md) | CT/AC 시험·수치 사전 고정·증거·세션 소유권 |
| [이번 수정의 통합 증거](docs/evidence/pr11-round2-validation.md) | exact epoch·수락 인계·소스/설치본 검증과 E2 한계 |
| [OAK-D survivor 학습 계획](docs/implementation/oakd-survivor-training-plan.md) | oakd/image_raw 영상 수집, 1초 간격 추출·전처리, survivor 단일 클래스 학습·파라미터 비교와 결과 문서화 (실행 예정) |
| [YOLO26n·11n·v8n 모델 비교](docs/implementation/yolo-model-comparison/README.md) | 같은 v2 데이터·100 epoch 설정의 지표·추론 시간·곡선 비교 |
| [Survivor v2 재학습 결과](docs/implementation/yolo26n-v2-results.md) | 별도 amr-mini-project 설정, 100/최대200 epoch 평가와 최종 채택 best.pt(100 epoch), 사람 오탐 재평가는 남음 |
| [YOLO26n 학습 결과·비교](docs/implementation/yolo26n-training-results.md) | 무증강·scale_light 결과, 원거리 현장 관측(재학습 전 기록, 최종 모델은 v2) |
| [개발과 로컬 검사](docs/implementation/development.md) | 현재 구현 범위·CLI·프로파일 형식·Python 검사 |
| [BRD Part I v0.1 (팀 원문)](docs/requirements/brd-part1-v0.1.md) | 지진 재난 현장 대응형 실내 탐색 로봇: 카메라 A 재난 감지 → TB1 SLAM·탐색·사람 좌표 DB → TB2 구급상자 출동 → 출구 카운트 → TB1 재확인 → 구출 완료 → 복귀. Phase 1 범위·후순위·수락 기준의 기준 문서. 단, 출구 카메라 B·출구 카운트는 웹캠 1대 결정(2026-10-09, [#44 OPS-20 ②](https://github.com/jaebeom/ROKEY-7/pull/44#issuecomment-6078811090))으로 빠졌다. 웹캠 하나가 재난 징후와 대상 감시를 맡고, 재확인은 대상 소실 후보로 시작한다 |
| [BRD v0.1 (팀 초안 원문)](docs/requirements/brd-v0.1.md) | 지진 트리거 → TB1 정찰 → TB2 구조 지원 → 출구 카운트 → 재확인 시나리오의 팀 요구 문서 |
| [BRD v0.2 (제안)](docs/requirements/brd-v0.2.md) · [SRD v0.2 (제안)](docs/requirements/srd-v0.2.md) | 웹캠 1대·운영자 구간 승인·머리 위 기둥 통로·이중 확인 완료 판정 시나리오를 교육 양식 목차로 쓴 BRD·SRD. 채택 전 `[제안]`, 기준 문서는 Part I v0.1 |
| [네트워크·Discovery Server](docs/design/network-discovery.md) | Fast DDS Discovery Server 조사와 채택안(토폴로지 A), 로컬 측정, PC 별 설정·되돌리기, 오프라인 시험 규칙, 실행 전 점검 `scripts/field/preflight.py` |
| [운영 PC 현황 기록](docs/process/ops-host-record.md) | 운영 PC 에 실제로 떠 있는 Discovery Server(서버 ID 0, 손으로 띄움, DHCP 주소)·관제 웹의 측정값, network-discovery 권고안 A 와의 차이, 로봇 Discovery 설정을 건드리지 않는 규칙 |
| [운영 DB 파일과 테이블 관계도](docs/process/ops-db-tables.md) | SQLite 파일 넷(송신 대기열·운영 장부·현장 상태·웹 계정)의 쓰는 쪽·읽는 쪽, 테이블과 JSON 안 ID 로 이어지는 관계(Mermaid 그림), 읽기 전용으로 여는 법 |
| [작업 묶음](docs/process/work-bundles.md) | BRD 와 기존 설계의 대응, 세션별 첫 PR, 공통 규격, 사용자 확인 항목, 통합 순서 |
| [일정과 담당](docs/process/schedule.md) · [일정표 CSV](docs/process/schedule-timeline.csv) | 10-08 ~ 10-16 작업일·관문·사람별 담당(임시안), 구글 시트로 가져올 칸 일정 |
| [페이즈와 릴리즈 계획](docs/process/release-plan.md) | P0 ~ P5 페이즈, v0.1 ~ v1.0 릴리즈에 들어가는 것·빠지는 것·합격 조건, P1 후보 PR 대응, 대비책 |
| [작업 규칙](AGENTS.md) | 사람·AI 세션 공통 규칙: 레인, 브랜치·PR, 머리말, 증거, 로봇, 실패 처리 |

## 통합 리허설과 남은 실기 연결

[통합 구현 기록](docs/implementation/implementation-integration.md)의 명령으로 `summary.json`과 SQLite 보고/재전송 DB를 새 출력 폴더에 생성한다. 합성 A→B/B→A 제안, 수색 취소·정지→새 generation→실행기 수락 receipt, 자동 접근 차단, 원 관측의 나노초 시각 보존, 지도 보정, 통로 점유, 보고 복구를 검증한다. 수색 구역 분담·frontier·자동 역할 교대는 [F-01 후속](docs/requirements/scope.md)에 남아 있다. [실기 연결 관문](docs/process/field-runbook.md)에 ROS·backend·2 AMR·4 PC 준비 및 E3/E4 미실행 항목을 모았다.

## 문서 판단 기준

- `[사용자 요구]`는 2026-10-07 대화에서 정한 범위다. 요구의 기록은 [범위 문서](docs/requirements/scope.md)에 둔다.
- `[원본 코드]`는 링크한 커밋의 정적 관찰이다. 실행 결과로 해석하지 않는다.
- `[원본 기록]`은 과거 문서에 기록된 결과다. 이번 분석에서 재측정한 값이 아니다.
- `[제안]`과 `[미확인]`은 머지된 문서 안에서도 구현·실기 확정 사항이 아니다.
- 두 원본의 과제·미니 프로젝트를 통째로 옮기지 않는다. 필요한 코드와 그 동작을 검증하는 시험을 함께 선별한다.

원본은 [jaebeom/ROKEY_P4](https://github.com/jaebeom/ROKEY_P4)와 [mi1nn/amr-mini-project](https://github.com/mi1nn/amr-mini-project)다. 이후 이식에서도 원본 커밋과 출처를 남긴다.

## 로컬 프로파일 검사

```bash
python3 -I scripts/run_tests.py
PYTHONPATH=src/rescue_bringup python3 -m rescue_bringup config/deployment.example.json
```

`STATIC CONFIG VALID`는 정적 파일 검사 성공이다. 로봇·ROS 그래프·실제 시계·주행 준비를 검증한 결과가 아니다. 실행 환경·wheel 설치 검증과 미실행 항목은 [개발 안내](docs/implementation/development.md)를 따른다.

## 문서 검사

```bash
python3 -I scripts/check_docs.py
# 고정 SHA의 깨끗한 원본 체크아웃이 형제 폴더에 있을 때:
python3 -I scripts/check_docs.py --sources ../ROKEY_P4 ../amr-mini-project
```

기본 검사와 `docs / docs-local` CI는 로컬 링크·원본 SHA 표기·줄 범위 형식을 검사한다. 비공개 원본 접근이 필요한 파일 존재·실제 줄 상한·목록 집계는 `--sources`로 검증하며 기본 모드에서는 `SKIP`이다. 문장의 의미·웹 가용성·로컬 제목 앵커·ROS·실기를 대신 검증하지 않는다.
