# ROKEY-7

AMR 2대가 재난 모사 환경을 수색하며 지도를 갱신하고, 구조 대상자를 발견하면 접근 가능 여부를 판단해 위치와 현장 정보를 보고하는 파이널 프로젝트다.

마스터 PC는 고사양 2대와 중간 사양 2대, 총 4대를 사용한다. 이번 완료 범위는 **대상 발견 → 안전한 접근 → 위치·관측 정보·접근 결과 보고**다. 사람 이송, 안전지대 안내, 구호물품 전달은 이번 범위에서 제외한다.

## 현재 단계

범위·구조·계약 설계에 이어 **두 로봇·네 호스트의 로컬 배포 프로파일 검사**를 구현했다. 실제 동작은 ROS 없는 검사 CLI이며 다섯 패키지는 예약 구조다. SLAM·Nav2·인식·공통 ROS 메시지·노드는 아직 구현하지 않았다. 원본의 실행 코드·가중치·지도·설정은 아직 이식하지 않았다. 문서의 재사용 분류는 검토안이며, 기능 동작이나 실기 검증 완료를 뜻하지 않는다.

| 문서 | 확인할 내용 |
| --- | --- |
| [프로젝트 범위](docs/scope.md) | 사용자 요구, 성공·실패 시나리오, 미결정 사항, 계획 PR 순서 |
| [원본 자산 선별](docs/source-selection.md) | 고정한 원본 커밋, 이식 후보, 바꿔야 할 전제, 제외 근거 |
| [레슨런](docs/lessons-learned.md) | 원본 코드·기록의 교훈과 새 프로젝트에서 확인할 시험 |
| [원본 목록 집계](docs/evidence/source-audit.json) | 고정 커밋의 추적 파일·패키지·시험 파일 집계 |
| [PR 진행과 검토 지침](docs/review-workflow.md) | 병렬 작성, Codex 최종 검토, Claude 로직 검토, Cursor 검증·수정·머지와 원본 URL |
| [디렉터리와 이식 경계](docs/repository-layout.md) | 패키지 책임·원본 후보의 목적지·설정/자산/시험 경계 |
| [구조와 배포](docs/architecture.md) | 2 AMR·4 PC의 책임·데이터 흐름·명령/TF·실행 관문 |
| [알고리즘 결정](docs/algorithm-decisions.md) | SLAM·좌표·탐색·인식·접근·통행의 선택 근거와 대안 |
| [인터페이스 계약](docs/contracts.md) | 관측·시간·좌표·임무·정지·보고·QoS/API의 의미 |
| [상태 전이](docs/state-machines.md) | 로봇·대상·접근·보고·정지와 실패/재시작 처리 |
| [시험과 작업 분담](docs/acceptance-plan.md) | CT/AC 시험·수치 사전 고정·증거·세션 소유권 |
| [개발과 로컬 검사](docs/development.md) | 현재 구현 범위·CLI·프로파일 형식·Python 검사 |

## 문서 판단 기준

- `[사용자 요구]`는 2026-10-07 대화에서 정한 범위다. 요구의 기록은 [범위 문서](docs/scope.md)에 둔다.
- `[원본 코드]`는 링크한 커밋의 정적 관찰이다. 실행 결과로 해석하지 않는다.
- `[원본 기록]`은 과거 문서에 기록된 결과다. 이번 분석에서 재측정한 값이 아니다.
- `[제안]`과 `[미확인]`은 머지된 문서 안에서도 구현·실기 확정 사항이 아니다.
- 두 원본의 과제·미니 프로젝트를 통째로 옮기지 않는다. 필요한 코드와 그 동작을 검증하는 시험을 함께 선별한다.

원본은 [jaebeom/ROKEY_P4](https://github.com/jaebeom/ROKEY_P4)와 [mi1nn/amr-mini-project](https://github.com/mi1nn/amr-mini-project)다. 이후 이식에서도 원본 커밋과 출처를 남긴다.

## 로컬 프로파일 검사

```bash
PYTHONPATH=src/rescue_bringup python3 -m unittest discover -s tests -v
PYTHONPATH=src/rescue_bringup python3 -m rescue_bringup config/deployment.example.json
```

`STATIC CONFIG VALID`는 정적 파일 검사 성공이다. 로봇·ROS 그래프·실제 시계·주행 준비를 검증한 결과가 아니다. 실행 환경·wheel 설치 검증과 미실행 항목은 [개발 안내](docs/development.md)를 따른다.
