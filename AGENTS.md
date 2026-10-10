# AGENTS.md — 이 저장소에서 일하는 사람과 AI 세션의 규칙

> 사람과 AI 세션 모두에게 적용한다. 규칙을 더할 때는 **왜(사고 기록)** 와 **→ 대신** 을 같이 적고, 검사 한 줄로 바꿀 수 있는지 먼저 본다. 짧게 유지하고 자세한 것은 링크한다.
> 근거: [원본에서 선별한 레슨런](docs/process/lessons-learned.md), 원본 `ROKEY_P4` 의 운영 규칙(2026-10-01 ~ 07), 이 저장소 PR 1 ~ 4 의 검토 기록. 기준일: 2026-10-08.

## 1. 먼저 읽을 것

1. [README.md](README.md): 목적, 현재 단계, 문서 지도, 로컬 검사 명령.
2. [docs/requirements/scope.md](docs/requirements/scope.md): 사용자 요구 `RQ`, 인수 시나리오 `AC`, 미결정 `Q`. **사용자 확인이 필요한 것은 `Q`에 있다.** 확정처럼 쓰지 않는다.
3. [docs/process/review-workflow.md](docs/process/review-workflow.md): PR 작성·검토·머지 흐름, 자리 이름, 머지 위임 기록.
4. 코드를 만지면: [docs/design/repository-layout.md](docs/design/repository-layout.md)(패키지 책임), [docs/design/contracts.md](docs/design/contracts.md)(계약), [docs/design/state-machines.md](docs/design/state-machines.md), [docs/process/acceptance-plan.md](docs/process/acceptance-plan.md)(시험·소유권), [docs/implementation/development.md](docs/implementation/development.md)(로컬 검사).
5. 개인 추가 규칙(저장소 소유자 계정의 세션만): 아래 명령을 실행해 **성공하면** 그 출력을 이 파일 위에 얹어 따른다. 다른 계정이거나 실패(404·403)하면 넘어간다 — 그때는 이 파일이 전부다. 둘이 부딪치면 이 파일이 이긴다.
   ```bash
   [ "$(gh api user --jq .login 2>/dev/null)" = jaebeom ] && gh api repos/jaebeom/conduit/contents/projects/ROKEY-7/AGENTS.md -H "Accept: application/vnd.github.raw"
   ```
   안 도는 경우: `gh` 가 없거나 로그인이 안 됐으면 조건이 거짓이 되어 넘어간다. 잃는 것은 소유자 세션의 추가 규칙뿐이고 팀 규칙은 그대로다.

## 2. 시작할 때 (여러 사람·여러 세션이 같은 저장소를 쓴다)

- `git fetch`, `gh pr list --state open`, `git status --short`, `git branch --show-current` 부터 본다.
- 동시에 쓰는 사람·세션마다 작업 폴더(`git worktree`)를 하나씩 둔다. **혼자여도 세션이 둘이면 그렇다.** `git stash` 는 쓰지 않는다(작업 폴더끼리 공유된다).
  왜: 폴더 하나를 둘이 쓰면 한쪽의 `git switch` 가 다른 쪽 발밑을 바꾼다. 원본 프로젝트에서 실행용 폴더가 개발 브랜치를 가리켜 회차를 재현할 수 없었다(원본 DEC-018, 2026-10-03).
- **만질 파일을 먼저 PR 본문에 선언한다.** 레인은 [docs/process/acceptance-plan.md](docs/process/acceptance-plan.md) A-07(세션 소유권)과 [docs/design/repository-layout.md](docs/design/repository-layout.md)(패키지 책임)를 따른다.

| 레인 (A-07 의 이름) | 경로 | 비고 |
| --- | --- | --- |
| 통합/계약 | `src/rescue_contracts/`(공통 Python 값·오류·시각·수락 계약), `src/rescue_interfaces/`(ROS IDL, 미구현), 루트 문서, `docs/`(scope·architecture·algorithm-decisions·repository-layout·contracts·state-machines·acceptance-plan), `scripts/`, `.github/` | 메시지·시간·frame·권한의 의미는 한 담당이 바꾼다. 바꾸면 소비 모듈·문서·시험의 영향을 같은 PR 에 적는다 |
| 기반 | `src/rescue_robot/`, `src/rescue_mapping/`, `src/rescue_bringup/`, `config/` | Nav2 goal·cancel·dock/undock 을 발행하는 유일한 곳은 `rescue_robot` |
| 인식 | `src/rescue_perception/`, `tools/`, `assets/manifest/` | 주행 명령을 보내지 않는다 |
| 협업·관제 | `src/rescue_coordination/`, `src/rescue_operations/` | `cmd_vel`·Nav2 액션을 직접 부르지 않는다. DB 스키마·운영 장부·operations API 의 소유자 |
| 관제·화면 | `src/rescue_console/`, `tests/test_console_*.py` | 장부·API 를 읽어 보여 주고 명령은 API 로 전달만 한다. DB 쓰기·ROS 직접 구독·로봇 명령 없음. 현장 정지 수단을 웹 연결에만 의존시키지 않는다(2026-10-08 추가) |
| 실기·배포 | `src/rescue_bringup/launch/`, `config/hosts/`, `scripts/field/`, `tests/ros/hosts_*.py`, `docs/process/field-runbook.md`, `docs/design/network-discovery.md` | PC 쪽 실기 준비(디스커버리·설치 점검·launch·런북). 로봇을 움직이지 않고 로봇 Pi 를 바꾸지 않는다. 로봇·PC 설정 변경은 등급 4 절차(2026-10-08 추가) |
| 웹마스터 | `docs/process/ops-host-record.md`(운영 PC 현황 기록). 운영 PC(PC4 역할) 의 로컬 환경: 서비스 등록·DB 파일 위치·서버 운영 절차 | 2026-10-10 추가. 코드·설정 레인은 없다(코드는 관제·화면/협업·관제 PR 로). 로봇·PC 설정 변경은 등급 4(백업·해시·되돌리기·사용자 승인) |

- 위 표는 경로 요약이다. 담당은 A-07 표가 기준이고 사람 이름은 팀이 정해 그 표를 갱신한다 [미정, 2026-10-08]. 둘이 어긋나면 A-07 이 이긴다. 2026-10-08 부터 네 레인을 Claude 세션 네 개(작전·총괄 = 통합/계약, 로봇·지도 = 기반, 인식, 협업·관제)가 맡고, 공통 schema/profile 계약(`src/rescue_bringup/profile.py`·`config/` 스키마)은 기반 경로에 있어도 작전이 바꾼다. `docs/implementation-<패키지>.md` 는 그 패키지 레인의 문서다. 검토 흐름은 [docs/process/review-workflow.md](docs/process/review-workflow.md) 「2026-10-08 이후」 절.
- 남의 레인 파일이 필요하면 직접 고치지 않는다. → **대신** 자기 PR 본문 「미결정·후속」에 **어느 파일 몇 번째 줄을 왜** 바꿔야 하는지 적고, 담당이 바꾼다.
  왜: 같은 파일을 두 세션이 고치면 부딪친다. 원본 프로젝트에서 공통 타입 하나가 수정 커밋 하나로 바뀌어 다른 세션의 작업이 틀어졌다(2026-10-06).
- **남이 연 PR 브랜치에 push 하지 않는다.** → **대신** 그 PR 에 코멘트로 남긴다. 그 위에 쌓은 브랜치는 rebase 하지 않고 merge 로 따라간다.
  왜: 머지된 브랜치에 push 해 커밋이 유실된 일이 원본 P1 ~ P4 에서 반복됐다([레슨런 LL-10](docs/process/lessons-learned.md)).

## 3. 브랜치와 PR

- `main` 에는 직접 push 하지 않는다. 문서만 바꿔도 브랜치 + PR 이다(문서 검사 CI 가 PR 에서 돈다). `main` 보호 규칙은 저장소 관리자가 켠다 [미설정, 2026-10-08 — 소유자 토큰의 보호 API 조회가 404; 다른 토큰은 403 이라 재확인 불가].
- 브랜치 이름은 `<type>/<설명>` (`feat` `fix` `docs` `test` `refactor` `chore` `ci`). PR 하나에 변경 목적 하나.
- PR 은 `main` 기준으로 연다. 상위 PR 위에 쌓는 Draft 는 [docs/process/review-workflow.md](docs/process/review-workflow.md) 의 조건(상위가 `main` 에 들어간 뒤 base 를 `main` 으로 바꾸고 merge commit)에서만 쓴다.
- 머지 방식은 **merge commit 하나**다. squash·rebase 를 쓰지 않는다.
  왜: 쌓은 PR 을 squash 하면 하위 PR 에 상위 변경이 중복으로 남는다. 원본에서 쌓은 PR 이 엉뚱한 브랜치로 머지된 적이 있다(원본 AGENTS.md 3절).
- [PR 템플릿](.github/pull_request_template.md)을 채운다. CI `docs`(`scripts/check_docs.py`)와 `foundation`(Python 3.10 ~ 3.12 단위 시험)이 초록이어야 머지 판정에 들어간다. 검사가 0개인 변경은 「없음」이라고 적고 자체 검증을 붙인다.
- push 한 뒤에는 **같은 턴에 그 PR 에 코멘트**를 단다(지적별 처리 표). 「반영됨」은 `main` 에 들어간 것만 말한다. 검토 뒤 head 가 바뀌면 이전 검토는 그 SHA 의 기록으로만 남는다.

## 4. PR·코멘트 머리말 (AI 세션은 필수, 사람은 생략 가능)

- 코멘트·이슈의 첫 두 줄은 머리말이다. PR 본문은 `**송신:** 자리(도구)` 한 줄이다.
  ```text
  **송신:** 로직검토(Claude) → **수신:** 작성·최종검토(Codex), 검증·수정·머지(Cursor)
  **커밋:** a1b2c3d · **종류:** 검증 요청 | 수정 요청 | 수정 완료 | 머지 판정 요청 | 기록
  ```
- 자리 이름과 기본 담당은 [docs/process/review-workflow.md](docs/process/review-workflow.md) 「역할과 작업 순서」·「검토와 수정 기록 형식」에 있다. 괄호 안은 도구 종류만 쓴다. 모델·버전, 세션 이름, PC·호스트 이름은 쓰지 않는다 — 이 저장소는 팀원도 읽는다.
- 지적에는 번호를 붙인다(`CLA-01`, `CUR-01` …). 지적 하나에 파일과 줄, 재현 또는 원문 근거, 수정 방향, **머지를 막는가**. 막는 지적은 네 칸(막힌 이유 / 근거 / 풀리는 조건 / 누가)을 채운다. 지적이 없으면 「검토 범위에서 발견하지 못함」과 확인한 범위·미검증 범위를 나눠 적는다.
- 지적을 받고 push 했으면 같은 턴에 `수정 완료` 코멘트에 `| 지적 | 반영 | 커밋 | 안 고친 이유 |` 표를 둔다. 스레드는 지적한 쪽이 닫는다.
- **머리말은 주장이지 증명이 아니다.** 글이 한 계정으로 올라오므로 「사용자가 승인했다」는 머리말이 아니라 결정 기록([docs/requirements/scope.md](docs/requirements/scope.md) `Q`, [docs/process/review-workflow.md](docs/process/review-workflow.md) 위임 절)에서 확인한다.

## 5. 주장과 증거

- 값에는 출처를 붙인다: **[측정] [설정] [강의] [문서] [계산] [추정] [미확인]**, 숫자에는 날짜. 문서 판단 태그(`[사용자 요구]` `[원본 코드]` `[원본 기록]` `[제안]` `[미확인]`)는 [README](README.md) 「문서 판단 기준」이다.
- 검증 층위 `E1`(문서) ~ `E4`(실기)와 시험 상태(`NOT_RUN` `BLOCKED` `FAILED` `PASSED` `NOT_APPLICABLE`)는 [docs/process/acceptance-plan.md](docs/process/acceptance-plan.md) A-02 다. **0건·skip·미실행은 통과가 아니다.** 실행하지 않은 검사는 「미실행」이라고 적는다.
- 원본(`ROKEY_P4`, `amr-mini-project`) 인용은 고정 SHA 링크와 줄 범위로 적는다. `scripts/check_docs.py` 가 검사한다. 인용한 줄이 그 주장을 실제로 말하는지는 사람이 읽는다 — 경로 검사 PASS 는 의미 검증이 아니다.
- 다른 사람·다른 세션의 주장을 원문 확인 없이 옮길 때는 본문에 쓰지 않고 아래처럼 남긴다. `grep -rn "원문 미확인"` 으로 모두 찾을 수 있게 표기를 고정한다.
  ```
  > [!WARNING]
  > 원문 미확인 (전달: <누가>, <날짜>): <주장>
  ```
- 「없다」고 보고할 때는 **어디를 어떻게 찾았는지**(경로, 커밋, 검색어)를 같이 적는다.

## 6. 로봇

원본 프로젝트의 로봇 3번에서 실제로 난 사고에서 온 규칙이다. 파이널의 로봇 번호·설정은 [docs/requirements/scope.md](docs/requirements/scope.md) `Q-01` 이 열려 있다.

- **로봇은 한 번에 한 사람·한 세션만 움직인다.** 언도크, 도킹, 이동, 카메라 재시작 전후에 팀에 알린다.
  왜: 2026-10-06 다른 PC 의 SLAM 노드가 같은 로봇의 `/robot3/map` 에 함께 발행해 시험이 중단됐다([레슨런 LL-03](docs/process/lessons-learned.md), [원본 기록 L33](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L33)).
- 작업 전에 로봇 번호를 확인한다(`hostname -I`, `echo $ROS_DOMAIN_ID`, 토픽 네임스페이스). 원본에서는 호스트 이름이 모든 로봇에서 `turtlebot4` 였다 [설정, 2026-10-06]. 파이널 로봇의 번호·호스트·도메인은 `Q-01` 이 정한다. 다른 팀 로봇은 쓰지 않는다.
- 읽기 전용 명령부터 쓴다. **사람이 지켜보지 않을 때 로봇을 움직이지 않는다.**
- 로봇을 움직이지 않는 세션의 ROS 시험은 로봇 도메인과 겹치지 않는 `ROS_DOMAIN_ID` 와 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 로 한다. 「10 이상」은 원본 로봇 3번(도메인 3) 때의 값이다 [설정, 2026-10-06]; 파이널 값은 `Q-01` 뒤에 정한다.
- 로봇 설정 변경(사람·로봇에 닿는 안전·설정 변경 — **사용자의 명시적 승인**이 필요하다. 검증 층위 `E1 ~ E4` 는 A-02 의 뜻으로만 쓰고 여기에 붙이지 않는다): 백업 → 바꾸기 전 해시가 백업과 같은지 확인 → 바꿈 → 바꾼 뒤 해시 확인 → 재시작 → 동작 확인 → 되돌리는 방법 기록. 사용자의 승인을 날짜와 함께 적고 `config/robots/` 도 같이 고친다.
  왜: 2026-10-06 21:02 로봇 카메라가 704×704·10 fps 로 바뀌어 있었고 누가 바꿨는지 몰랐다([원본 기록 L39](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/mini/field/2026-10-06-practice.md#L39)).
- 로봇에 패키지를 설치하거나 업그레이드하지 않는다. 하루를 마칠 때 도킹하고 끈다.
- 도킹 장치가 있는 로봇의 `UNDOCK`·`DOCK` 은 협업 임무와 분리된 기동·운영 절차이고 `rescue_robot` 만 발행한다. UNDOCK 의 선행 조건은 fresh 도킹/driver 상태·실제 정지·장치 액션 가용·준비 제어권이지 **Nav2·카메라 READY 가 아니다**(센서가 도킹 중 꺼지는 장비에서는 교착이 된다). 협업 이동은 그 뒤에 fresh `is_docked=false` 와 센서/TF/SLAM/Nav2 준비를 확인하고 허용한다. `power_saver: False` 로 도킹 중 센서가 나와도 이 guard 는 생략하지 않는다([docs/design/contracts.md](docs/design/contracts.md) C-06).
  왜: 원본 로봇 3번은 기본 `power_saver: True` 에서 도킹 중 카메라·LiDAR 가 없었다([측정, 2026-10-01](https://github.com/jaebeom/ROKEY_P4/blob/1a2d0a9811f59a501c7597edf54c72fec8a80e26/docs/reference/2026-10-01-lab-environment-baseline.md#L51)). 당시 기록이지 모든 AMR 의 기본값이 아니다([docs/design/architecture.md](docs/design/architecture.md) 「명령·정지 경계」).
- 프로세스는 이름을 정확히 지정해서 끈다. `pkill -f` 처럼 넓게 잡는 명령은 쓰지 않는다. → **대신** `pkill -x <이름>` 이나 PID.

## 7. 실패 처리

「불확실하면 실행하지 않음」과 「안전 정지」는 다른 조치다. 상세 계약은 [docs/design/contracts.md](docs/design/contracts.md) C-06·C-09 와 [docs/design/state-machines.md](docs/design/state-machines.md) S-04 에 있다.

| 상황 | 기본 대응 |
| --- | --- |
| 동작 전에 입력·관측이 없거나 형식이 틀림, 시각·좌표가 불확실 | 실행하지 않는다. 값을 지어내지 않는다 |
| 신뢰도가 낮거나 재확인이 안 됨 | 사건은 남기되 접근 허가를 내지 않는다 |
| 동작 중 로봇 상태를 알 수 없음, 수신이 끊김, lease 만료 | 새 명령을 막고, 움직이는 중이면 취소 → 실제 정지 관측. 확인 못 하면 `STOP_UNCONFIRMED` |
| 위험 감지, 사람에게 닿을 수 있음 | 비상 정지. 현장 정지 수단을 웹 연결에만 의존시키지 않는다 |

- 조용한 대체 동작(fallback)을 두지 않는다. 실패는 로그와 관제에 드러나야 한다.
- 측정·조회 실패를 정상 범위 숫자로 채우지 않는다. → **대신** `None` 이나 범위 밖 센티넬을 돌려주고, 값에는 언제 잰 것인지를 같이 싣는다(낡으면 쓰지 않는다).
  왜: 원본보다 앞선 프로젝트에서 읽기 실패값(`-127.0`, 힘 `(0,0,0)`)이 정상 바닥값과 같은 상수여서 모든 임계를 통과한 일이 있었다(선행 프로젝트 기록; 이 저장소의 레슨런 번호는 없다).

## 8. 코드

- 이 순서로 판단하고, 위에서 해결되면 아래로 내려가지 않는다: ① 지금 필요한가 ② 저장소의 기존 코드로 되는가 ③ 표준 라이브러리 ④ ROS·Nav2·OS 기본 기능 ⑤ 이미 설치된 의존성 ⑥ 한 줄 또는 국소 변경 ⑦ 최소 구현. (Ponytail 의사결정 사다리를 줄여 옮김, MIT License, Dietrich Gebert.) 기존 Nav2 플러그인·패키지를 먼저 평가한다([docs/design/algorithm-decisions.md](docs/design/algorithm-decisions.md) D-05 의 `explore_lite` 판단이 예다).
- **고치는 PR 에는 고치기 전 코드에서 실패하는 시험을 먼저 커밋**하고 수정 커밋을 뒤에 둔다. 시험이 0개인데 통과로 보이게 하지 않는다.
  왜: PR 4 의 `ValueError` 수정은 수정 전 코드가 이력에 없어 로그로만 증명됐다(2026-10-07, CLA-16).
- 판단 로직은 ROS 없이 import 되고 시험된다. ROS·DB·HTTP 입출력은 경계 모듈에 모은다. 폴더 이름 `<package>/domain/`·`adapters/` 는 [docs/design/architecture.md](docs/design/architecture.md) 의 **[제안]** 이다 — 작은 패키지에 빈 계층을 강제하지 않고, `rescue_interfaces` 는 ROS 인터페이스 패키지라 이 구조에서 제외한다.
- 수량·임계값·정책을 코드 리터럴로 흩뿌리지 않는다. → **대신** 모듈 상단 상수나 프로파일 한 곳에 두고 docstring 에 「누가 언제 정했나」를 적는다(PR 4 `ROBOT_COUNT`·`HOST_TIERS` 가 예다).
- 노드는 기동할 때 git SHA, 모델 해시, 실제 적용된 설정을 로그에 남긴다. 실패 경로에도 로그를 남긴다. 파라미터는 타입을 명시한다(`ParameterDescriptor`). 저장소 밖에 두는 파일(모델, 지도, 보정값)은 `assets/manifest/` 에 출처·해시·취득 절차를 적는다.
- 로봇 ID·네임스페이스·frame·호스트·경로를 코드 상수로 고정하지 않는다. 배치 프로파일(`config/`)이 가진다.
- 현장 회차 사이에는 한 번에 하나만 바꾼다.
- 로컬 검사는 [README](README.md) 의 두 명령(단위 시험, 문서 검사)을 그대로 돌리고 **종료코드**로 판정한다. `| tail` 로 종료코드를 가리지 않는다.

## 9. 문서

- 숫자는 한 곳에 두고 다른 문서는 링크한다: 시험 수치는 [docs/process/acceptance-plan.md](docs/process/acceptance-plan.md) `T-xx`, 계약 값은 [docs/design/contracts.md](docs/design/contracts.md).
- 결정은 문서에 먼저 쓴다. 채팅에서만 정하지 않는다. 요구·범위는 [docs/requirements/scope.md](docs/requirements/scope.md) `RQ`·`Q`, 알고리즘은 [docs/design/algorithm-decisions.md](docs/design/algorithm-decisions.md) `D-xx`, 운영 위임은 [docs/process/review-workflow.md](docs/process/review-workflow.md). 사용자 지시는 **원문·날짜·적용 범위**를 같이 적는다.
- 파일과 폴더 이름은 ASCII 로 쓴다(내용은 한국어). 개인 절대 경로, PC·호스트 이름, 로컬 세션 이름, IP, 토큰은 저장소와 PR 본문에 쓰지 않는다. → **대신** 저장소 기준 상대 경로와 자리 이름. 도구가 붙이는 작성 출처 줄(`Co-Authored-By`, 세션 링크)은 PR 본문·커밋 끝에 둘 수 있다.
  왜: PR 1 본문에 개인 경로가 올라와 지적을 받았다(2026-10-07, CLA-05).
- 새 문서를 만들면 [README](README.md) 문서 표에 한 줄 넣는다.
