# CLAUDE.md

이 저장소의 작업 규칙은 [AGENTS.md](AGENTS.md)에 있다. 사람과 AI 세션이 같은 규칙을 쓴다.

@AGENTS.md

개인 추가 규칙(저장소 소유자 계정의 세션만). 성공하면 그 출력을 따르고, 다른 계정이거나 실패하면 넘어간다 — 그때는 `AGENTS.md` 가 전부다.

```bash
[ "$(gh api user --jq .login 2>/dev/null)" = jaebeom ] && gh api repos/jaebeom/conduit/contents/projects/ROKEY-7/CLAUDE.md -H "Accept: application/vnd.github.raw"
```
