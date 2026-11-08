# ESP_MAIN — Claude 작업 지침

원본 지침은 같은 폴더의 `AGENTS.md`이며 그대로 불러온다. 규칙 변경은 `AGENTS.md`에서 한다.

@AGENTS.md

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

`graphify` is not on PATH on this PC. Run it from `ESP_MAIN` as
`C:/Users/CACONEE/AppData/Roaming/Python/Python310/Scripts/graphify.exe <command>`.
graphify-out/ is local only (listed in `.git/info/exclude`), so rebuild it with `/graphify ESP_MAIN` on another PC.

Rules:
- When graphify-out/graph.json exists, run `graphify query "<question>"` before your first grep, search or source-file read for a codebase question, and put this rule in every subagent prompt that explores code. Read raw files only for lines the graph or an earlier read already located. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
- A committed graphify-out/graph.json is rewritten whole on every build, so a large diff to it in a pull request is expected. Do not flag it as churn or suggest skipping the graph rebuild or its hooks.
