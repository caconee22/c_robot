"""Export the Claude Code conversations of this project to readable Markdown.

Reads every session file (~/.claude/projects/D--c-robot/*.jsonl). A continued
session repeats the earlier messages, so records are de-duplicated by their id
and written in time order, one Markdown file per day (KST): the user's
messages, Claude's answers, and one short line per tool call (tool name + its
description, no tool output). System reminders, hook text and tool results are
left out (they can hold file contents, logs or secrets). Private data is
masked (see SCRUB).

  python conversation_logs/export_transcripts.py [jsonl folder] [output folder]

Written 2026-11-14 to keep the 2026 IRC development record (IRC2026_RETROSPECTIVE.md).
"""
import glob
import json
import os
import re
import sys
from collections import defaultdict
from datetime import datetime, timedelta, timezone

SRC = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/.claude/projects/D--c-robot")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(os.path.abspath(__file__))
KST = timezone(timedelta(hours=9))

# Private data that must not reach git.
SCRUB = [
    (re.compile(r"[A-Za-z0-9._%+-]+@gmail\.com"), "[email]"),
    (re.compile(r"(?i)\b(psk|password|passwd)\s*[=:]\s*\S+"), r"\1=[masked]"),
    (re.compile(r"tskey-[A-Za-z0-9-]+"), "[tailscale-key]"),
    (re.compile(r"(?i)\b(ghp|github_pat)_[A-Za-z0-9_]+"), "[token]"),
]
WRAPPED = re.compile(r"<(system-reminder|local-command-[a-z]+|command-[a-z]+|task-notification)>.*?</\1>", re.S)


def scrub(text: str) -> str:
    for pattern, repl in SCRUB:
        text = pattern.sub(repl, text)
    return text


def stamp(ts: str):
    try:
        return datetime.fromisoformat(ts.replace("Z", "+00:00")).astimezone(KST)
    except (ValueError, AttributeError):
        return None


def records():
    """Unique user/assistant records from every session file, in time order."""
    seen, out = set(), []
    for path in sorted(glob.glob(os.path.join(SRC, "*.jsonl"))):
        for raw in open(path, encoding="utf-8"):
            try:
                rec = json.loads(raw)
            except ValueError:
                continue
            if rec.get("type") not in ("user", "assistant") or rec.get("isMeta"):
                continue
            key = rec.get("uuid") or (rec.get("timestamp"), raw[:200])
            when = stamp(rec.get("timestamp", ""))
            if key in seen or when is None:
                continue
            seen.add(key)
            out.append((when, rec))
    out.sort(key=lambda item: item[0])
    return out


def render(when, rec):
    content = (rec.get("message") or {}).get("content")
    t = when.strftime("%H:%M")
    if rec["type"] == "user":
        if isinstance(content, str):
            text = content
        elif isinstance(content, list):
            text = "\n".join(c.get("text", "") for c in content if c.get("type") == "text")
        else:
            return []
        text = WRAPPED.sub("", text).strip()
        if not text or text.startswith("<") or "This session is being continued from a previous" in text:
            return []
        return [f"\n### 🧑 학현 ({t})\n\n{scrub(text)}\n"]
    parts = []
    for c in content if isinstance(content, list) else []:
        if c.get("type") == "text" and c.get("text", "").strip():
            parts.append(f"\n**🤖 Claude** ({t})\n\n{scrub(c['text'].strip())}\n")
        elif c.get("type") == "tool_use":
            inp = c.get("input") or {}
            what = inp.get("description") or inp.get("file_path") or inp.get("pattern") or ""
            parts.append(f"- `{c.get('name', '')}` {scrub(str(what))[:160]}")
    return parts


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    for old in glob.glob(os.path.join(OUT, "*.md")):
        if os.path.basename(old) != "README.md":
            os.remove(old)
    days = defaultdict(list)
    users = defaultdict(int)
    for when, rec in records():
        lines = render(when, rec)
        if not lines:
            continue
        day = when.strftime("%Y-%m-%d")
        days[day].extend(lines)
        if rec["type"] == "user":
            users[day] += 1
    index = []
    for day in sorted(days):
        name = f"{day}.md"
        with open(os.path.join(OUT, name), "w", encoding="utf-8") as f:
            f.write(f"# Claude Code 대화 기록 {day} (KST)\n\n"
                    f"사용자 메시지 {users[day]}개. 도구 결과·시스템 메시지는 빼고, 개인정보는 가렸다.\n")
            f.write("\n".join(days[day]) + "\n")
        index.append((day, name, users[day]))
    with open(os.path.join(OUT, "README.md"), "w", encoding="utf-8") as f:
        f.write("# C_ROBOT ↔ Claude 대화 기록 (2026 IRC)\n\n"
                "2026 IRC 씨름 로봇 개발 중 Claude Code와 나눈 대화를 날짜별로 정리했다(시간 KST).\n"
                "사용자 메시지와 Claude 답변, 도구 호출 이름만 남겼다. 도구 결과와 원본 세션 파일(.jsonl)은\n"
                "넣지 않았다(원본은 작성자 노트북 `D:\\c_robot_pi_backup\\claude_transcripts\\`).\n"
                "다시 만들기: `python conversation_logs/export_transcripts.py`.\n\n"
                "| 날짜 | 파일 | 사용자 메시지 |\n|---|---|---|\n")
        for day, name, count in index:
            f.write(f"| {day} | [{name}]({name}) | {count} |\n")
    print("\n".join(f"{d} {n} {c}" for d, n, c in index))


if __name__ == "__main__":
    main()
