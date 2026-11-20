#!/usr/bin/env python3
"""Helpers for the `tourney` CLI: templating, metrics, freeze checks, summaries.

Standard library only. Invoked as: python3 -I lib.py <subcommand> [args].
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path


# ------------------------------------------------------------------ utilities
def git(cwd: str | Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(cwd), *args], capture_output=True,
                          text=True, check=True).stdout


def tracked_files(cwd: str | Path, paths: list[str]) -> list[str]:
    if not paths:
        return []
    out = git(cwd, "ls-files", "-z", "--", *paths)
    return sorted(p for p in out.split("\0") if p)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_json(path: str | Path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def read_mapping(rd: Path) -> dict[str, str]:
    m = {}
    f = rd / "mapping.tsv"
    if f.exists():
        for line in f.read_text().splitlines():
            if "\t" in line:
                k, v = line.split("\t", 1)
                m[k] = v
    return m


def agent_stats(log_path: str | Path) -> dict:
    d = load_json(log_path, {}) or {}
    if isinstance(d, list):  # stream-json fallback: last result event
        d = next((e for e in reversed(d) if isinstance(e, dict) and e.get("type") == "result"), {})
    return {
        "cost_usd": d.get("total_cost_usd"),
        "turns": d.get("num_turns"),
        "duration_s": round(d["duration_ms"] / 1000) if d.get("duration_ms") else None,
        "is_error": d.get("is_error"),
        "subtype": d.get("subtype"),
    }


def fmt(v, nd=2):
    if v is None:
        return "–"
    if isinstance(v, bool):
        return "✅" if v else "❌"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def md_table(headers: list[str], rows: list[list]) -> str:
    out = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    out += ["| " + " | ".join(fmt(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


# ------------------------------------------------------------------ commands
def cmd_render(a):
    """Replace {{NAME}} with $TV_NAME. Unknown placeholders are left visible."""
    text = Path(a.template).read_text()

    def sub(m):
        return os.environ.get("TV_" + m.group(1), m.group(0))

    sys.stdout.write(re.sub(r"\{\{([A-Z_]+)\}\}", sub, text))


def cmd_agent_stats(a):
    s = agent_stats(a.log)
    if a.oneline:
        print(f"cost=${fmt(s['cost_usd'])} turns={fmt(s['turns'])} "
              f"{'ERROR ' + str(s['subtype']) if s['is_error'] else ''}".strip())
    else:
        print(json.dumps(s))


def cmd_detect_config(a):
    repo = Path(a.repo)
    presets = {
        "Cargo.toml": dict(
            BUILD_CMD="cargo build --all-targets --quiet",
            TEST_CMD="cargo test --no-fail-fast --quiet",
            LINT_CMD="cargo clippy --all-targets --quiet",
            LINT_COUNT_PATTERN="^(warning|error)",
            SCOPE_PATHS="src", FROZEN_PATHS="SPEC.md tests", LOC_EXTENSIONS="rs",
            TOOLS="Bash(cargo *)",
            EXTRA=['"unwraps|grep -rho \'\\.unwrap()\' src | wc -l"',
                   '"unsafe|grep -rhow unsafe src | wc -l"',
                   '"deps|cargo tree --depth 1 -e normal --quiet 2>/dev/null | tail -n +2 | wc -l"']),
        "package.json": dict(
            BUILD_CMD="npm run build --if-present", TEST_CMD="npm test --silent",
            LINT_CMD="npx --no-install eslint . || true", LINT_COUNT_PATTERN="(warning|error)",
            SCOPE_PATHS="src", FROZEN_PATHS="SPEC.md test", LOC_EXTENSIONS="ts tsx js jsx",
            TOOLS="Bash(npm *) Bash(npx *) Bash(node *)",
            EXTRA=['"any_types|grep -rhoE \':\\s*any\\b\' src | wc -l"']),
        "pyproject.toml": dict(
            BUILD_CMD="", TEST_CMD="python -m pytest -q", LINT_CMD="ruff check . || true",
            LINT_COUNT_PATTERN=r"^\S+:\d+:\d+:", SCOPE_PATHS="src",
            FROZEN_PATHS="SPEC.md tests/acceptance", LOC_EXTENSIONS="py",
            TOOLS="Bash(python *) Bash(pytest *) Bash(ruff *) Bash(uv *)", EXTRA=[]),
        "go.mod": dict(
            BUILD_CMD="go build ./...", TEST_CMD="go test ./... 2>&1",
            LINT_CMD="go vet ./... 2>&1 || true", LINT_COUNT_PATTERN=r"\.go:\d+",
            SCOPE_PATHS=".", FROZEN_PATHS="SPEC.md acceptance", LOC_EXTENSIONS="go",
            TOOLS="Bash(go *)", EXTRA=[]),
    }
    p = next((v for k, v in presets.items() if (repo / k).exists()), None) or dict(
        BUILD_CMD="", TEST_CMD="", LINT_CMD="", LINT_COUNT_PATTERN="^(warning|error)",
        SCOPE_PATHS="src", FROZEN_PATHS="SPEC.md tests", LOC_EXTENSIONS="", TOOLS="Bash", EXTRA=[])
    base_tools = "Read Edit Write Glob Grep Bash(ls *) Bash(cat *) Bash(git diff *) Bash(git status *)"
    extra = "\n".join(f"  {e}" for e in p["EXTRA"])
    print(f"""# tourney run config (bash, sourced by scripts/tourney). Edit freely.
MODE={a.mode}                       # greenfield | reboot

# --- commands, run inside each candidate worktree --------------------------
BUILD_CMD="{p['BUILD_CMD']}"
TEST_CMD="{p['TEST_CMD']}"         # must exercise the frozen acceptance tests
LINT_CMD="{p['LINT_CMD']}"
LINT_COUNT_PATTERN='{p['LINT_COUNT_PATTERN']}'   # regex counted in lint output
TEST_PASSED_REGEX='([0-9]+) passed'           # summed over test output
TEST_FAILED_REGEX='([0-9]+) failed'

# --- what candidates own --------------------------------------------------
SCOPE_PATHS="{p['SCOPE_PATHS']}"       # space-separated; candidates may rewrite these freely
FROZEN_PATHS="{p['FROZEN_PATHS']}"     # acceptance contract; hashed at freeze, edits are flagged
LOC_EXTENSIONS="{p['LOC_EXTENSIONS']}"   # file extensions counted for LOC (empty = all)

# Extra numeric metrics: "name|shell command printing a number" (run in each worktree)
EXTRA_METRICS=(
{extra}
)

# --- agents ---------------------------------------------------------------
MODEL=""            # e.g. opus, sonnet; empty = CLI default
EFFORT=""           # low|medium|high|xhigh|max; empty = default
BUDGET_USD="20"     # per agent run (--max-budget-usd); empty = unlimited
PARALLEL=4          # concurrent agents
MEASURE_PARALLEL=1  # concurrent build/test runs (builds are heavy)
PERMISSION_MODE="acceptEdits"
# Implementers/probers: edit files + run the project's toolchain. Judges are read-only.
ALLOWED_TOOLS="{base_tools} {p['TOOLS']}"
JUDGE_TOOLS="Read Glob Grep Bash(ls *) Bash(wc *) Bash(git diff *)"

# Worktrees live outside the repo (default: ../<repo>.tourney/<run>/)
WORKTREE_ROOT=""

# --- differential test (optional) -----------------------------------------
# A harness file kept in this run dir, written against the spec's public interface.
# It is copied (untracked) into each worktree, run, and removed. It must print one JSON
# object per line: {{"case": "id", "ok": true, "values": [numbers...]}} or
# {{"case": "id", "ok": false, "error": "Kind"}}; optional "tol" overrides DIFFTEST_REL_TOL.
DIFFTEST_HARNESS=""        # e.g. difftest/harness.rs (relative to this run dir)
DIFFTEST_INSTALL=""        # where it goes in a worktree, e.g. tests/zz_difftest.rs
DIFFTEST_CMD=""            # e.g. cargo test --release --quiet --test zz_difftest -- --nocapture
DIFFTEST_REFERENCE_ENV=""  # e.g. DIFFTEST_REFERENCE=1: also run the harness's own oracle as 'reference'
DIFFTEST_REL_TOL="1e-6"
DIFFTEST_ABS_TOL="1e-9"
""")


def cmd_freeze(a):
    cwd = Path(a.dir)
    files = tracked_files(cwd, a.paths.split())
    print(json.dumps({f: sha256(cwd / f) for f in files}, indent=1))


def cmd_count_json(a):
    print(len(load_json(a.file, {}) or {}))


def frozen_violations(cwd: Path, frozen: dict) -> list[str]:
    bad = []
    for f, h in frozen.items():
        p = cwd / f
        if not p.exists():
            bad.append(f"deleted:{f}")
        elif sha256(p) != h:
            bad.append(f"modified:{f}")
    return bad


def in_paths(f: str, paths: list[str]) -> bool:
    return any(p in (".", "") or f == p or f.startswith(p.rstrip("/") + "/") for p in paths)


def sum_regex(pattern: str, text: str) -> int | None:
    if not pattern:
        return None
    nums = [int(m.group(1)) for m in re.finditer(pattern, text)]
    return sum(nums) if nums else None


def read(p: str | Path) -> str:
    try:
        return Path(p).read_text(errors="replace")
    except OSError:
        return ""


def cmd_measure(a):
    cwd = Path(a.dir)
    scope, frozen_paths = a.scope.split(), a.frozen_paths.split()
    exts = {"." + e.lstrip(".") for e in a.ext.split()}
    loc = files = 0
    for f in tracked_files(cwd, scope):
        if exts and Path(f).suffix not in exts:
            continue
        if in_paths(f, frozen_paths):
            continue
        files += 1
        loc += sum(1 for line in read(cwd / f).splitlines() if line.strip())
    changed = [f for f in git(cwd, "diff", "--name-only", a.spec_ref, "HEAD").split("\n") if f]
    out_of_scope = [f for f in changed if not in_paths(f, scope) and not in_paths(f, frozen_paths)
                    and f not in ("DESIGN_NOTES.md", "TOURNAMENT_DESIGN.md")]
    test_log = read(a.logs + ".test.log")
    lint_log = read(a.logs + ".lint.log")
    extra = {}
    for line in read(a.logs + ".extra.tsv").splitlines():
        if "\t" in line:
            k, v = line.split("\t", 1)
            try:
                extra[k] = float(v) if "." in v else int(v)
            except ValueError:
                extra[k] = v or None
    print(json.dumps({
        "target": a.target,
        "build_ok": a.build_rc == 0,
        "tests_ok": a.test_rc == 0,
        "tests_passed": sum_regex(a.passed_re, test_log),
        "tests_failed": sum_regex(a.failed_re, test_log),
        "test_seconds": a.test_seconds,
        "lint_count": len(re.findall(a.lint_pattern, lint_log, re.M)) if a.lint_pattern else None,
        "loc": loc, "files": files,
        "frozen_violations": frozen_violations(cwd, load_json(a.frozen, {}) or {}),
        "out_of_scope_changes": out_of_scope,
        "extra": extra,
        "agent": agent_stats(a.agent_log),
    }, indent=1))


def cmd_brief_metrics(a):
    m = load_json(a.file, {})
    print(f"build={fmt(m.get('build_ok'))} tests={fmt(m.get('tests_ok'))} "
          f"pass/fail={fmt(m.get('tests_passed'))}/{fmt(m.get('tests_failed'))} "
          f"loc={m.get('loc')} lint={fmt(m.get('lint_count'))} "
          f"frozen_violations={len(m.get('frozen_violations', []))}")


def cmd_metrics_summary(a):
    rd = Path(a.rd)
    mapping = read_mapping(rd)
    rows, extra_keys = [], []
    metrics = []
    for f in sorted((rd / "metrics").glob("*.json")):
        m = load_json(f)
        if m:
            metrics.append(m)
            for k in m.get("extra", {}):
                if k not in extra_keys:
                    extra_keys.append(k)
    if a.blind:
        metrics = [m for m in metrics if m["target"] in mapping]
    for m in metrics:
        t = m["target"]
        label = t if a.blind or t not in mapping else f"{t} ({mapping[t]})"
        ag = m.get("agent", {})
        rows.append([label, m["build_ok"], m["tests_ok"], m.get("tests_passed"), m.get("tests_failed"),
                     m["loc"], m["files"], m.get("lint_count"),
                     len(m["frozen_violations"]) or "0", len(m["out_of_scope_changes"]),
                     *[m["extra"].get(k) for k in extra_keys],
                     ag.get("cost_usd"), ag.get("turns")])
    headers = ["target", "build", "tests", "passed", "failed", "LOC", "files", "lint",
               "frozen⚠", "out-of-scope", *extra_keys, "cost $", "turns"]
    print("# Metrics\n")
    print(md_table(headers, rows))
    flagged = [m for m in metrics if m["frozen_violations"]]
    if flagged:
        print("\n**Frozen-contract violations (treat as disqualifying unless justified):**")
        for m in flagged:
            print(f"- {m['target']}: " + ", ".join(m["frozen_violations"]))
    oos = [m for m in metrics if m["out_of_scope_changes"]]
    if oos:
        print("\n**Out-of-scope changes:**")
        for m in oos:
            print(f"- {m['target']}: " + ", ".join(m["out_of_scope_changes"][:15]))
    print("\nLOC excludes frozen test paths. Lower LOC is not automatically better; "
          "read it together with the probe results.")


def cmd_probe_record(a):
    cwd = Path(a.dir)
    stat = git(cwd, "diff", "--numstat", a.base, "HEAD")
    files = ins = dele = 0
    touched = []
    for line in stat.splitlines():
        parts = line.split("\t")
        if len(parts) == 3:
            files += 1
            touched.append(parts[2])
            ins += int(parts[0]) if parts[0].isdigit() else 0
            dele += int(parts[1]) if parts[1].isdigit() else 0
    log = load_json(a.agent_log, {}) or {}
    result = log.get("result", "") if isinstance(log, dict) else ""
    Path(a.notes_out).write_text(f"# Maintainer notes: {a.target} / {a.probe}\n\n{result}\n")
    print(json.dumps({
        "target": a.target, "probe": a.probe,
        "files_changed": files, "insertions": ins, "deletions": dele, "churn": ins + dele,
        "touched": touched,
        "tests_ok": a.test_rc == 0,
        # only edits made by the probe itself (the candidate's own tampering is reported by measure)
        "frozen_violations": [f"modified:{f}" for f in touched if in_paths(f, a.frozen_paths.split())],
        "agent": agent_stats(a.agent_log),
    }, indent=1))


def cmd_probe_summary(a):
    rd = Path(a.rd)
    mapping = read_mapping(rd)
    recs = [r for f in sorted((rd / "probe").glob("*__*.json")) if (r := load_json(f))]
    if a.blind:
        recs = [r for r in recs if r["target"] in mapping]
    rows = []
    for r in recs:
        t = r["target"]
        label = t if a.blind or t not in mapping else f"{t} ({mapping[t]})"
        rows.append([label, r["probe"], r["tests_ok"], r["files_changed"], r["insertions"],
                     r["deletions"], len(r["frozen_violations"]) or "0",
                     r["agent"].get("cost_usd"), r["agent"].get("turns")])
    print("# Change probes (hidden follow-up features)\n")
    print(md_table(["target", "probe", "tests after", "files", "+", "−", "frozen⚠", "cost $", "turns"], rows))
    # aggregate per target: lower churn/cost with tests passing = easier to change
    agg: dict[str, list] = {}
    for r in recs:
        agg.setdefault(r["target"], []).append(r)
    arows = []
    for t, rs in sorted(agg.items()):
        label = t if a.blind or t not in mapping else f"{t} ({mapping[t]})"
        ok = sum(1 for r in rs if r["tests_ok"])
        costs = [r["agent"].get("cost_usd") or 0 for r in rs]
        turns = [r["agent"].get("turns") or 0 for r in rs]
        arows.append([label, f"{ok}/{len(rs)}", sum(r["churn"] for r in rs) / len(rs),
                      sum(r["files_changed"] for r in rs) / len(rs),
                      sum(costs) / len(rs), sum(turns) / len(rs)])
    print("\n## Per target (means)\n")
    print(md_table(["target", "probes green", "churn", "files", "cost $", "turns"], arows))
    print("\nChurn = lines added + removed to implement the probe. Read the diffs and maintainer "
          "notes too: a small diff that bolts on a hack is worse than a clean, slightly larger one.")


def _load_jsonl(path: Path) -> dict[str, dict]:
    out = {}
    for line in read(path).splitlines():
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if isinstance(d, dict) and "case" in d:
            out[str(d["case"])] = d
    return out


def _status(d: dict | None) -> str:
    if d is None:
        return "missing"
    if d.get("ok", True):
        return "ok"
    return "error:" + str(d.get("error", "?")).split("(")[0].split("{")[0].strip()[:40]


def cmd_difftest_summary(a):
    """Compare per-case outputs across targets.

    Consensus per case: the 'reference' target if present, else the majority status and
    (for ok cases) the element-wise median of values over targets with that status.
    """
    import statistics
    rd = Path(a.rd)
    mapping = read_mapping(rd)
    runs = {f.stem: _load_jsonl(f) for f in sorted((rd / "difftest").glob("*.jsonl"))}
    if a.blind:
        runs = {t: r for t, r in runs.items() if t in mapping or t in ("spec", "reference")}
    if not runs:
        print("# Differential test\n\n(no results)")
        return
    order = [t for t in ("reference", "spec") if t in runs] + sorted(t for t in runs if t not in ("reference", "spec"))
    cases = sorted({c for r in runs.values() for c in r})
    stats = {t: dict(agree=0, value=0, status=0, missing=0, maxrel=0.0) for t in order}
    worst: list[tuple[float, str, str, list[str]]] = []

    def label(t):
        if t in ("spec", "reference"):
            return {"spec": "spec (original code)", "reference": "reference oracle"}[t]
        return t if a.blind or t not in mapping else f"{t} ({mapping[t]})"

    for c in cases:
        recs = {t: runs[t].get(c) for t in order}
        if "reference" in recs and recs["reference"] is not None:
            cons_status, cons_vals = _status(recs["reference"]), recs["reference"].get("values")
        else:
            sts = [_status(r) for r in recs.values() if r is not None]
            cons_status = max(set(sts), key=sts.count)
            vals = [r["values"] for r in recs.values()
                    if r is not None and _status(r) == "ok" and isinstance(r.get("values"), list)]
            cons_vals = None
            if cons_status == "ok" and vals:
                n = max(set(map(len, vals)), key=[len(v) for v in vals].count)
                vals = [v for v in vals if len(v) == n]
                cons_vals = [statistics.median(col) for col in zip(*vals)]
        dissent = []
        for t, r in recs.items():
            if t == "reference":
                continue
            st = _status(r)
            if st == "missing":
                stats[t]["missing"] += 1; dissent.append(f"{t}:missing"); continue
            if st != cons_status:
                stats[t]["status"] += 1; dissent.append(f"{t}:{st}"); continue
            if st == "ok" and cons_vals is not None:
                v = r.get("values") or []
                rel_tol = float(r.get("tol", a.rel))
                if len(v) != len(cons_vals):
                    stats[t]["value"] += 1; dissent.append(f"{t}:len{len(v)}"); continue
                worst_rel = 0.0
                bad = False
                for x, y in zip(v, cons_vals):
                    try:
                        x, y = float(x), float(y)
                    except (TypeError, ValueError):
                        bad = True; worst_rel = float("inf"); break
                    err = abs(x - y)
                    if err != err:  # NaN
                        bad = True; worst_rel = float("inf"); break
                    rel = err / max(abs(y), 1e-300)
                    if err > a.abs + rel_tol * abs(y):
                        bad = True
                        worst_rel = max(worst_rel, rel)
                if bad:
                    stats[t]["value"] += 1
                    stats[t]["maxrel"] = max(stats[t]["maxrel"], worst_rel)
                    dissent.append(f"{t}:rel{worst_rel:.1e}")
                    continue
            stats[t]["agree"] += 1
        if dissent:
            worst.append((len(dissent), c, cons_status, dissent))

    print("# Differential test\n")
    basis = "the reference oracle" if "reference" in runs else "the majority (median of values)"
    print(f"{len(cases)} cases. Consensus = {basis}. Tolerance: abs {a.abs:g} + rel {a.rel:g}·|consensus| "
          "(cases may set their own rel tol).\n")
    rows = [[label(t), stats[t]["agree"], stats[t]["value"], stats[t]["status"], stats[t]["missing"],
             f"{stats[t]['maxrel']:.1e}" if stats[t]["maxrel"] else "–"] for t in order if t != "reference"]
    print(md_table(["target", "agree", "value≠", "status≠", "missing", "max rel err"], rows))
    if worst:
        worst.sort(key=lambda w: (-w[0], w[1]))
        print("\n## Disagreements (most dissent first, up to 30)\n")
        for _, c, cs, d in worst[:30]:
            print(f"- `{c}` consensus `{cs}`: " + ", ".join(d))
    print("\nA target in the minority is not automatically wrong (the majority can share a bug, "
          "and the original code is one voter). Investigate each disagreement cluster.")


def cmd_extract_result(a):
    d = load_json(a.log, {}) or {}
    res = d.get("result") if isinstance(d, dict) else None
    if not res:
        res = f"(agent produced no result; subtype={d.get('subtype') if isinstance(d, dict) else '?'}; " \
              f"see {a.log} and the .err file)"
    print(res)


def cmd_status(a):
    rd, wt = Path(a.rd), Path(a.wt)
    mapping = read_mapping(rd)
    st = rd / "state"

    def mark(cond):
        return "✅" if cond else "⬜"

    print(f"run: {rd.name}   worktrees: {wt}")
    print(f"{mark((rd / 'logs/spec.json').exists())} spec drafted")
    print(f"{mark((st / 'spec_ref').exists())} spec frozen "
          f"({len(load_json(st / 'frozen.json', {}) or {})} frozen files)")
    done = [k for k in mapping if (rd / f"logs/implement-{k}.json").exists()]
    print(f"{mark(mapping and len(done) == len(mapping))} diverge: {len(done)}/{len(mapping)} candidates")
    for k, v in mapping.items():
        s = agent_stats(rd / f"logs/implement-{k}.json")
        print(f"     {k}  {v:<28} cost=${fmt(s['cost_usd'])} turns={fmt(s['turns'])}"
              f"{'  ERROR' if s['is_error'] else ''}")
    print(f"{mark((rd / 'metrics/summary.md').exists())} measured")
    print(f"{mark((rd / 'probe/summary.md').exists())} probed "
          f"({len(list((rd / 'probe').glob('*__*.json')))} probe runs)")
    print(f"{mark(any((rd / 'reports').glob('judge-*.md')))} judged "
          f"({', '.join(p.stem for p in (rd / 'reports').glob('judge-*.md'))})")
    print(f"{mark((rd / 'synthesis/DESIGN.md').exists())} synthesized")
    print(f"{mark((rd / 'logs/build.json').exists())} final built")
    total = sum((agent_stats(f)["cost_usd"] or 0) for f in (rd / "logs").glob("*.json"))
    print(f"\ntotal agent spend: ${total:.2f}")


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)

    p = sp.add_parser("render"); p.add_argument("template"); p.set_defaults(f=cmd_render)
    p = sp.add_parser("agent-stats"); p.add_argument("log"); p.add_argument("--oneline", action="store_true")
    p.set_defaults(f=cmd_agent_stats)
    p = sp.add_parser("detect-config"); p.add_argument("repo"); p.add_argument("mode")
    p.set_defaults(f=cmd_detect_config)
    p = sp.add_parser("freeze"); p.add_argument("dir"); p.add_argument("paths"); p.set_defaults(f=cmd_freeze)
    p = sp.add_parser("count-json"); p.add_argument("file"); p.set_defaults(f=cmd_count_json)

    p = sp.add_parser("measure")
    for n in ("dir", "target", "spec_ref", "frozen"):
        p.add_argument(n)
    for n in ("--build-rc", "--test-rc", "--lint-rc", "--test-seconds"):
        p.add_argument(n, type=int, default=0)
    for n in ("--scope", "--frozen-paths", "--ext", "--lint-pattern", "--passed-re", "--failed-re",
              "--logs", "--agent-log"):
        p.add_argument(n, default="")
    p.set_defaults(f=cmd_measure)

    p = sp.add_parser("brief-metrics"); p.add_argument("file"); p.set_defaults(f=cmd_brief_metrics)
    p = sp.add_parser("metrics-summary"); p.add_argument("rd"); p.add_argument("--blind", action="store_true")
    p.set_defaults(f=cmd_metrics_summary)

    p = sp.add_parser("probe-record")
    for n in ("dir", "target", "probe", "base", "frozen"):
        p.add_argument(n)
    p.add_argument("--test-rc", type=int, default=0)
    for n in ("--frozen-paths", "--agent-log", "--notes-out"):
        p.add_argument(n, default="")
    p.set_defaults(f=cmd_probe_record)

    p = sp.add_parser("probe-summary"); p.add_argument("rd"); p.add_argument("--blind", action="store_true")
    p.set_defaults(f=cmd_probe_summary)
    p = sp.add_parser("difftest-summary"); p.add_argument("rd"); p.add_argument("--blind", action="store_true")
    p.add_argument("--rel", type=float, default=1e-6); p.add_argument("--abs", type=float, default=1e-9)
    p.set_defaults(f=cmd_difftest_summary)
    p = sp.add_parser("extract-result"); p.add_argument("log"); p.set_defaults(f=cmd_extract_result)
    p = sp.add_parser("status"); p.add_argument("rd"); p.add_argument("wt"); p.set_defaults(f=cmd_status)

    a = ap.parse_args()
    a.f(a)


if __name__ == "__main__":
    main()
