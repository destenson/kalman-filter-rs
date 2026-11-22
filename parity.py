#!/usr/bin/env python3
"""Compare two differential-test result files case by case.

Usage: parity.py <default.jsonl> <nalgebra.jsonl> [--rel 1e-9] [--abs 1e-12]
                 [--allow-status case[,case...]]

Cases listed in --allow-status may differ in outcome (error vs Ok). The spec allows
either outcome on a numerically ill-conditioned model, and the two engines sum in a
different order, so their relative pivot tests can land on opposite sides at the
boundary. Value disagreements are never allowed.

Both files are JSON lines produced by the run's difftest harness, one per case:
{"case": id, "ok": bool, "values": [..]} or {"case": id, "ok": false, "error": "Kind"}.
Per-case tolerances ("tol"/"abs") are honoured, defaulting to the values above.

Exit status 0 only when every case agrees in status and in values.
"""
import json
import sys


def load(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith("{"):
                continue
            rec = json.loads(line)
            out[rec["case"]] = rec
    return out


def status(rec):
    if rec is None:
        return "missing"
    return "ok" if rec.get("ok") else rec.get("error", "error")


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    left, right = load(argv[1]), load(argv[2])
    rel_default, abs_default = 1e-9, 1e-12
    for flag, target in (("--rel", "rel"), ("--abs", "abs")):
        if flag in argv:
            value = float(argv[argv.index(flag) + 1])
            if target == "rel":
                rel_default = value
            else:
                abs_default = value

    allowed = set()
    if "--allow-status" in argv:
        allowed = {c for c in argv[argv.index("--allow-status") + 1].split(",") if c}

    cases = sorted(set(left) | set(right))
    status_bad, value_bad, value_max = [], [], 0.0
    status_reviewed = []
    for case in cases:
        a, b = left.get(case), right.get(case)
        if status(a) != status(b):
            (status_reviewed if case in allowed else status_bad).append((case, status(a), status(b)))
            continue
        if status(a) != "ok":
            continue
        va, vb = a.get("values") or [], b.get("values") or []
        if len(va) != len(vb):
            value_bad.append((case, f"len {len(va)} vs {len(vb)}"))
            continue
        rel = float(a.get("tol", rel_default))
        abs_tol = float(a.get("abs", abs_default))
        for x, y in zip(va, vb):
            err = abs(x - y)
            scale = abs(x)
            if err > abs_tol + rel * scale:
                value_bad.append((case, f"{x!r} vs {y!r} (err {err:.3e}, rel {err / max(scale, 1e-300):.3e})"))
                break
            value_max = max(value_max, err / max(scale, 1e-300))

    print(f"cases compared: {len(cases)}")
    print(f"status disagreements: {len(status_bad)} (plus {len(status_reviewed)} reviewed boundary cases)")
    for case, sa, sb in status_reviewed:
        print(f"  reviewed {case}: default {sa}, nalgebra {sb}")
    for case, sa, sb in status_bad[:20]:
        print(f"  {case}: default {sa}, nalgebra {sb}")
    print(f"value disagreements: {len(value_bad)}")
    for case, detail in value_bad[:20]:
        print(f"  {case}: {detail}")
    print(f"max relative difference on agreeing cases: {value_max:.3e}")
    ok = not status_bad and not value_bad
    print("PARITY: " + ("ok" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
