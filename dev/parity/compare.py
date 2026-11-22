#!/usr/bin/env python3
"""Compare the two engines of the linear filter, case by case.

Usage: compare.py <default.jsonl> <nalgebra.jsonl>
                  [--rel 1e-9] [--abs 1e-12] [--allow-status case[,case...]]

Both inputs are the output of dev/parity/harness.rs, one JSON object per case:

    {"case": id, "ok": true,  "values": [numbers...], "tol": ..., "abs": ...}
    {"case": id, "ok": false, "error": "Kind"}

A case's own "tol"/"abs" override the defaults. Cases listed in --allow-status
may differ in outcome (error versus Ok): the contract allows either outcome on a
numerically ill-conditioned model, and the two engines sum in a different order,
so their relative pivot tests can land on opposite sides of the boundary.
Values are never allowed to differ.

Exit status 0 only when every case agrees, or differs only in outcome for an
allowed case.
"""
import json
import sys


def load(path):
    cases = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("{"):
                record = json.loads(line)
                cases[record["case"]] = record
    return cases


def outcome(record):
    if record is None:
        return "missing"
    return "ok" if record.get("ok") else record.get("error", "error")


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    left, right = load(argv[1]), load(argv[2])

    rel_default, abs_default, allowed = 1e-9, 1e-12, set()
    for flag in ("--rel", "--abs", "--allow-status"):
        if flag not in argv:
            continue
        value = argv[argv.index(flag) + 1]
        if flag == "--rel":
            rel_default = float(value)
        elif flag == "--abs":
            abs_default = float(value)
        else:
            allowed = {case for case in value.split(",") if case}

    cases = sorted(set(left) | set(right))
    failures, reviewed, worst = [], [], 0.0
    for case in cases:
        a, b = left.get(case), right.get(case)
        if outcome(a) != outcome(b):
            (reviewed if case in allowed else failures).append(
                f"{case}: default {outcome(a)}, nalgebra {outcome(b)}"
            )
            continue
        if outcome(a) != "ok":
            continue
        va, vb = a.get("values") or [], b.get("values") or []
        if len(va) != len(vb):
            failures.append(f"{case}: {len(va)} values vs {len(vb)}")
            continue
        rel = float(a.get("tol", rel_default))
        abs_tol = float(a.get("abs", abs_default))
        for x, y in zip(va, vb):
            error = abs(x - y)
            scale = max(abs(x), 1e-300)
            if error > abs_tol + rel * scale:
                failures.append(f"{case}: {x!r} vs {y!r} (relative {error / scale:.3e})")
                break
            worst = max(worst, error / scale)

    print(f"cases compared: {len(cases)}")
    if reviewed:
        print(f"reviewed boundary cases that differ in outcome: {len(reviewed)}")
        for line in reviewed:
            print(f"  {line}")
    print(f"value disagreements: {len(failures)}")
    for line in failures[:20]:
        print(f"  {line}")
    print(f"largest relative difference where both engines returned a value: {worst:.3e}")
    ok = not failures
    print("PARITY: " + ("ok" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
