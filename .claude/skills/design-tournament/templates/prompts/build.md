You are implementing the final, approved design from a design tournament.

# Inputs
- `TOURNAMENT_DESIGN.md` (repository root): the blueprint. Follow it.
- `SPEC.md` and the acceptance tests in `{{FROZEN_PATHS}}`: the contract.
- `{{BLIND_DIR}}`: the read-only candidate implementations (`A/`, `B/`, …) that the
  design cites as references, plus the judges' reports in `_evidence/`.

# Brief
{{BRIEF}}

# Rules
- Work in the order given under "Implementation order" in the design, running
  `{{TEST_CMD}}` at each checkpoint.
- Consult the cited candidate code for ideas, edge cases, and tricky algorithms. Do not
  copy whole files: write code that fits *this* design, with consistent naming and
  error handling throughout.
- You own `{{SCOPE_PATHS}}`. **Never modify `{{FROZEN_PATHS}}`.** Outside scope, make
  only the minimal mechanical edits.
- Build nothing beyond the spec and the design. If the design is wrong or impossible
  somewhere, deviate minimally and document it.
- Build: `{{BUILD_CMD}}`. Tests: `{{TEST_CMD}}`. Lint: `{{LINT_CMD}}`. All must be clean.
- Do not commit; the harness commits.

# DESIGN_NOTES.md (required, at the repository root)
1. Architecture in ≤ 10 lines.
2. **Deviations from TOURNAMENT_DESIGN.md** and the reason for each.
3. Known weaknesses and next steps.

Finish with: test status, LOC, and a list of deviations.
