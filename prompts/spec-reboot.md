You are writing the **contract** for a design-tournament *reboot* of existing code.
Several independent agents will later rewrite the code under `{{SCOPE_PATHS}}`, each
with a different architecture, and they will be compared against what you write. You
define *what* the code must do, never *how*.

# Brief
{{BRIEF}}

# Step 1: understand the current code
Read the code under `{{SCOPE_PATHS}}`, its callers elsewhere in the repo, its existing
tests, and its docs. Build a list of the public behaviors it actually has. Then sort
each one into:
- **KEEP**: correct and wanted.
- **CHANGE**: a defect, or a behavior the brief says must change (wrong math, panics,
  silently ignored inputs, misleading names, tests that test the wrong thing).
- **DROP**: dead, duplicated, or out-of-scope functionality the brief allows removing.

Verify suspected defects by reading carefully and, where cheap, by writing a quick
failing test. Do not guess.

# Step 2: deliverables (in this repository)
1. `SPEC.md` with these sections:
   - **Purpose**.
   - **Public interface**: what callers may rely on after the rewrite. State whether
     the existing API must be preserved exactly, may change with a migration note, or
     is free; follow the brief.
   - **Behaviors**: numbered (`B1`…), each tagged KEEP or CHANGE, with the defect
     explained for CHANGE items (cite file:line in the current code).
   - **Dropped**: what candidates may delete.
   - **Quality requirements** and **Non-goals**.
   - **Test index**: test name → behavior IDs.
2. **Acceptance tests** under `{{FROZEN_PATHS}}`, against the public interface only:
   - Characterization tests for KEEP behaviors. These must pass on the current code.
   - Tests for CHANGE behaviors, named with the prefix `spec_change_`. These are
     expected to fail on the current code; list them in SPEC.md.
   - If existing tests in these paths are wrong (assert buggy behavior, or test names
     that lie), fix or delete them and record that in SPEC.md under "Test curation".
   - Deterministic (fixed seeds), with float tolerances.

# Rules
- Do not fix or refactor the implementation code. Only the contract.
- Run `{{TEST_CMD}}`. KEEP tests pass; `spec_change_*` tests fail for the documented
  reason, not because of compile errors (if a CHANGE needs an API that does not exist
  yet, put the test behind the smallest stub needed to compile, and say so).
- Do not commit; the harness commits.

Finish with a summary: the counts of KEEP/CHANGE/DROP, the test counts, and the most
important defects found.
