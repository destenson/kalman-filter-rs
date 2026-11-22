You are one of several engineers independently implementing the same specification.
Each engineer was assigned a different design stance so that the team can explore the
design space. Afterwards, reviewers compare all the implementations on:

1. **Correctness**: the frozen acceptance tests, plus a careful read for edge cases.
2. **Simplicity**: how little a newcomer must understand to work in the code.
3. **Ease of change**: other engineers will later implement *unseen* follow-up
   features in your codebase. How much they have to touch, and how much the design
   fights them, is measured.

The best ideas from every implementation will be harvested, so a clear, honest
implementation of your stance is more useful than a hedge toward the average.

# Your design stance
{{STANCE}}

Commit to the stance. If, after a genuine attempt, part of it clearly cannot work for
this problem, deviate only there and explain why in DESIGN_NOTES.md. That finding is
valuable.

# Brief
{{BRIEF}}

# The contract
Read `SPEC.md` and the acceptance tests in `{{FROZEN_PATHS}}` first.

# Rules
- You own `{{SCOPE_PATHS}}`: restructure, rename, delete, or add files there freely.
- **Never modify anything under `{{FROZEN_PATHS}}`.** Edits there are detected
  automatically and disqualify the candidate. If you believe a test is wrong, say so
  in DESIGN_NOTES.md and leave it failing.
- Outside your scope, make only the minimal mechanical edits needed to keep the
  project building. Every such file is counted and reported.
- You may add your own tests in new files.
- Build only what the spec asks for. Speculative features and abstractions with one
  implementation count against you, unless your stance specifically calls for them.
- Iterate until these pass or you have exhausted reasonable approaches:
  - build: `{{BUILD_CMD}}`
  - tests: `{{TEST_CMD}}`
  - lint: `{{LINT_CMD}}` (fix warnings in your code)
- Do not commit; the harness commits your worktree.

# DESIGN_NOTES.md (required, at the repository root, ≤ 150 lines)
Reviewers read this blind, so **do not name or label your stance**. Describe the design itself.
1. **Architecture in ≤ 10 lines**: the main types/modules and how data flows.
2. **Key decisions**: each with the alternative you rejected and why.
3. **What fought back**: where the problem resisted this design.
4. **Known weaknesses**: be candid; reviewers will find them anyway.
5. **Next changes**: what you expect to be easy or hard to add later.
6. (reboot) **What was worth keeping** from the original code, and what was not.

Finish with a 5-line summary: test status, LOC, and the single most important design idea.
