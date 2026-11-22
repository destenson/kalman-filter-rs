# tourney run config (bash, sourced by scripts/tourney). Edit freely.
MODE=reboot                       # greenfield | reboot

# --- commands, run inside each candidate worktree --------------------------
BUILD_CMD="cargo build --all-targets --quiet"
TEST_CMD="cargo test --quiet --test spec_linear_kf && cargo test --quiet --lib"
LINT_CMD="cargo clippy --all-targets --quiet"
LINT_COUNT_PATTERN='^(warning|error)'   # regex counted in lint output
TEST_PASSED_REGEX='([0-9]+) passed'           # summed over test output
TEST_FAILED_REGEX='([0-9]+) failed'

# --- what candidates own --------------------------------------------------
SCOPE_PATHS="src/filter.rs src/matrix src/types.rs src/error.rs src/builder.rs src/unified.rs src/legacy.rs src/validation.rs"
FROZEN_PATHS="SPEC.md tests/spec_linear_kf.rs"
LOC_EXTENSIONS="rs"   # file extensions counted for LOC (empty = all)

# Extra numeric metrics: "name|shell command printing a number" (run in each worktree)
EXTRA_METRICS=(
  "unwraps|grep -rho '\.unwrap()' src/filter.rs src/matrix | wc -l"
  "vec_allocs|grep -rho 'vec!\[' src/filter.rs src/matrix | wc -l"
  "cfg_attrs|grep -rho '#\[cfg' src/filter.rs src/matrix src/unified.rs | wc -l"
)

# --- agents ---------------------------------------------------------------
MODEL=""            # e.g. opus, sonnet; empty = CLI default
EFFORT=""           # low|medium|high|xhigh|max; empty = default
BUDGET_USD="12"
PARALLEL=3
MEASURE_PARALLEL=1  # concurrent build/test runs (builds are heavy)
PERMISSION_MODE="acceptEdits"
# Implementers/probers: edit files + run the project's toolchain. Judges are read-only.
ALLOWED_TOOLS="Read Edit Write Glob Grep Bash(ls *) Bash(cat *) Bash(git diff *) Bash(git status *) Bash(cargo *)"
JUDGE_TOOLS="Read Glob Grep Bash(ls *) Bash(wc *) Bash(git diff *)"

# Worktrees live outside the repo (default: ../<repo>.tourney/<run>/)
WORKTREE_ROOT=""


# differential test (run-specific harness; see difftest/harness.rs)
DIFFTEST_HARNESS="difftest/harness.rs"
DIFFTEST_INSTALL="tests/zz_difftest.rs"
DIFFTEST_CMD="cargo test --quiet --test zz_difftest -- --nocapture"
DIFFTEST_REFERENCE_ENV="DIFFTEST_REFERENCE=1"
DIFFTEST_REL_TOL="1e-7"
DIFFTEST_ABS_TOL="1e-9"
