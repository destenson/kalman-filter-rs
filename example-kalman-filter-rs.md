# Worked example: rebooting the linear core of `kalman-filter-rs`

`destenson/kalman-filter-rs` is a ~14k-line Rust crate that agents wrote with little
architectural guidance. It is a typical reboot candidate: lots of features, real
knowledge embedded in the tests, and structural problems that make every change
expensive. The defects below were checked against commit `c93b68c`. Re-check them
before reusing this brief, because the code may have moved.

## Observed problems (input to the brief)
- **Two parallel implementations of the same filter.** `KalmanFilter<T>` (flat
  `Vec<T>` matrices, `src/filter.rs:32`) and `StaticKalmanFilter<T, M, N>`
  (nalgebra, `src/filter.rs:460`) duplicate predict/update. On top of them,
  `src/unified.rs` (720 lines: an enum, `IntoUnified`, and `KalmanFilterTrait`),
  `src/legacy.rs`, `src/builder.rs`, and `src/builders/*` add more layers. That is
  four ways to construct and wrap one algorithm.
- **The covariance update is not the Joseph form**, though a test is named for it. Both
  backends use `P = (I − KH)P` and then symmetrize (`src/filter.rs:328-347`, `:533`).
  `test_joseph_form` (`src/filter.rs:1065`) only asserts that P shrinks, which the
  simple form also does, so a test that cannot fail hides the defect.
- **Explicit inverse of S** (`DefaultBackend::invert`, `src/filter.rs:297`) instead
  of a Cholesky solve, which is less stable and slower.
- **Allocation on every step.** About 10 `vec![…]` temporaries per predict+update
  (135 `vec!` occurrences in `filter.rs`), and one hand-rolled triple loop
  (`P·Hᵀ`, `:309`) that bypasses the backend.
- **Cross-cutting concerns tangled into the math.** `prometheus-metrics` and `log`
  trace blocks are interleaved with every numeric step of `predict`/`update`.
- **Inputs silently ignored.** The legacy API accepts `control_dim` and documents it as
  "unused in new implementation" (`src/legacy.rs:20`). Control input `B·u` is not
  supported at all.
- **Public mutable fields** (`kf.x`, `kf.P`, …) mean invariants such as symmetric PSD
  covariance and matching dimensions cannot be enforced.
- **`.unwrap()` in library code**: 19 in `filter.rs` and 191 across `src/` (some inside `#[cfg(test)]` modules).
- Other filters (`extended`, `unscented`, `information`, `scented`, …) depend on
  `filter.rs` and `matrix/`, so the scope boundary has to be explicit.

## Run setup

```bash
SKILL=.claude/skills/design-tournament/scripts/tourney   # or ~/.claude/skills/...
$SKILL init linear-core --mode reboot
```

### `config.sh` (changes from the detected defaults)
```bash
SCOPE_PATHS="src/filter.rs src/matrix src/types.rs src/error.rs src/builder.rs src/unified.rs src/legacy.rs"
FROZEN_PATHS="SPEC.md tests/spec_linear_kf.rs"
BUILD_CMD="cargo build --all-targets --quiet"            # the whole crate must still build
TEST_CMD="cargo test --quiet --test spec_linear_kf && cargo test --quiet --lib"
LINT_CMD="cargo clippy --all-targets --quiet"
EXTRA_METRICS=(
  "unwraps|grep -rho '\.unwrap()' src/filter.rs src/matrix | wc -l"
  "vec_allocs|grep -rho 'vec!\[' src/filter.rs src/matrix | wc -l"
  "cfg_attrs|grep -rho '#\[cfg' src/filter.rs src/matrix src/unified.rs | wc -l"
)
ALLOWED_TOOLS="Read Edit Write Glob Grep Bash(ls *) Bash(cat *) Bash(git diff *) Bash(git status *) Bash(cargo *)"
PARALLEL=3            # each worktree compiles the crate; ~1-2 GB target/ apiece
MEASURE_PARALLEL=1
```

### `brief.md`
```markdown
## Goal
Rebuild the *linear* Kalman filter core (state model, predict, update, matrix layer,
construction API) as one clean, numerically robust implementation that the nonlinear
filters (EKF/UKF/CKF/IF) can later be rebuilt on.

## Scope
Candidates own: src/filter.rs, src/matrix/, src/types.rs, src/error.rs, src/builder.rs,
src/unified.rs, src/legacy.rs. Other modules may receive only minimal mechanical edits
to keep compiling (they are rebuilt in later tournaments).

## Hard constraints
- Rust edition 2024, default features must build with no system libraries.
- No panics reachable from the public API on bad input: return KalmanError.
- f32 and f64 supported.
- nalgebra may be a required dependency, or the matrix layer may be own-code; the
  candidate decides and justifies the choice. ndarray/BLAS backends are out of scope.
- Public API may break; give a short migration note in DESIGN_NOTES.md.

## What is wrong today
(the list from "Observed problems" above, with file:line)

## Quality bar
- Joseph-form (or equivalently stable) covariance update; solve via Cholesky (no explicit S⁻¹).
- Covariance stays symmetric PSD over 10⁴ steps on an ill-conditioned model.
- Zero heap allocation in steady-state predict/update for fixed dimensions.
- Logging/metrics never interleaved with the math (hooks or an observer at the edge).
- One way to construct a filter.

## Non-goals
Nonlinear filters, particle/ensemble filters, OpenCV, prometheus integration redesign,
ndarray/BLAS backends, no_std (keep it possible, don't implement it).
```

### Stances
Keep `simplest`, `functional-core`, `type-driven`, `data-oriented`, and
`ports-adapters`. Delete `domain-model` and `minimal-deps`, and add two that are
specific to this domain:

`stances/nalgebra-native.md`
```markdown
**Lean on nalgebra entirely.** One filter type generic over nalgebra `Dim` (static and
dynamic dimensions through the same code via `OMatrix<T, N, N>`). Use nalgebra's
Cholesky and in-place ops. No custom matrix layer at all; delete src/matrix.
Likely failure mode: generic bounds sprawl (`DefaultAllocator: Allocator<…>`).
```

`stances/square-root.md`
```markdown
**Square-root / factored form.** Propagate a Cholesky factor of P (or UDUᵀ) instead
of P itself, so PSD holds by construction. Expose P only as a computed view.
Likely failure mode: complexity for small gains at low dimension; harder probes.
```

### Hidden probes (`probes/*.md`, never shown to candidates)
`probes/control-input.md`
```markdown
Add control input support: predict with x = F·x + B·u, where B is n×k and u is
supplied per call. Filters built without B must behave exactly as before. Add tests,
including one where a constant-acceleration control produces the analytically
expected position after 10 steps.
```
`probes/rts-smoother.md`
```markdown
Add a Rauch–Tung–Striebel fixed-interval smoother: given a filter configuration and
a sequence of measurements, return smoothed states and covariances for every step.
Smoothed covariance trace must be ≤ the filtered one at every step; on a 1-D
random-walk example, results must match the textbook recursion to 1e-9.
```
`probes/observer-hook.md`
```markdown
Users want to monitor the filter: after every update, report the innovation, its
covariance S, and the normalized innovation squared (NIS) to a user-supplied callback.
Zero cost when no callback is registered. Add a test that NIS averages ≈ m over 5,000
steps of a correctly specified model (within 10%).
```
These three stress different axes: a model extension (control), a new algorithm that
reuses internals (smoother), and a cross-cutting concern (observer, which is exactly
what the current metrics code gets wrong).

## Running it
```bash
$SKILL spec linear-core          # ~15-30 min; review SPEC.md + tests/spec_linear_kf.rs
$SKILL freeze linear-core
$SKILL all linear-core           # diverge → measure → probe → judge → synthesize (hours)
$SKILL status linear-core
# review .tournament/linear-core/synthesis/DESIGN.md, edit, then:
$SKILL build linear-core
$SKILL measure linear-core final && $SKILL probe linear-core final
```

Expected spec CHANGE items: the Joseph form (a test with an asymmetric-gain case where
the simple form loses PSD in f32), no panic on singular S, no silently dropped control
dimension, and dimension validation at construction.

## After this tournament
Freeze the winning linear core as the baseline, then run follow-up tournaments in
dependency order: `extended` + `scented` + `unscented` (scope: the nonlinear family,
sharing one `NonlinearSystem` abstraction), then `information`, then `particle` and
`ensemble`. Each later spec can reuse the previous run's DESIGN.md as a hard
constraint.
