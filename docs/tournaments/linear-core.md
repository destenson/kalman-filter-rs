# Tournament record: linear-core

Durable record of a design tournament. The run directory (`.tournament/`) is local;
this file is what survives. A later tournament can start from it with
`tourney init <next> --previous <this file>`, which carries every DEFER item forward.

| | |
|---|---|
| Baseline | `c93b68ca344f54de99d5f3f7e28234255443ca1b` |
| Frozen spec | `b883546d049299975962e0bd53517559603dc942` |
| Final build | `1f17a3fa129b5d426ba550c32b829213f89d15e2` |

## Candidates (stance per letter)

- `A`: data-oriented
- `B`: functional-core
- `C`: nalgebra-native
- `D`: ports-adapters
- `E`: simplest
- `F`: square-root
- `G`: type-driven

## Requirements ledger

Every requirement that comes up during the tournament gets exactly one decision.
That covers ideas from the brief, the spec agent, auditors, the user, judges, probes
and the synthesis.

| Decision | Meaning | Required column |
|---|---|---|
| `KEEP` | In this run's SPEC.md. | `Ref`: the behavior ID, e.g. `B12` |
| `DEFER` | Deliberately left for a later tournament. | `Ref`: a tracker issue (link or key) |
| `DROP` | Rejected. | `Why` |
| `?` | Not decided yet. `tourney freeze` refuses while any remain. | — |

Sources: `brief`, `user`, `spec`, `audit:<lens>`, `fold`, `judge:<lens>`, `probe:<name>`,
`synthesis`, `previous:<run>`.

Candidates never see this file. SPEC.md may list non-goals, but must never say that
something is planned for later, because candidates would design toward it.

| ID | Requirement | Source | Decision | Why | Ref |
|---|---|---|---|---|---|
| R1 | Covariance update is Joseph-form or equally stable; solve with S, never invert it | brief | KEEP | | B11, Q1 (DES-147) |
| R2 | P stays symmetric PSD on ill-conditioned models, and every successful update leaves P PSD | brief, audit | KEEP | | B11, B21 (DES-147, DES-145) |
| R3 | No heap allocation in steady-state predict/update | brief | KEEP | | B12 (DES-153) |
| R4 | No panic reachable from the public API (zero dims, overflowing dimension products, bad lengths) | brief, audit | KEEP | | B3, B15, B19 (DES-152) |
| R5 | Construction inputs validated: lengths, PSD p0/q/r, finite values | brief, spec | KEEP | | B2, B4, B5, B6 (DES-151, DES-150) |
| R6 | Results and validity independent of units/scale (uniform and per component) | spec, audit, difftest | KEEP | | B10, B18, B22 (DES-148, DES-149) |
| R7 | predict/update never commit a non-finite estimate; errors leave the filter unchanged | audit | KEEP | | B9, B20 (DES-146) |
| R8 | Logging/metrics never interleaved with the math | brief | KEEP | | Q4 |
| R9 | One validated construction path, plus at most one delegating front end | brief, user | KEEP | | B16, Q5 (DES-155) |
| R10 | No public mutable fields; no accepted-but-ignored inputs | brief, spec | KEEP | | B13, B14 (DES-155) |
| R11 | Time-varying F, Q, H, R per step, so EKF/UKF/CKF can be built on the core | user | DEFER | keep this spec small; next tournament | DES-143 |
| R12 | DimensionMismatch reports the exact supplied shape | audit:claims-vs-code | DROP | field values deliberately unspecified in the contract | |
| R13 | Control input (x = F·x + B·u) | brief non-goal; hidden probe | ? | proposed DEFER: the probe measures how well each design takes it; decide after results | |
| R14 | Fixed-interval (RTS) smoother | brief non-goal; hidden probe | ? | proposed DEFER: same as R13 | |
| R15 | Treat a NaN measurement component as "missing" (skip it) instead of rejecting the update | user review | ? | proposed DEFER: rejecting is the safe default (B6) | |
| R16 | Defined behavior for R with an exactly zero diagonal entry | fold | ? | proposed DROP: unspecified on purpose; difftest will show if candidates diverge | |
| R17 | ndarray/BLAS backends | brief non-goal | ? | proposed DEFER to the existing backend issues | DES-59, DES-142 |
| R18 | no_std support (do not foreclose it) | brief non-goal | ? | proposed DEFER: needs a tracker issue | |
| R19 | Ergonomic construction: helper constructors and/or builders with fewer parameters, all delegating to `new`'s validation | user (after freeze) | KEEP | added at synthesis/build as an owner amendment; relaxes B16/Q5; measured by the ergonomic-constructors probe | B16 amended (DES-155) |
| R20 | Fallible constructors are named `try_*` and return `KalmanResult<Self>` (`try_new`, `try_build`, ...); plain names only for infallible ones | user (after freeze) | KEEP | final build uses try_new; temporary `new` forward removed by a mechanical rename after verification | spec §2 interface amended (DES-155) |
| R21 | The core needs no matrix library: nalgebra stays an optional feature, never a required dependency | user (after candidates) | KEEP | rules out candidate C as the base; A chosen | Q6 amended; DES-143 |
| R22 | Accept very ill-conditioned updates (P0 huge, R tiny) with a usable PSD covariance instead of SingularMatrix: square-root/UD update | build agent (known weakness) | ? | proposed DEFER: final rejects 5 of 20 such generated cases; B21-correct but conservative | |

## Approved design

### Graft plan: linear Kalman filter core, built on A

I read the in-scope code of A, B and C and the evidence files, and diffed A against `_baseline`. I did not build or run anything: copying A to the scratchpad needed a permission I didn't have. Every claim below comes from reading the code, and the build order says where it must be checked.

#### 1. Verdict on A

A is a good base. It has a small kernel module with no dependencies (`A/src/matrix/mod.rs`) and an allocation-free workspace whose commit is an O(1) pointer swap (`A/src/filter.rs:300`). Its PSD check is the only one of the three that is truly unit-invariant: it scales the matrix to unit diagonal before testing (`A/src/validation.rs:22-60`). B's tolerance is relative to max|a| and C's to the trace, so neither is invariant per component. A's Cholesky pivot test is relative to each pivot's own diagonal, and it keeps P exactly symmetric by computing the upper triangle and mirroring it.

Its weaknesses:
- The error variants are inconsistent: a NaN in `p0`, `q` or `r` comes back as a covariance error instead of `NonFiniteInput`.
- `symmetrize` can overflow a finite `q` or `r` to ∞.
- Three different ad-hoc tolerance constants.
- The finiteness checks are duplicated in predict and update.
- The public surface is wider than needed: three import paths and four extra accessors.
- Two unexplained false `SingularMatrix` results in the difftest (`illcond00`, `illcond04`).
- Examples break under `--features nalgebra`.

#### 2. Grafts (all four checks passed; most valuable first)

| # | Graft | Source | Where in A | Why it is better | Net |
|---|---|---|---|---|---|
| 1 | Check finiteness of **all six** construction inputs, before any PSD test | `C/src/filter.rs:139-141` | `A/src/filter.rs:135-137` (checks only x0, F, H) | A sends NaN in `p0` to `InvalidCovariance` and NaN in `q`/`r` to `InvalidNoiseCovariance` (difftest `edge_nan_q`). One cause should give one variant. | 0 |
| 2 | `check_len(data, rows, cols)` does the `checked_mul` itself, called in a loop over the six inputs. After it passes, `nn`/`mn`/`mm` are just `p0.len()`, `h.len()`, `r.len()` | `C/src/filter.rs:136-138, 288-293` (same idea: `B/src/validation.rs:23-36`) | `A/src/filter.rs:90-97, 121-134` | Removes the separate `let (Some(nn),…) else` block and the redundant `len` parameter. An overflowing product can never equal a real slice length, so B19 gives `DimensionMismatch` naturally. | −8 |
| 3 | Symmetrize stored matrices by **mirroring** instead of averaging, and apply it to `p0` as well (use the existing `mirror_upper`) | `C/src/filter.rs:325-326` | `A/src/filter.rs:147-152` | **Bug fix.** `(aij+aji)*0.5` (`A/src/matrix/mod.rs:115`) overflows for entries above about 9e307 (f64) or 1.7e38 (f32). A validated finite Q becomes ∞, and every later predict fails. Mirroring does no arithmetic and returns exactly symmetric input bit-for-bit, so B1 still holds. It also makes P exactly symmetric from construction on, not just after the first step as A's doc claims. | −2 |
| 4 | One `commit` that runs the finiteness check (plus A's `variances_consistent`) and then swaps | `C/src/filter.rs:262-271` | `A/src/filter.rs:207-218, 283-296, 300-303` | Four copies of the check and their messages become one. The early innovation check at `:242-244` can go too, because a non-finite `y` always makes `x⁺` non-finite (`k·∞` is ∞ or NaN). Signature: `fn commit(&mut self, on_inconsistent: impl FnOnce() -> KalmanError) -> KalmanResult<()>`. **It must take a closure:** building a `FilterDivergence(String)` eagerly would allocate on every step and break B12. | −10 |
| 5 | Replace `ZeroDimension` + overflow handling with a unit `ZeroDimension` variant (overflow → `DimensionMismatch`, see graft 2) | `C/src/error.rs` (`ZeroDimension`) | `A/src/error.rs:18-24, 50-53`; `A/src/filter.rs:121-128` | Leaves one meaning per variant. The payload duplicated arguments the caller already has. | −5 |
| 6 | Model accessors become `pub(crate)` | idea from `B/src/filter.rs:351` (`pub(crate) fn model`) | `A/src/filter.rs:173-191` | `transition`, `process_noise`, `observation` and `measurement_noise` are public API beyond SPEC. Their only caller is in-crate (`information/conversion.rs:30-33`). | 0 |
| 7 | `half = T::one() / (T::one() + T::one())` | `B/src/matrix/mod.rs:56` | `A/src/matrix/mod.rs:110`, `A/src/validation.rs:31` | Removes `unwrap_or_else(T::zero)`: if that conversion ever failed, it would silently zero the matrix. | 0 |
| 8 | Test that the filter recovers after a rejected step (stale scratch must not leak into the next step) | `C/src/filter.rs:392-400` | `A/src/filter.rs` tests | A's correctness depends on every workspace buffer being fully overwritten before it is read, and nothing tests that. A short test is cheap insurance. | +8 |

#### 3. Fixes to A that no candidate does better

| Issue | `A/path:line` | Fix | Net |
|---|---|---|---|
| Three ad-hoc tolerances (`64·m·ε`, `64·n·ε` slack, `16·n·ε`), plus a clumsy `<T as num_traits::Float>::epsilon()` because `KalmanScalar::epsilon` shadows it, and an `unwrap_or_else(T::max_value)` fallback that would accept *everything* | `filter.rs:71-73, 80`; `validation.rs:11-13` | One `pub(crate) fn rel_tol<T: Float>(k: usize) -> T` in `matrix/` (`64·k·ε`), used by all three. Re-run the f32 tests: the PSD tolerance gets 4× looser. | −6 |
| `variances_consistent` failing after an update returns `SingularMatrix`. This may cause the false errors on `illcond00` and `illcond04`, where the oracle says `ok`; for `illcond04`, C also returns `SingularMatrix` | `filter.rs:289-294` | **Find out which check fires first** (build step 3). If it is the pivot test, keep it: it is B21-correct and the cases sit on the boundary. If it is this check, keep the guard but document it. Don't tune constants to make the difftest pass. | 0 |
| `add_transpose_matmul_upper` has a dead `k` parameter (`let _ = k;`) | `matrix/mod.rs:82-83`, call at `filter.rs:280` | Remove the parameter. | −2 |
| `Workspace::new(n, m, nn, mn, mm)` passes values that can be derived | `filter.rs:63` | `Workspace::new(n, m)` (the lengths are valid after graft 2). | −1 |
| `symmetrize` (kept for the EKF) still has the overflow from graft 3 | `matrix/mod.rs:115` | `avg = aij*half + aji*half`. | 0 |
| Three public paths to one type (`kalman_filters::LinearKalmanFilter`, `::filter::…`, `::linear::…`) | `lib.rs:38, 63` | `mod filter;` (private). Keep `pub mod linear` (`filter.rs:27-29`) as the only path and drop the root re-export. Don't copy C's `pub use filter as linear` onto a private module: it hits E0365. | −1 |
| Stale crate docs ("EKF (future)", "UKF (future)"); the example uses `.unwrap()` | `lib.rs:13-14, 23-24` | Fix the bullets; write the example with `try_new(..)?`. | −1 |
| The constructor doc lists `NonFiniteInput (x0, F, H)` and `InvalidDimension` | `filter.rs:103-108` | Update to the new error set. | 0 |

#### 4. Removals from A
- The in-file `scalar_update_matches_closed_form` test (`A/src/filter.rs:345-351`): it duplicates the frozen test of the same name.
- The `symmetrize` calls in the constructor (`filter.rs:149-150`), replaced by `mirror_upper` (graft 3).
- The `InvalidDimension` variant and its Display arm (`error.rs:18-24, 50-53`). Also update `invalid_dimensions_do_not_panic` (`filter.rs:354-359`) to expect `ZeroDimension` / `DimensionMismatch`.
- The root re-export `pub use filter::LinearKalmanFilter` (`lib.rs:63`).
- The four `pub` model accessors stay but become `pub(crate)` (graft 6).
- Not removed, though tempting: `matvec`, `matmul_transpose`, `add_inplace`'s `size` parameter, `symmetrize` and `invert_matrix`. `extended.rs`, `information/*`, `scented.rs`, `unscented.rs` and `builders/information.rs` still use them. Put them under a `// Transitional: nonlinear filters only; delete when rebuilt` heading in `matrix/mod.rs`.

#### 5. Rejected ideas

| Idea | Source | Reason |
|---|---|---|
| `Model` / `Estimate` value types with pure step functions | `B/src/filter.rs:31-130` | Bloat: B's filter is about 90 lines longer for the same behaviour, and A's workspace swap already makes steps atomic. |
| `correct(prior, h, r, y, …)` as a seam for the EKF | `B/src/filter.rs:197` | Speculative. Add it when the EKF is actually rebuilt; it is not in SPEC. |
| Pivoted LDLᵀ PSD test, tolerance relative to max\|a\| | `B/src/matrix/mod.rs:111-140` | Worse: not invariant to per-component units (B18). A's unit-diagonal scaling is strictly better. |
| PSD test via a trace-shifted Cholesky | `C/src/filter.rs:313-336` | Worse, for the same reason (trace-relative), and it needs nalgebra. |
| `InvalidInput(&'static str)` | `B/src/error.rs` | Puts zero dimensions and NaN in one variant; difftest consensus disagrees (`edge_inf_f`, `edge_nan_measurement`). |
| Logging inside `commit` | `B/src/filter.rs:363` | Q4 says instrumentation wraps the math. Callers can log the `Err`. |
| `Const<N>` compile-time dimensions, nalgebra `gemm`/`quadform` | `C/src/filter.rs` throughout | nalgebra would become required, and the owner ruled that out. |
| A public `model()` returning a tuple of `OMatrix` | `C/src/filter.rs:191` | nalgebra types; graft 6 covers the need. |
| B's legacy `MatrixOps` trait shim | `B/src/matrix/legacy.rs` | Bloat; A's free functions serve the same callers. |
| C's single `100ε` constant | `C/src/filter.rs:64` | A's dimension-scaled tolerance is fine; my unification fix above covers it. |
| Retuning tolerances until `illcond00`, `illcond04` and `illcond18` agree with the oracle | difftest | These are boundary cases of B21; see the fix in §3. A's `illcond18` error (6.7e-2) is in the same range as C's (1.5e-2). |

#### 6. Amendment design

```rust
// kalman_filters::linear  (all items re-exported from `pub mod linear`)
impl<T: KalmanScalar> LinearKalmanFilter<T> {
    /// The one validated constructor.
    #[allow(clippy::too_many_arguments)]
    pub fn try_new(state_dim: usize, measurement_dim: usize,
                   x0: &[T], p0: &[T], f: &[T], q: &[T], h: &[T], r: &[T]) -> KalmanResult<Self>;

    /// Temporary, so the frozen tests compile. Remove afterwards.
    #[doc(hidden)]
    pub fn new(/* same args */) -> KalmanResult<Self> { Self::try_new(/* … */) }

    /// Infallible: it only records borrowed inputs. Everything is checked in `try_build`.
    pub fn builder<'a>(state_dim: usize, measurement_dim: usize, f: &'a [T], h: &'a [T])
        -> LinearKalmanFilterBuilder<'a, T>;

    /// Per axis the state is [pos, vel]: F = [[1,dt],[0,1]], H picks positions,
    /// Q = accel_var·[[dt⁴/4, dt³/2],[dt³/2, dt²]], R = meas_var·I, x0 = 0.
    pub fn try_constant_velocity(axes: usize, dt: T, accel_var: T, meas_var: T,
                                 p0: Covariance<'_, T>) -> KalmanResult<Self>;
}

#[derive(Clone, Copy, Debug)]
pub enum Covariance<'a, T> {
    Full(&'a [T]),      // k×k row-major, borrowed, never copied
    Diagonal(&'a [T]),  // length k
    Scalar(T),          // σ²·I
}

pub struct LinearKalmanFilterBuilder<'a, T> { /* n, m, f, h, x0: Option<&'a [T]>, p0/q/r: Option<Covariance<'a, T>> */ }
impl<'a, T: KalmanScalar> LinearKalmanFilterBuilder<'a, T> {
    pub fn x0(self, x0: &'a [T]) -> Self;              // default: zeros
    pub fn p0(self, p0: Covariance<'a, T>) -> Self;    // required
    pub fn q(self, q: Covariance<'a, T>) -> Self;      // required (no silent Q = 0)
    pub fn r(self, r: Covariance<'a, T>) -> Self;      // required
    pub fn try_build(self) -> KalmanResult<LinearKalmanFilter<T>>;
}
```

Rules that keep this from panicking and from duplicating validation:
- `try_build` first checks `n`, `m` and the lengths of `f` and `h`. Only then does it expand `Covariance` into a `Cow<[T]>`: `Full` stays borrowed; `Diagonal` and `Scalar` are allocated with `checked_mul` + `Vec::try_reserve_exact`, so a huge dimension returns an error instead of aborting the process. A wrong diagonal length gives `DimensionMismatch`. Then it calls `try_new`, the only place that validates.
- `try_constant_velocity` computes `n = axes.checked_mul(2)`, builds F/Q/H/R the same way, and goes through `try_build`. A non-finite `dt` gives `NonFiniteInput`; a negative variance gives `InvalidNoiseCovariance` or `InvalidCovariance`. Nothing is checked on its own.
- Size: about 30 lines for `Covariance`, 45 for the builder, 30 for the helper, plus one no-panic unit test for each.

**Error variants (final set):**

| Variant | Meaning (one cause → one variant) |
|---|---|
| `DimensionMismatch { expected, actual }` | Wrong slice or diagonal length, wrong `z` length, dimension product overflows |
| `ZeroDimension` | `n == 0` or `m == 0` (difftest `edge_zero_*_dim`) |
| `NonFiniteInput` | NaN/±∞ in any construction input or in `z` (`edge_nan_q`, `edge_inf_f`, `edge_nan_measurement`) |
| `InvalidCovariance` | `p0` not symmetric PSD |
| `InvalidNoiseCovariance` | `q` or `r` not symmetric PSD |
| `SingularMatrix` | `S` not numerically positive definite, relative to its own diagonal |
| `FilterDivergence(String)` | A step would produce a non-finite or inconsistent estimate |
| `BuilderIncomplete(String)` | `try_build` without `p0`, `q` or `r` (variant already exists; `builders/*` use it too) |
| `JacobianComputation(String)` | Out-of-scope modules only; stays while they use it |

Check order in `try_new`: dimensions, then lengths, then finiteness, then PSD. That order is what makes "one cause → one variant" true.

#### 7. Out-of-scope edits in A

Mostly minimal and mechanical: call sites move from `KalmanFilter::initialize` / `invert_matrix` to `matrix::invert_matrix` / `LinearKalmanFilter::new`; the `*_unified` examples are deleted along with the dropped API; benches are ported one-for-one; old tests are gated with `#[cfg(any())]`, which the spec allows. Six items are not purely mechanical:
1. **`--features nalgebra` is broken.** `examples/{simple_1d,position_2d,sensor_fusion}.rs` still have `#[cfg(feature = "nalgebra")] use kalman_filters::StaticKalmanFilter`, and that type no longer exists. Delete the nalgebra branches (A's `:39-44, :244+`, `:35-37, :352+`, `:37-39, :196+`). nalgebra must stay a working optional feature.
2. **The singularity threshold changed in the nonlinear filters.** `matrix::invert_matrix` now uses a relative `n·ε·max|A|` pivot test. The baseline used an absolute ε. This changes behaviour in the EKF/UKF/CKF/IF (`extended.rs:357`, `information/*`, `scented.rs:394`, `unscented.rs:508`). It is probably an improvement, but it is not mechanical; record it in DESIGN_NOTES.
3. `extended.rs:64` uses `use crate::matrix as DefaultBackend;`, a module aliased under a type-like name. Minimal churn, fine as a temporary shim, but list it under transitional.
4. `examples/sensor_fusion.rs:245-248` rebuilds the filter on every sensor switch (allocation plus revalidation per measurement) instead of mutating H and R. It's an acceptable workaround in an example; note it.
5. `information/conversion.rs`: `kf.predict()` became `kf.predict()?`, so errors now propagate, and `information_to_kalman` now rejects non-PSD input. Both changes are justified; record them.
6. Cargo features `naive-matrix`, `blas-backend` and `ndarray-linalg*` no longer gate anything (A's Cargo.toml:26-40). Leave them for now, as the brief allows, but list them as dead.

#### 8. Build order

| Step | Work | Checks |
|---|---|---|
| 1 | Errors and construction: grafts 1, 2, 3, 5, 7; fix the overflow in `symmetrize`; drop `InvalidDimension` | `cargo test --test spec_linear_kf` (32/32); `cargo test --lib`; `cargo build --all-targets`; `grep -r InvalidDimension src` is empty; a new unit test: f64 `q = [1e308]` survives one predict |
| 2 | Steps: graft 4 (closure-based `commit`), shared `rel_tol`, dead `k` parameter, `Workspace::new(n, m)`, graft 8 | `spec_change_no_heap_allocation_in_steady_state` (confirms the closure does not allocate); `tests/linear_kf_robustness.rs`; B11 tests in f32 **and** f64 |
| 3 | Triage `illcond00`/`illcond04`: temporarily log which check fires (pivot or `variances_consistent`) | Re-run the 270-case difftest. Expect 0 status mismatches on the `edge_*` cases with the new variant set, and no new mismatches anywhere else. Write the illcond conclusion into DESIGN_NOTES. |
| 4 | Public surface: `try_new` + `new` forward, private `mod filter`, `pub(crate)` accessors (graft 6), remove the root re-export, doc fixes | `cargo build --all-targets`; `cargo doc --no-deps`; `cargo clippy --all-targets`; doc-test passes |
| 5 | Amendments: `Covariance`, builder, `try_constant_velocity` | Unit tests: each helper equals `try_new` on the same dense input (bit-exact); missing field → `BuilderIncomplete`; `usize::MAX` axes / huge `m` with `Scalar` → `Err`, no abort; NaN `dt` → `NonFiniteInput`; a CV filter converges on a straight track |
| 6 | Out-of-scope fixes: nalgebra example branches; DESIGN_NOTES entries for §7 items 2–6 | `cargo build --all-targets` with default features **and** `--features nalgebra`; `cargo tree -e normal` shows nalgebra only under the feature |
| 7 | After the frozen tests switch to `try_new`: delete the `new` forward | Full `cargo test`; difftest; rerun `_evidence/metrics.md` (LOC should end slightly below 965 before the amendments, about +110 with them) |

Before step 3, the claims to confirm by running code are the overflow in `symmetrize` (step 1's unit test checks it) and which check causes the `illcond00`/`illcond04` errors.

## Metrics

### Metrics

| target | build | tests | passed | failed | LOC | files | lint | frozen⚠ | out-of-scope | unwraps | vec_allocs | cfg_attrs | cost $ | turns |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A (data-oriented) | ✅ | ✅ | 140 | 0 | 965 | 5 | 160 | 0 | 30 | 8 | 2 | 2 | 7.16 | 154 |
| B (functional-core) | ❌ | ✅ | 144 | 0 | 1061 | 6 | 149 | 0 | 29 | 16 | 3 | 2 | 8.00 | 247 |
| C (nalgebra-native) | ✅ | ✅ | 137 | 0 | 747 | 4 | 137 | 0 | 29 | 14 | 1 | 1 | 7.62 | 5 |
| final | ✅ | ✅ | 146 | 0 | 1247 | 6 | 170 | 0 | 30 | 11 | 2 | 2 | 5.63 | 176 |
| spec | ✅ | ❌ | 14 | 18 | 3744 | 12 | 141 | 0 | 0 | 27 | 206 | 99 | – | – |

**Out-of-scope changes:**
- A: benches/kalman_benchmarks.rs, examples/logging_demo.rs, examples/metrics_server.rs, examples/position_2d.rs, examples/position_2d_unified.rs, examples/sensor_fusion.rs, examples/sensor_fusion_unified.rs, examples/simple_1d.rs, examples/simple_1d_unified.rs, src/builders/information.rs, src/extended.rs, src/information/conversion.rs, src/information/distributed.rs, src/information/extended.rs, src/information/filter.rs
- B: benches/kalman_benchmarks.rs, examples/logging_demo.rs, examples/metrics_server.rs, examples/position_2d.rs, examples/position_2d_unified.rs, examples/sensor_fusion.rs, examples/sensor_fusion_unified.rs, examples/simple_1d.rs, examples/simple_1d_unified.rs, src/builders/information.rs, src/extended.rs, src/information/conversion.rs, src/information/distributed.rs, src/information/extended.rs, src/information/filter.rs
- C: Cargo.toml, benches/kalman_benchmarks.rs, examples/logging_demo.rs, examples/position_2d.rs, examples/position_2d_unified.rs, examples/sensor_fusion.rs, examples/sensor_fusion_unified.rs, examples/simple_1d.rs, examples/simple_1d_unified.rs, src/builders/information.rs, src/extended.rs, src/information/conversion.rs, src/information/distributed.rs, src/information/extended.rs, src/information/filter.rs
- final: benches/kalman_benchmarks.rs, examples/logging_demo.rs, examples/metrics_server.rs, examples/position_2d.rs, examples/position_2d_unified.rs, examples/sensor_fusion.rs, examples/sensor_fusion_unified.rs, examples/simple_1d.rs, examples/simple_1d_unified.rs, src/builders/information.rs, src/extended.rs, src/information/conversion.rs, src/information/distributed.rs, src/information/extended.rs, src/information/filter.rs

LOC excludes frozen test paths. Lower LOC is not automatically better; read it together with the probe results.

## Differential test

### Differential test

270 cases. Consensus = the reference oracle. Tolerance: abs 1e-09 + rel 1e-07·|consensus| (cases may set their own rel tol).

| target | agree | value≠ | status≠ | missing | max rel err |
|---|---|---|---|---|---|
| spec (original code) | 249 | 5 | 16 | 0 | 6.5e-02 |
| A (data-oriented) | 263 | 2 | 5 | 0 | 6.7e-02 |
| B (functional-core) | 262 | 2 | 6 | 0 | 7.5e-01 |
| C (nalgebra-native) | 266 | 3 | 1 | 0 | 1.8e-02 |
| final | 265 | 0 | 5 | 0 | – |

#### Disagreements (most dissent first, up to 30)

- `illcond00` consensus `ok`: spec:rel1.8e-02, A:error:SingularMatrix, B:error:SingularMatrix, C:rel1.8e-02, final:error:SingularMatrix
- `illcond04` consensus `ok`: spec:rel6.5e-02, A:error:SingularMatrix, B:rel7.5e-01, C:error:SingularMatrix, final:error:SingularMatrix
- `illcond18` consensus `ok`: spec:rel5.4e-04, A:rel6.7e-02, B:rel8.2e-04, C:rel1.5e-02, final:error:SingularMatrix
- `edge_nan_q` consensus `error:NonFiniteInput`: spec:ok, A:error:InvalidNoiseCovariance, B:error:InvalidInput
- `edge_zero_meas_dim` consensus `error:ZeroDimension`: spec:ok, A:error:InvalidDimension, B:error:InvalidInput
- `edge_zero_state_dim` consensus `error:ZeroDimension`: spec:ok, A:error:InvalidDimension, B:error:InvalidInput
- `illcond19` consensus `ok`: A:rel2.7e-04, C:rel7.1e-04, final:error:SingularMatrix
- `edge_inf_f` consensus `error:NonFiniteInput`: spec:ok, B:error:InvalidInput
- `edge_nan_measurement` consensus `error:NonFiniteInput`: spec:ok, B:error:InvalidInput
- `edge_bad_p0` consensus `error:InvalidCovariance`: spec:ok
- `illcond10` consensus `ok`: final:error:SingularMatrix
- `illcond15` consensus `ok`: spec:rel2.8e-04
- `illcond16` consensus `ok`: spec:rel6.6e-04
- `scale00_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale04_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale08_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale12_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale16_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale20_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale24_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale28_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale32_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale36_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance

A target in the minority is not automatically wrong (the majority can share a bug, and the original code is one voter). Investigate each disagreement cluster.

## Change probes

(not available)

