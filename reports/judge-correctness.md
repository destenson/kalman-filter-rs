## Ranking

| candidate | score (correctness & robustness) | one-line verdict |
|---|---|---|
| C | 8 | The only green tree and the only candidate whose singularity policy is explicit and documented, with the smallest worst-case error — but B21's general postcondition is still only spot-checked. |
| B | 5 | The most carefully decomposed numerics and the best error naming, but `cargo build --all-targets` is red so no test in its tree can protect the code, and it accepts a nearly singular `S` with a 0.75 relative error. |
| A | 4 | Correct equations and the strongest test in the pool — which it ships red: on valid random models an `Ok` update leaves an indefinite covariance. |

All three edited nothing in the frozen contract (SPEC.md and `tests/spec_linear_kf.rs` are byte-identical across A/B/C; `frozen⚠` is 0 for all) and all pass the 32 frozen tests. Everything below is outside what the frozen suite can see.

## Per candidate

### C

**Strengths**

- The singularity decision is made explicitly instead of delegated: `Cholesky::new_unchecked` plus a per-pivot comparison `L_kk² > 100ε·S_kk` (`C/src/filter.rs:232-242`). nalgebra's `Cholesky::new` only tests `pivot > 0`, so this is the one place in the pool where the check the spec asks for is actually written out; the pivot is compared to the *pre-factorization* diagonal captured at `C/src/filter.rs:229-231`, which makes the test invariant under rescaling a measurement component (B10, B21, B22).
- `S` is solved, never inverted: `Kᵀ = S⁻¹(HP)` by `Cholesky::solve_mut` (`C/src/filter.rs:245-246`), and `K` is never formed (`gemv_tr` at `:251`).
- Every slice length goes through `checked_mul` before any arithmetic (`C/src/filter.rs:288-293`), zero dimensions and `Const<N>`/`n` disagreement are rejected without touching nalgebra's assertions (`C/src/filter.rs:275-285`).
- One commit point: symmetrize, check every value finite, then swap (`C/src/filter.rs:262-271`). Nothing is committed before a successful step, so B9/B20 atomicity is structural.
- Two tests target the failure mode the others ignore: a rejected predict leaves scratch garbage that a following update must survive, and a singular update must be followed by a working one (`C/src/filter.rs:392-409`).
- A 1000-step f32 run on model B asserts `λ_min ≥ -1e-5 λ_max` every step (`C/src/filter.rs:375-390`) — the only in-tree regression test for B11 outside the frozen models.
- `C/DESIGN_NOTES.md` has the required architecture rationale, the nalgebra decision, a migration table (`:113-127`) and a candid "known weaknesses" section (`:77-90`).
- `full-test-status.md:15` records the whole tree green; that matters because a test target that does not compile protects nothing.

**Defects**

- **[major]** B21's second sentence ("every `update` that returns `Ok` leaves `P` finite and symmetric PSD within the B11 tolerances, on any valid model") is not checked anywhere: `commit` verifies only finiteness (`C/src/filter.rs:265-267`). Combined with `100ε` per-pivot tolerance (`C/src/filter.rs:64-66`), `S` is accepted up to a condition number of about `1/(100ε)` = 4.5e13 in f64 and 8e4 in f32; at that point `K` is wrong and the Joseph sum is only PSD if `K` is accurate. The frozen suite only looks at models A and B, so 32/32 is not evidence for B21 here. A's randomized test (see below) shows the same policy is insufficient on other valid models; C has no equivalent test that could catch it.
- **[minor]** The PSD validation scales its tolerance by `trace(A)` (`C/src/filter.rs:330-335`). When the diagonal sums past `f64::MAX`, `tol*trace` is `inf`, `identity*inf` puts `NaN` on the off-diagonals, and the Cholesky fails: `p0 = diag(1.7e308, 1.7e308)` — finite and PSD — is rejected with `InvalidCovariance`, contradicting B4. At the other end the shift underflows, so rank-deficient PSD matrices with denormal entries (e.g. `[[1e-315,1e-315],[1e-315,1e-315]]`, strictly positive diagonal) are rejected, contradicting B5/B18. C's own test stops at 1e30 (`C/src/filter.rs:414-415`).
- **[minor]** A non-finite `S` (overflow) is reported as `SingularMatrix` (`C/src/filter.rs:236-242`), so "S overflowed" and "S is singular" are indistinguishable to the caller. The notes admit this (`C/DESIGN_NOTES.md:81-82`); B20 only requires an error, so it is not a violation, but it loses information.
- **[minor]** The public compatibility shim silently does nothing when a length is wrong: `DefaultBackend::matvec`/`matmul`/`symmetrize` write the output only if `load` succeeds and discard the error (`C/src/matrix/mod.rs:51-55`, `:58-62`, `:79-83`). `matrix` is a public module (`C/src/lib.rs:37`), so this is a public API accepting an argument and then ignoring it (B14, in the dropped-API shim).
- **[minor]** `ws.s.take().unwrap_or_else(|| OMatrix::zeros…)` (`C/src/filter.rs:226`) hides an invariant violation behind an allocating fallback; it never triggers today, but the failure would be silent.

**Ideas worth grafting (keep in the rewrite)**

- The `new_unchecked` + `l_dirty()` + relative-pivot idiom (`C/src/filter.rs:232-242`): it is the only way in nalgebra to express the spec's singularity rule.
- `FilterAllocator` as one named trait for the five allocator bounds (`C/src/filter.rs:50-58`) — removes a class of copy-paste bound errors.
- The scratch-recovery tests (`C/src/filter.rs:392-409`).

### B

**Strengths**

- Atomicity is structural, not by convention: `predict`/`correct` read a validated `Model`/`Estimate` and write a separate `out` (`B/src/filter.rs:143-243`); the shell swaps only on `Ok` (`B/src/filter.rs:356-367`). The core is testable as pure values, and `B/src/filter.rs:371-451` does test it that way.
- The PSD test uses pivoted diagonal elimination and accepts singular PSD matrices with no diagonal shift at all (`B/src/matrix/mod.rs:111-140`), so it does not have C's trace-overflow failure; it handles `diag(1e-300)` and `diag(1.7e308)` alike.
- One relative-tolerance helper used consistently for update decisions (`B/src/matrix/mod.rs:157-160`), and no absolute magnitude threshold in the linear path.
- `KalmanError::InvalidInput(&'static str)` names the offending argument (`B/src/error.rs:28-29`) — more useful than C's variant-only `NonFiniteInput`, and it is what makes `Model::new`'s "first problem" reporting readable.
- `require_finite` is one function applied to the whole estimate (`B/src/filter.rs:245-247`), and the unit test at `B/src/filter.rs:405-413` asserts Joseph beats the simple form on `P = 1e8, R = 1e-8`.
- Probe evidence: B refuses to silently drop a control term — `predict()` on a filter with `k > 0` returns `DimensionMismatch`, and `kalman_to_information` refuses a controlled filter rather than dropping `B·u` (`_evidence/B__control-input.notes.md:3-10`). That is the B14 attitude applied forward.

**Defects**

- **[critical]** `cargo build --all-targets` is red and therefore `cargo test` compiles nothing: `B/tests/functional_tests.rs:7` still imports the removed `kalman_filters::filter::KalmanFilter`, while the same file was half-ported to the new constructor (`B/tests/functional_tests.rs:146-180`). `_evidence/metrics.md:6` records `build ❌`; `_evidence/full-test-status.md:14` records that no test target runs. This breaks the hard constraint in SPEC §2/Q6 and means B's 144 passing tests protect nothing unless invoked target by target. The probe notes call it "pre-existing at HEAD" (`_evidence/B__observer-hook.notes.md:8`, `_evidence/B__ergonomic-constructors.notes.md:8`), which is wrong: the baseline had `filter::KalmanFilter` (BRIEF.md:24-32), so B introduced the break by removing the type and stopping there.
- **[major]** No post-update PSD check anywhere: the only postcondition after the Joseph sum is `require_finite` (`B/src/filter.rs:242`), so B21 is unverified. The difftest shows what accepting a nearly singular `S` costs: on `illcond04` B returns `Ok` with a relative error of 7.5e-1 (`_evidence/difftest.md:15`), the worst number in the run. Under the spec this is not strictly a violation (only PSD, not accuracy, is required off models A/B), but it is the wrong behaviour for a filter whose stated stance is "report, don't repair".
- **[minor]** A ported property test now treats a spec-allowed outcome as failure: `prop_covariance_positive_semidefinite` returns `false` when `predict` errors (`B/tests/quickcheck_invariants.rs:120-122`), but `FilterDivergence` from overflow is a documented `predict` result (B7/B20). The probe notes already call this test flaky (`_evidence/B__control-input.notes.md:9`); it is B's own guard that is wrong, not the filter.
- **[minor]** `prop_kf_if_equivalence` unconditionally returns `true` (`B/tests/quickcheck_invariants.rs:208-224`) — a property that cannot fail, exactly the category the spec calls out in §8.
- **[minor]** No `DESIGN_NOTES.md` in `B/`, so Q7 and the migration note required by SPEC §2 (SPEC.md:17-18) are absent. The probe maintainers confirm the file does not exist (`_evidence/B__control-input.notes.md:1-3`).

**Ideas worth grafting**

- The pure `(model, prior, workspace) -> out` step functions (`B/src/filter.rs:143-243`): they make "a failed step cannot touch the estimate" a type-level consequence and let the numerics be tested without a shell.
- The pivoted diagonal-elimination PSD test (`B/src/matrix/mod.rs:111-140`): accepts singular PSD matrices with no diagonal shift, so it is immune to the trace overflow/underflow that bites C.
- `InvalidInput(&'static str)` naming the bad argument (`B/src/error.rs:28-29`).
- "Refuse rather than drop" for the control term (`_evidence/B__control-input.notes.md`), which is the right reading of B14 for any future front end.

### A

**Strengths**

- The construction checks are ordered correctly: zero/overflowing dimensions via `checked_mul` first, then exact lengths, then finiteness, then PSD (`A/src/filter.rs:120-137`). Overflow is rejected before any product is computed, which is what B19 asks for.
- The PSD test is the only genuinely unit-invariant one in the pool: it normalises to a correlation matrix (`a_ij/√(a_ii a_jj)`) and Cholesky-tests `C + tol·I` (`A/src/validation.rs:26-61`). It accepts `1e-300` and `1e300` alike, including rank-deficient matrices, and has no trace overflow (contrast C, above).
- The Cholesky layer distinguishes a non-finite pivot from a singular one (`A/src/matrix/mod.rs:122-129`), so `S = ∞` becomes `FilterDivergence` rather than `SingularMatrix` (`A/src/filter.rs:253-256`).
- Joseph form with `S` solved by triangular solves (`A/src/filter.rs:253-281`), results built in a preallocated workspace and adopted by swapping two boxed slices (`A/src/filter.rs:300-303`), so a rejected step cannot alter the filter and steady state allocates nothing.
- The kernels use `chunks_exact`/`zip` throughout, so a wrong internal length truncates instead of panicking (`A/src/matrix/mod.rs:1-10`, `:52-58`); the only public inverse validates its slice (`A/src/matrix/mod.rs:191-197`).
- The pool's only randomized robustness test: 200 models, per-component-scaled PSD inputs, 100 steps, exact symmetry plus a Jacobi eigenvalue check against the B11 tolerance (`A/tests/linear_kf_robustness.rs:69-130`). It is the best test artefact here.

**Defects**

- **[critical]** An `Ok` update can commit a covariance that violates B11/B21. A's own `tests/linear_kf_robustness.rs` is red at its tip: f32 `λ ∈ [-9.8e-3, 8.47]` at step 1 (n=5, m=4), f64 `λ ∈ [-1.19e-15, 1.25e-13]` at step 57 (`_evidence/finding-A-psd.md:13-17`, `_evidence/full-test-status.md:13`). The cause is that `variances_consistent` (`A/src/filter.rs:79-84`) is only a necessary condition — non-negative variances and `|p_ij| ≤ √(p_ii p_jj)` are satisfiable by an indefinite n ≥ 3 matrix — and it is used as the PSD postcondition for both steps (`A/src/filter.rs:213-217`, `:292-294`). The pivot test it is standing in for is `64·m·ε` relative to each pivot's own diagonal (`A/src/filter.rs:69-73`, `:253`), i.e. condition numbers up to ~2e13 are accepted at f64 and ~3e4 at f32 (m=4).
- **[major]** The failure is misattributed and unguarded: when the Joseph sum loses PSD-ness, `update` returns `SingularMatrix` (`A/src/filter.rs:289-294`) although `S` may be perfectly conditioned. That is a wrong error for the caller and explains the two difftest cases where A refuses an update the oracle completes (`illcond00`, `illcond04`, `_evidence/difftest.md:14-15`).
- **[major]** The delivered tree is red on a spec behaviour and the author's own test says so: the observer probe notes record "`linear_kf_robustness` random-model PSD failures are pre-existing at HEAD" (`_evidence/A__observer-hook.notes.md:6`), i.e. the test was already failing when A was delivered. A red behavioural test next to a "140 passed" metric is the single most misleading artefact in the run.
- **[major]** No `DESIGN_NOTES.md` in `A/`: Q7 and the migration note of SPEC §2 are missing. The probe maintainers note this too (`_evidence/A__ergonomic-constructors.notes.md:1-3`).
- **[minor]** `predict` can return `FilterDivergence` for a finite result on a valid model, using the same insufficient check (`A/src/filter.rs:213-217`). False rejections of this kind are invisible to A's own test, which accepts `SingularMatrix`/`FilterDivergence` with an unchanged filter as success.
- **[minor]** The `CholeskyError::NonFinite → FilterDivergence` arm (`A/src/filter.rs:255`) is untested; reaching it needs `S` to overflow while `x` and `y` are finite (e.g. `H = 1e200`), a case no test constructs.

**Ideas worth grafting**

- `is_psd`'s correlation rescaling (`A/src/validation.rs:26-61`): normalise by `√(a_ii a_jj)`, then Cholesky `C + tol·I` with pivot `> 0`. Unit-invariant per component, and free of the trace overflow/underflow that makes C reject valid extremes.
- `CholeskyError::{Singular, NonFinite}` (`A/src/matrix/mod.rs:122-129`) so the caller can tell "S is singular" from "S overflowed".
- `mirror_upper` (`A/src/matrix/mod.rs:92-99`) to make covariance exactly symmetric by copying one triangle rather than averaging.
- The randomized robustness test in full (`A/tests/linear_kf_robustness.rs:17-130`), including the Jacobi eigenvalue helper and the per-component scaling of generated PSD inputs.
- Pointer-swap commit (`A/src/filter.rs:300-303`), which avoids the copy B's shell performs.

## Best skeleton

Start from **C**. Under this lens the decisive property is how few places can hold a numerical invariant wrongly, and C has the fewest: one type, one validated constructor, one place where `S` is judged singular (`C/src/filter.rs:232-242`), one commit point (`C/src/filter.rs:262-271`), and nalgebra doing the products so there is no hand-written kernel file to audit. That is not a claim that nalgebra is preferable in general — C's `DESIGN_NOTES.md:19-22` argues it and the owner's dependency decision is a separate matter — it is a claim about the correctness surface: A and B each add 200-300 lines of hand-written kernels whose index arithmetic has to be read and re-read to trust, whereas C's remaining risk is concentrated in three lines of pivot test.

C's own weak spots must be fixed in the rewrite rather than inherited: add a real post-update PSD check (or a tighter pivot test plus a square-root/UD form), and replace the trace-based PSD tolerance. Both are cheap and both have ready-made grafts from A and B below.

## Grafts

- **Correlation-scaled PSD validation** — `A/src/validation.rs:26-61`. Replaces C's `trace` shift (`C/src/filter.rs:330-335`), which rejects finite PSD matrices once the trace overflows (`diag(1.7e308, 1.7e308)`) or the shift underflows (rank-deficient PSD at denormal scale).
- **`CholeskyError::{Singular, NonFinite}`** — `A/src/matrix/mod.rs:122-129` and its mapping at `A/src/filter.rs:253-256`. Replaces C's habit of reporting an overflowing `S` as `SingularMatrix` (`C/src/filter.rs:236-242`).
- **The randomized PSD/eigenvalue test** — `A/tests/linear_kf_robustness.rs:69-130`. This is the test that would have caught A, and it is the missing guard for C's B21 exposure. It needs one change on transplant: the f32 tolerance should be the spec's 1e-5, not the source's 1e-3 (`A/tests/linear_kf_robustness.rs:130`).
- **Pure step functions writing to a separate `out`** — `B/src/filter.rs:143-243`. C's workspace-plus-swap is already atomic, but the core/out split makes the step testable from values and keeps the "garbage in scratch after an error is harmless" argument local.
- **Pivoted diagonal-elimination PSD test** — `B/src/matrix/mod.rs:111-140`. Accepts singular PSD matrices with no diagonal shift at all, so it is the fallback if correlation scaling is judged too clever; it has neither of C's trace pathologies.
- **Argument-naming error** — `B/src/error.rs:28-29` (`InvalidInput(&'static str)`), so a zero dimension and a NaN in `q` are distinguishable at the call site. C's `NonFiniteInput`/`ZeroDimension` split is a coarser version of the same idea.
- **Refuse rather than drop** — `_evidence/B__control-input.notes.md:3-10`: on a filter built with a control matrix, `predict()` without `u` must error, not silently apply `u = 0`. This is the correct reading of B14 if a front end is ever added.
- **Exact symmetry by copying one triangle** — `A/src/matrix/mod.rs:92-99`, in place of C's `fill_upper_triangle_with_lower_triangle` only if a non-nalgebra layer is later introduced; C's current call is equivalent because `P` is exactly symmetric.

## Where the numbers mislead

- **`metrics.md` — "tests ✅ / passed 140 / failed 0" for A and "tests ✅ / passed 144 / failed 0" for B.** Both are true only of the subsets that were run (`--test spec_linear_kf --lib`, per `full-test-status.md:7-9`). A's tree has a red behavioural test and B's tree does not compile at all; `_evidence/full-test-status.md:13-15` is the honest version. The `build` column (B ❌) is the only cell in `metrics.md` that reflects the delivered state, and it is easy to read past next to a green `tests` cell.
- **`metrics.md` — `turns`: C = 5, A = 154, B = 247.** This cannot be a quality signal: C produced 747 LOC, a full green tree and the only migration table in five turns. Whatever the metric counts, it is not comparable across candidates and should not be used to discount C's result.
- **`metrics.md` — `LOC` (C 747 < A 965 < B 1061) and `unwraps` (A 8, B 16, C 14).** Every one of those unwraps is inside a `#[cfg(test)]` module (verified by grep in all three owned trees), so the column measures test style, not reachable panics. LOC likewise rewards deleting a hand-written kernel file (C delegates to nalgebra) rather than making the numeric path safer.
- **32/32 frozen tests for everyone.** The frozen suite exercises B11 on exactly the two models the spec names, and B21's general postcondition only on two crafted cases. A passes all 32 while violating B21 on random valid models (`_evidence/finding-A-psd.md`). A uniform 32/32 therefore says nothing about the property this lens cares most about.
- **`difftest.md` — `max rel err` (B 7.5e-1, A 6.7e-2, C 1.8e-2).** The column mixes three different things: numerical error, rejection where the oracle succeeds, and error-variant naming (`edge_nan_q`, `edge_zero_*`). A's 6.7e-2 and C's 1.8e-2 come from cases where the *original code* is also 1.8e-2/6.5e-2 off the consensus (`_evidence/difftest.md:14-16`), so the "reference oracle" is not obviously right there. B's 7.5e-1 is genuinely bad, but ranking on this single number overstates the gap on everything else.
- **`probes.md` — churn (`A` 713 mean, `C` 563) and green counts (4/4 each).** The churn is dominated by test code: A's ergonomic-constructors probe adds about 1000 lines, mostly tests, while C's control-input probe removes 32 lines. All four probes are green for all three candidates, so neither churn nor the 4/4 score separates them; the probe that does carry correctness signal is B's refusal to silently drop control (`_evidence/B__control-input.notes.md`), which no summary column records.
- **`finding-A-psd.md` itself.** The run's own measurement did not find A's defect — a candidate's test did, and only after the fact. Any synthesis that ranks on the run's metrics alone will place A above B and miss that A is the one candidate with a demonstrated wrong answer on valid input.
