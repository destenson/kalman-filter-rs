## Ranking

| candidate | score | one-line verdict |
|---|---|---|
| B | 8 | Every hidden change had a named place to land and needed no copied or re-derived arithmetic; the red build and missing notes are repairs, not structural costs. |
| C | 7 | The only green tree and the fewest files per change, but a new model input becomes a public type parameter and the smoother copies two of the filter's numeric policies. |
| A | 6 | Probes land in the filter's own idiom and the builder shares one dimension rule, but the PSD invariant is checked twice with different strength and the tree is red. |

## Per candidate

### B

**Strengths**
- Every probe found its layer. Control is a field on the validated `Model` plus one parameter on the core `predict` (`_evidence/B__control-input.diff:37-70,122-140`); the smoother is a pure `smooth_step` beside `predict`/`update`/`correct` with its own `SmoothWorkspace` (`_evidence/B__rts-smoother.diff:36-44,76-89`); the observer is a shell method called after the step (`_evidence/B__observer-hook.diff:155-171`); the builder only assembles slices for `new`. Production churn over the four probes is the lowest of the three: 710 lines vs 793 (C) and 852 (A), excluding tests and DESIGN_NOTES, counted from `_evidence/*.diff`.
- The workspace keeps `S` and its Cholesky factor in separate buffers (`B/src/filter.rs:110-111`; `B/src/filter.rs:216` reads `ws.s`, writes `ws.l`). That split is why the observer probe reports S by borrowing it (`_evidence/B__observer-hook.diff:165`) with only one m-element scratch for NIS (`_evidence/B__observer-hook.diff:160`). A and C both had to copy S.
- `correct(prior, h, r, y, ws, out)` (`B/src/filter.rs:197-243`) is exactly the linearized update the EKF rebuild needs, already a separate function. C's notes call the same extraction "a small refactor" (`C/DESIGN_NOTES.md:99-100`).
- The core is testable without a filter: `step_update` builds a workspace and calls `update` directly (`B/src/filter.rs:380-384`) — the cheapest place to pin a numeric change; no other candidate offers it.
- The filter is generic over the crate's `KalmanScalar` (`B/src/filter.rs:270`), so the nonlinear rebuild will not hit a trait mismatch. C's notes concede that problem for C (`C/DESIGN_NOTES.md:61-67`).
- `InvalidInput(&'static str)` names the offending argument (`B/src/error.rs:29-30`), so a new validated input needs no new enum variant.

**Defects**
- [critical] Build constraint breached: `B/tests/functional_tests.rs:7` still imports the removed `filter::KalmanFilter`, so `cargo build --all-targets` is red and plain `cargo test` runs nothing (`_evidence/full-test-status.md:14`, `_evidence/metrics.md:6`). A and C gated the same import (`A/tests/functional_tests.rs:7-8`, `C/tests/functional_tests.rs:7-8`); B's fix is the same mechanical gate.
- [major] No `DESIGN_NOTES.md` anywhere; `docs/DESIGN.md` is byte-identical to `_baseline/docs/DESIGN.md` and describes the removed `UnifiedKalmanFilter`/`KalmanFilterTrait`/`KalmanFilterBuilder` (`A/docs/DESIGN.md:23`), while `SPEC.md:296-297` requires the rationale and migration note.
- [minor] Documented invariant not enforced: `Model` is documented with symmetric Q and R (`B/src/filter.rs:29`) but `Model::new` stores the caller's slices verbatim (`B/src/filter.rs:50`) after a tolerance check (`B/src/validation.rs:49-56`). A within-tolerance asymmetric R enters S and the Joseph term; A and C canonicalise (`A/src/filter.rs:149-150`, `C/src/filter.rs:326`).
- [minor] The "dimensions fit in `usize`" rule is spelled three times: `B/src/validation.rs:29`, the builder pre-check (`_evidence/B__ergonomic-constructors.diff:140`) and `constant_velocity`. A has one function.
- [minor] The observer probe deleted `#[derive(Debug, Clone)]` and added only `Debug` (`_evidence/B__observer-hook.diff:73,85`), so the filter stopped being `Clone`; and the note claims S "had to be rebuilt or copied" (`_evidence/B__observer-hook.notes.md:6`) although the diff borrows it — the claim understates the design's strength and misleads a reader.
- [minor] The step contract "on `Err`, `out` holds garbage" (`B/src/filter.rs:138`) and `mem::take(&mut ws.y)` (`B/src/filter.rs:176`) are easy to break in a later edit.

**Ideas worth grafting**
- Keep `S` and its factor separate (`B/src/filter.rs:110-111`): that is the seam that made the observer free.
- Keep `correct` (`B/src/filter.rs:197-243`) as the one linearized update for the EKF.
- Keep a separate `SmoothWorkspace` for batch operations (`_evidence/B__rts-smoother.diff:36`) instead of widening the per-step workspace.
- Keep `InvalidInput(&'static str)` (`B/src/error.rs:29-30`).

### C

**Strengths**
- The only tree that builds and passes everything (`_evidence/full-test-status.md:15`), so a change can be verified by running the suite; all four probe test files ran in a green tree.
- Smallest footprint per change: control one file, observer one file, builder a new sibling module, smoother a new module plus two `pub(crate)` edits. Excluding design-notes edits, 1.75 files/probe vs 2.25 for A and B (`_evidence/*.diff`).
- The design notes predicted the hidden work — "control input (one more `gemv` in predict …)" and "exposing the innovation/`S` for NIS gating (both are already in the workspace)" (`C/DESIGN_NOTES.md:94-96`) — and the maintainers' diffs are those moves. Seams where change happened.
- One canonical parse-and-check function for all three covariance inputs, returning the checked matrix (`C/src/filter.rs:143-145,313-336`; symmetry restored at `:326`).
- One `model()` accessor (`C/src/filter.rs:191`) served the smoother; no other read path was needed.
- The observer change left the filter's `Clone`/`Debug`/`Send` alone (only the new view type derives, `_evidence/C__observer-hook.diff:70`) — the least API disruption of the three observer implementations.
- `filled`/`area` turn an impossible allocation into an error (`_evidence/C__ergonomic-constructors.diff:400-415`); the only candidate that does not abort.

**Defects**
- [major] The smoother copies two numeric policies: predict's two calls (`_evidence/C__rts-smoother.diff:295-296`) and the pivot-singularity expression (`_evidence/C__rts-smoother.diff:308-311` vs `C/src/filter.rs:236-238`). Change either in the filter and the smoother silently disagrees.
- [major] One new model input became a new type parameter on the public type and trait: `FilterAllocator<N, M, K>` and `LinearKalmanFilter<T, N, M, K>` (`_evidence/C__control-input.diff:123,155,176,194`). The one-file diff hides that the change is threaded through every generic bound; C's notes concede "Bounds sprawl" (`C/DESIGN_NOTES.md:68`).
- [major] The filter is bounded on `RealField`, not the crate's `KalmanScalar` (`C/src/filter.rs:74`). C's notes say the nonlinear rebuild will need a trait refactor and that the shim round-trips values through f64 (`C/DESIGN_NOTES.md:61-67`, `C/src/matrix/mod.rs:18,29`); every generic probe test had to import nalgebra and repeat four bounds (`_evidence/C__ergonomic-constructors.diff:434-435,497-502`).
- [minor] The observer's m×m buffer is allocated in `new` for every filter (`_evidence/C__observer-hook.diff:89`), and `update_inner<const OBSERVE: bool, F>` (`_evidence/C__observer-hook.diff:148`) changes the internal update shape for every future caller.
- [minor] The smoother is a free function for a reason that does not hold: an inherent impl may live in any module of the crate and the smoother only uses public accessors — A does exactly that (`_evidence/A__rts-smoother.diff:166,176`). See `_evidence/C__rts-smoother.notes.md:3`.
- [minor] The builder infers n from `f.len().isqrt()` and m from `h.len()/n` (`_evidence/C__ergonomic-constructors.diff:296-297`), so a malformed F/H surfaces as a length error from `new` and anyone changing the constructor must re-derive the rule.

**Ideas worth grafting**
- The `DESIGN_NOTES.md` shape: architecture, decisions with rejected alternatives, "what fought back", known weaknesses, "next changes", migration table (`C/DESIGN_NOTES.md:3,17,59,77,92,113`).
- `filled`/`area` with `try_reserve_exact` mapping allocation failure to an error (`_evidence/C__ergonomic-constructors.diff:400-415`).
- A parse-and-canonicalise PSD function that returns the checked matrix instead of a bool (`C/src/filter.rs:313-336`).
- A per-call observer callback (`_evidence/C__observer-hook.diff:138-148`): combined with B's S/L split it needs no stored observer, no copy and no extra buffer, and the filter keeps `Clone`.
- A builder that takes F and H and infers dimensions (`_evidence/C__ergonomic-constructors.diff:296-297`), with the error-indirection caveat.

### A

**Strengths**
- All four probes land in the filter's own idiom with modest production diffs: control as `predict_step(Option<&[T]>)` plus a B field (`_evidence/A__control-input.diff:167-184`); smoother as a module that reuses the filter's own kernels and checks (`_evidence/A__rts-smoother.diff:112`).
- The builder probe lifted the dimension rule into one function, `checked_model_dims`, used by `new` and the builder (`_evidence/A__ergonomic-constructors.diff:337`, called at `:250,:363`) — the best of the three answers to "one validated path" under the ergonomic probe.
- Real randomised test infrastructure: fixed-seed random PSD models and a Jacobi eigenvalue routine (`A/tests/linear_kf_robustness.rs:26-60,129-130`). Nothing in B or C can catch a new numeric regression like it.
- The filter is generic over `KalmanScalar` (`A/src/filter.rs:99`), and `invert_matrix` stays available in the same numeric world for the out-of-scope filters that use it (`A/src/matrix/mod.rs:186`, e.g. `A/src/unscented.rs:508`) — no f64 converting shim.

**Defects**
- [major] One invariant, two checks of different strength: the sound Cholesky test at construction (`A/src/validation.rs:26`) and the cheap necessary-conditions test used as the commit gate in `predict` and `update` (`A/src/filter.rs:79`, used at `:213,:292`). The cheap one let an indefinite P through (`_evidence/finding-A-psd.md:11-16`), and A also rejects ill-conditioned cases the consensus accepts (`_evidence/difftest.md:14-15`); a future PSD-policy change has to find both.
- [major] The tree is red at its tip: `A/tests/linear_kf_robustness.rs:129-130` fail (`_evidence/finding-A-psd.md:6-16`), so `cargo test` is not a usable baseline before any change.
- [major] No `DESIGN_NOTES.md`; `docs/DESIGN.md` is the unchanged baseline document (`A/docs/DESIGN.md:23`), violating `SPEC.md:296-297`.
- [minor] The same PSD failure maps to two different errors: `FilterDivergence` in predict (`A/src/filter.rs:213-217`) and `SingularMatrix` in update (`A/src/filter.rs:292-294`).
- [minor] The observer probe forced hand-written `Clone`/`Debug`, and clone silently drops the observer (`_evidence/A__observer-hook.diff:95,115`); the S copy sits between the numeric steps (`_evidence/A__observer-hook.diff:194`).
- [minor] Four public model accessors (`A/src/filter.rs:174-191`); only `transition()` is used by any probe — the other three are unused surface.
- [minor] `new_with_control` constructs the filter and then mutates it to install B (`_evidence/A__control-input.diff:86-92`), so the "one validated path" is a path plus a setter.

**Ideas worth grafting**
- `checked_model_dims` as the single dimension rule shared by `new` and every front end (`_evidence/A__ergonomic-constructors.diff:337`).
- The randomised PSD harness (`A/tests/linear_kf_robustness.rs:26-60`) — port it and keep it green.
- Bit-identical parity tests for an additive feature (`_evidence/A__control-input.diff:446`), so the old path is proved unchanged, not merely close.
- Keep the out-of-scope filters' inverse in the same scalar type rather than routing them through f64 (`A/src/matrix/mod.rs:186` vs `C/src/matrix/mod.rs:18,29`).

## Best skeleton

B. The split into validated values (`Model`, `Estimate`), pure steps that write to a separate `out`, and a shell that owns state and side effects is the only structure in which all four hidden features had an obvious home: control is a model field (`B/src/filter.rs:31-51`), the smoother is a pure step plus a batch method (`_evidence/B__rts-smoother.diff:76,206`), the observer is a shell method around the step (`_evidence/B__observer-hook.diff:155-171`), and the builder only assembles slices. It produced the lowest production churn, never copied S for a diagnostic, and never re-derived predict's arithmetic in the smoother. It also hands the next tournament a ready linearized update, `correct` (`B/src/filter.rs:197-243`), and uses the crate's `KalmanScalar`. The red build and missing notes are repairs the rewrite pays once; they do not change where the next change lands. C is the better-maintained tree today, but its skeleton makes each new model dimension a public type parameter and already caused two copied policies in one probe; A's skeleton carries two disagreeing checks of the same invariant on the hot path.

## Grafts

- From A: `checked_model_dims` — one overflow-and-zero rule shared by `new`, the builder and any future front end (`_evidence/A__ergonomic-constructors.diff:337`).
- From A: the randomised PSD test harness with a Jacobi eigensolver and fixed seeds (`A/tests/linear_kf_robustness.rs:26-60,129-130`); make it green before adding features.
- From A: the bit-identical parity test for additive features (`_evidence/A__control-input.diff:446`).
- From C: the design-notes template, especially "what fought back", "known weaknesses", "next changes" and the migration table (`C/DESIGN_NOTES.md:59,77,92,113`).
- From C: `filled`/`area` with `try_reserve_exact`, so a caller-supplied dimension that cannot be allocated is an error, not an abort (`_evidence/C__ergonomic-constructors.diff:400-415`).
- From C: a PSD check that returns the canonical matrix for the caller to store, not just a bool (`C/src/filter.rs:313-336`), so a tolerance-accepted input is stored symmetric.
- From C: the per-call observer callback (`_evidence/C__observer-hook.diff:138-148`); with B's `S`/factor split (`B/src/filter.rs:110-111`) it needs no stored observer, no copy and no extra buffer, and the filter keeps `Clone`. Keep A's registration form (`_evidence/A__observer-hook.diff:162`) only if a persistent observer is worth the `Clone`/`Send` cost.
- From C: builder construction from F and H with inferred dimensions (`_evidence/C__ergonomic-constructors.diff:296-297`), if error reporting can be made direct.

## Where the numbers mislead

- The probe `churn` means (A 713, B 584.75, C 563.25, `_evidence/probes.md:22-24`) are dominated by the maintainers' own new tests: A 2000, B 1629, C 1352 test lines against 852/710/793 production lines (`_evidence/*.diff`). Ordering by churn rewards writing fewer tests. On production code alone the order is B (710) < C (793) < A (852).
- `files` per probe counts `DESIGN_NOTES.md` edits for C and 2-line re-exports for A and B. Excluding docs it is C 1.75, A 2.25, B 2.25 per probe; but C's one-file control change threads a new type parameter through four generic declarations (`_evidence/C__control-input.diff:123,155,176,194`), which a file count cannot show.
- "4/4 probes green" means the new test files passed, not that the tree is green. A's own robustness suite is red and B's test target does not compile, so only C can verify a change by running the suite (`_evidence/full-test-status.md:13-15`).
- `passed` (A 140, B 144, C 137, `_evidence/metrics.md:5-7`) counts only the frozen contract plus `--lib`, not the whole tree. B's 144 looks highest while `cargo test` runs nothing; A's 140 hides the failing robustness test.
- LOC (A 965, B 1061, C 747, `_evidence/metrics.md:5-7`) mostly measures how much matrix layer each candidate deleted. It does not price C's per-signature generic cost, the 94-line f64-round-trip shim that must later be removed (`C/src/matrix/mod.rs:16-31`), or A's duplicated PSD check.
- The `cost $` and `turns` columns (C 5 turns, A 154, B 247, `_evidence/metrics.md:5-7`; probe costs all 0.00) measure the authoring runs, not the code; they are not comparable evidence about maintainability.
- `max rel err` and the status disagreements (`_evidence/difftest.md:14-15`) are correctness signals, not ease-of-change signals; note only that A's and B's extra `SingularMatrix` results point at gates a maintainer will have to revisit before trusting new numeric code.
