# Finding: candidate A commits an indefinite covariance on random models

Not found by the frozen suite, the differential test or the run's own measurement; found by
candidate A's own `tests/linear_kf_robustness.rs`, which is red at A's branch tip
(`4f0920d`).

Reproduce in `.tournament/linear-core/worktrees/A`:

    CARGO_HOME=... cargo test --test linear_kf_robustness

Both `random_models_stay_psd_{f32,f64}` fail, and they fail identically at A's tip with
the probe changes stashed, so it is A's code, not a probe artefact:

- f32: `P not PSD (step 1, n=5, m=4): λ ∈ [-9.801212976705923e-3, 8.470304071501664e0]`
- f64: `P not PSD (step 57, n=4, m=3): λ ∈ [-1.192451639179439e-15, 1.246683162822128e-13]`

The spec's bar (B11/B21) is `λ_min ≥ -tol·λ_max` with tol 1e-9 (f64) / 1e-5 (f32), so
both are violations by orders of magnitude, not rounding noise.

Why the test's own guard did not catch it: A's `variances_consistent` (a cheap O(n²)
check of non-negative diagonals and |p_ij| ≤ √(p_ii p_jj)) is necessary but not
sufficient for positive semidefiniteness. A P can satisfy it and still be indefinite.
It runs on the candidate covariance before `commit`, so an indefinite P is committed
with `Ok`.

Likely cause, for the build's triage: the Cholesky pivot test in
`filter.rs::singular_tol` is `64·m·ε` relative to each pivot's own diagonal, which
tolerates a condition number around 1/(64·m·ε) — about 2e13 in f64. A solve with an S
that ill-conditioned loses most of its digits, so K is wrong, and the Joseph sum no
longer restores a PSD result. The random models use per-component scales up to 10^±3 in
P0 and R, which is harsh but within "any valid model".

Possible fixes to weigh in synthesis (not decided here): a sufficient post-update PSD
test (one Cholesky of P per step, or a check that the smallest diagonal of the factor is
positive), a tighter relative pivot test, or a square-root/UD update that cannot produce
an indefinite P in the first place. The cost of any of these must be weighed against
B12 (no heap allocation) and the existing step cost.
