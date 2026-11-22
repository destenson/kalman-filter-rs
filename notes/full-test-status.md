# Full test-suite status at each candidate's branch tip

Measured locally on the candidate commits (no probe changes applied), with

    cargo test --no-fail-fast

in each worktree. The run's own `metrics.md` exercises only the frozen suite plus
`--lib`; this file records what the rest of each tree does, because the judges cannot run
cargo themselves.

| candidate | `cargo test --no-fail-fast` | detail |
|---|---|---|
| A | fails | `tests/linear_kf_robustness.rs::random_models_stay_psd_{f32,f64}` fail: A commits a covariance with a negative eigenvalue, for example f64 `λ ∈ [-1.19e-15, 1.25e-13]` (tolerance is 1e-9 relative) and f32 `λ ∈ [-9.8e-3, 8.47]` (tolerance 1e-5). Everything else in A's tree passes: frozen 32, lib 108, and 12 further integration tests. |
| B | fails to compile | `tests/functional_tests.rs` still imports `kalman_filters::filter::KalmanFilter`, which B removed. Because the test target does not compile, `cargo test` runs nothing at all. `cargo test --test spec_linear_kf` (32) and `--lib` (112) pass when run on their own. |
| C | passes | every target in the tree builds and passes: frozen 32, lib 105, and eight integration suites (11, 12, 12, 6, 7, 8, 3, 3 tests). `cargo build --all-targets` is green. |

Consequences worth holding on to:

- A's failure is a real violation of B21 ("every `Ok` update leaves P PSD within the B11
  tolerances, on any valid model"), not rounding noise, and it is invisible to the frozen
  suite, which only exercises models A and B.
- B's failure is a build break in a test file it touched: the spec allows mechanical
  porting of old tests, but B stopped half way.
- C is the only candidate whose whole tree is green. Its status as the rejected base is
  the owner's dependency decision (nalgebra must stay optional), not a correctness one.
