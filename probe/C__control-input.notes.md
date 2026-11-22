# Maintainer notes: C / control-input

Landed in `src/filter.rs`: a third dimension parameter `K` (default `Dyn`) and `FilterAllocator<N, M, K = Dyn>`, an optional `b: Option<OMatrix<T, N, K>>` field plus a preallocated `u` buffer in `Workspace`, `new_with_control(n, m, k, ..., &b)` sharing the private `build` with `new`, `control_dim()`, and `predict_with_control(u)` calling the same `predict_impl` that `predict()` calls with no control term. New tests in `tests/control_input.rs` (13, including the constant-acceleration analytic case: 10 steps, Q = 0, p = 100, v = 20). DESIGN_NOTES.md updated. Frozen files untouched.
Easy: predict is still one `gemv` into the workspace, so the control term is a second `gemv` (beta = 1) into the same candidate x; B and u slot into the existing workspace-plus-single-build validation, and the no-B path executes exactly the statements it executed before (frozen tests, plus an allocation-count test showing zero allocations over 100 control-aware cycles).
Hard: K had to become a type parameter to keep static storage, and the workspace u buffer must be sized with `K::try_to_usize().unwrap_or(0)` rather than a literal 0, because `Dim::from_usize` asserts on a mismatched Const.
Unsure: `predict()` on a filter built with B applies u = 0 instead of erroring (documented and tested); "this filter has no control matrix" is reported as `DimensionMismatch { expected: (0, 1), .. }` rather than a new error variant.
Verified: frozen 32, lib 105, new 13, full `cargo test`, doctests 12, `cargo build --all-targets` green, clippy clean in the changed files.

