# Maintainer notes: A / rts-smoother

New `src/smoother.rs`: `impl LinearKalmanFilter<T> { pub fn smooth(&self, measurements: &[T]) -> KalmanResult<SmoothedPath<T>> }`. Measurements are a flat row-major slice, m values per step; `SmoothedPath` holds all steps' states (N×n) and covariances (N×n×n) with `state(k)`/`covariance(k)`/`states()`/`covariances()`/`steps()`/`state_dim()`. `smooth` clones the filter for the forward pass, so `self` is untouched. New `tests/rts_smoother.rs`, 8 tests.
Existing files touched minimally: `src/lib.rs` adds `pub mod smoother;` and a re-export; `src/filter.rs` widens three helpers to `pub(crate)` (no behaviour change).
Convention documented: step 0 is the filter's current estimate before step 0's update (x_{0|0}=x0, P_{0|0}=P0); later steps are predict+update.
Easy: predict/update already write into owned buffers, so a forward pass is a clone plus the ordinary calls, and the backward pass needed one triangular solve per step in the same style as the filter.
Hard: the backward pass needs F, Q and the predicted covariances, which the filter does not keep; the smoother has to re-derive them, and the filter's Cholesky test is reused for a quantity it was not designed for, so a numerically singular predicted covariance errors instead of being pseudo-inverted.
Verified: frozen 32/32, lib 108/108, new 8/8, `cargo build --all-targets` green, clippy clean on new files.

