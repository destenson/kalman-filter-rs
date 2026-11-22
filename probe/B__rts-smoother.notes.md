# Maintainer notes: B / rts-smoother

Landed in `src/filter.rs`: a private pure `smooth_step` beside `predict`/`update`/`correct`, and the public `linear::LinearKalmanFilter::smooth(&self, measurements: &[&[T]]) -> KalmanResult<SmoothedEstimates<T>>` with `steps()`, `state_dim()`, `state(k)`, `covariance(k)`. New `tests/rts_smoother.rs` (13). No DESIGN_NOTES.md in this copy, so the work started from the code and SPEC.md.
Convention: for each measurement, predict then update, starting from the filter's current estimate; the last smoothed step equals the last filtered step; the filter is left unchanged. Internals: the gain is solved through the predicted covariance's Cholesky factor, never inverted, with a small `SmoothWorkspace` for the n×n scratch and no second construction path.
Easy: Model/Estimate are validated immutable values, so the forward pass is the existing steps on copies and the backward pass is the textbook recursion through the existing Cholesky solve; no new construction path and no new matrix kernel.
Hard: the zero-allocation promise is per-step, not per-batch, so the smoother allocates history and result; and the n×n factor the backward pass needs had no home in the filter's m×m Workspace, so it got its own small workspace instead of widening the filter's.
Unsure: when a predicted covariance is numerically singular (SPEC model A pins one direction to rounding precision) `smooth` returns `SingularMatrix` instead of smoothing, although the filter runs that model; the crate's report-don't-repair stance was kept rather than adding a pseudo-inverse.
Verified: frozen 32, lib 112, new 13, doctests 10; textbook match to 1e-9 (f64) and 1e-4 (f32), smoothed trace <= filtered trace on 4 models, atomic rejection of bad measurements.

