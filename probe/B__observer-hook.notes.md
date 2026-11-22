# Maintainer notes: B / observer-hook

Landed in `src/filter.rs`: new public `linear::{InnovationObserver, InnovationReport}` and `LinearKalmanFilter::{set_observer, clear_observer}`; `update` reports after a successful commit, reusing the y, S and Cholesky factor left in the workspace. `src/matrix/mod.rs` widens `dot` to `pub(crate)`. New `tests/observer_tests.rs` (8).
Behavior: innovation y, covariance S and NIS = yᵀS⁻¹y are reported after each successful update only. The observer is any FnMut over InnovationReport or a hand-written InnovationObserver; it is stored boxed (Send + Sync + 'static) so the filter stays Send + Sync. NIS reuses the update's Cholesky factor and a scratch Vec allocated at registration, so steady-state predict/update allocate zero with and without an observer.
Easy: the update already left y and the S factor in the workspace, so reporting is a call plus a scratch vector; the model/estimate split made the observer a property of the filter without touching validation.
Hard: S is consumed by the factorisation, so the report's S had to be rebuilt or copied and NIS needs one extra solve; storing a boxed observer cost the derived `Clone` (replaced by a manual `Debug`) and needed `Send + Sync` bounds to keep the filter movable.
Unsure: whether the registration API on the filter is better than a per-call callback, given the Clone/Send consequences.
Verified: new 8 (including the 5,000-step mean-NIS test: 1.9986 for m = 2, inside 10%), frozen 32, lib 112, doctests 10. `--all-targets` red only on tests/functional_tests.rs, pre-existing.

