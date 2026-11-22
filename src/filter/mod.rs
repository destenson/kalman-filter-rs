//! The linear Kalman filter.
//!
//! One public type, [`LinearKalmanFilter`], with a numeric engine chosen by
//! feature. The default build uses the crate's dense row-major kernels; with
//! `--features nalgebra` the same type is served by an independent
//! nalgebra-backed engine. Both engines expose the same API and keep the same
//! guarantees.
//!
//! Storage is allocated once in [`LinearKalmanFilter::try_new`]: the model
//! (F, Q, H, R), the estimate (x, P) and a fixed workspace sized for n and m.
//! `predict` and `update` only write into that workspace; the candidate
//! estimate is committed, by swapping buffers rather than copying, once every
//! check has passed. A failed step therefore leaves the filter untouched, and
//! steady-state operation performs no heap allocation.
//!
//! Numerics (the same in both engines):
//! - predict: `x ← F x`, `P ← F P Fᵀ + Q`.
//! - update: `S = H P Hᵀ + R` is Cholesky-factored and reported singular when a
//!   pivot is small relative to its own diagonal entry of `S`; `S` is solved,
//!   never inverted; the covariance uses the Joseph form
//!   `P ← (I − K H) P (I − K H)ᵀ + K R Kᵀ`, a sum of PSD terms.
//! - `P`, `Q` and `R` are kept exactly symmetric after construction, and every
//!   committed covariance is checked to be PSD to working precision.
//! - Every tolerance is relative, never an absolute magnitude threshold.
//!
//! The two engines differ in one requirement on the scalar. The default engine
//! needs only [`KalmanScalar`]. With `--features nalgebra` the filter holds
//! nalgebra matrices, so `T` must also satisfy nalgebra's
//! [`RealField`](crate::nalgebra::RealField): `f32` and `f64` do, but a
//! third-party `KalmanScalar` that is not a `RealField` cannot use the feature.
//! Public signatures spell the active engine's bound directly; `FilterScalar`
//! is a crate-private name used by implementation helpers only.

use crate::error::{KalmanError, KalmanResult};

/// The active engine's additional scalar requirement, for private helpers only.
///
/// The default engine adds nothing beyond [`KalmanScalar`]; the nalgebra engine
/// adds [`RealField`](crate::nalgebra::RealField), which `f32` and `f64`
/// satisfy. Public signatures spell this bound directly; the alias only keeps
/// the implementation helpers single-source.
#[cfg(not(feature = "nalgebra"))]
pub(crate) use crate::types::KalmanScalar as FilterScalar;
#[cfg(feature = "nalgebra")]
pub(crate) use crate::nalgebra::RealField as FilterScalar;

/// The public module named by the specification: `kalman_filters::linear`.
pub mod linear {
    pub use super::LinearKalmanFilter;
    pub use crate::builder::{Covariance, LinearKalmanFilterBuilder};
}

#[cfg(not(feature = "nalgebra"))]
mod flat;
#[cfg(feature = "nalgebra")]
mod nalgebra;

#[cfg(not(feature = "nalgebra"))]
pub use self::flat::LinearKalmanFilter;
#[cfg(feature = "nalgebra")]
pub use self::nalgebra::LinearKalmanFilter;

fn diverged(what: &str) -> KalmanError {
    KalmanError::FilterDivergence(format!("{what} is not finite"))
}

/// `ZeroDimension` unless both dimensions are positive.
pub(crate) fn check_dims(n: usize, m: usize) -> KalmanResult<()> {
    if n == 0 || m == 0 { Err(KalmanError::ZeroDimension) } else { Ok(()) }
}

/// `Ok` if `data` holds exactly `rows × cols` elements. An overflowing product
/// can never equal a real slice length, so it is a `DimensionMismatch` too.
pub(crate) fn check_len<T>(data: &[T], rows: usize, cols: usize) -> KalmanResult<()> {
    match rows.checked_mul(cols) {
        Some(len) if len == data.len() => Ok(()),
        _ => Err(KalmanError::DimensionMismatch { expected: (rows, cols), actual: (data.len(), 1) }),
    }
}
