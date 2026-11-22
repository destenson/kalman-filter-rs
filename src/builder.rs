//! Convenience construction for [`LinearKalmanFilter`]: a builder with
//! defaults and diagonal/scalar covariances, and a constant-velocity helper.
//!
//! Neither validates anything itself. Both only expand their inputs into
//! dense row-major slices and hand them to [`LinearKalmanFilter::try_new`], so
//! every path has exactly the same checks and errors.

use std::borrow::Cow;

use crate::error::{KalmanError, KalmanResult};
use crate::filter::{FilterScalar, LinearKalmanFilter, check_dims, check_len};
use crate::types::KalmanScalar;

/// A k×k covariance argument (k = n for P0 and Q, k = m for R).
#[derive(Clone, Copy, Debug)]
pub enum Covariance<'a, T> {
    /// Dense k×k, row-major; borrowed, never copied.
    Full(&'a [T]),
    /// The diagonal (length k) of a diagonal matrix.
    Diagonal(&'a [T]),
    /// `σ² · I`.
    Scalar(T),
}

impl<'a, T: KalmanScalar> Covariance<'a, T> {
    /// The dense k×k matrix. `Full` stays borrowed (`try_new` checks its
    /// length); the others are allocated without risk of aborting.
    fn to_dense(self, k: usize) -> KalmanResult<Cow<'a, [T]>> {
        match self {
            Covariance::Full(a) => Ok(Cow::Borrowed(a)),
            Covariance::Diagonal(d) => {
                check_len(d, k, 1)?;
                let mut a = zeros(k, k)?;
                d.iter().enumerate().for_each(|(i, &v)| a[i * k + i] = v);
                Ok(Cow::Owned(a))
            }
            Covariance::Scalar(s) => {
                let mut a = zeros(k, k)?;
                (0..k).for_each(|i| a[i * k + i] = s);
                Ok(Cow::Owned(a))
            }
        }
    }
}

/// A `rows × cols` zero matrix. An overflowing or unallocatable size is a
/// `DimensionMismatch` (the dimensions cannot describe a real matrix) rather
/// than a process abort.
fn zeros<T: KalmanScalar>(rows: usize, cols: usize) -> KalmanResult<Vec<T>> {
    let too_big = || KalmanError::DimensionMismatch { expected: (rows, cols), actual: (0, 1) };
    let len = rows.checked_mul(cols).ok_or_else(too_big)?;
    let mut v = Vec::new();
    v.try_reserve_exact(len).map_err(|_| too_big())?;
    v.resize(len, T::zero());
    Ok(v)
}

/// Builder for [`LinearKalmanFilter`]; see [`LinearKalmanFilter::builder`].
#[derive(Clone, Copy, Debug)]
pub struct LinearKalmanFilterBuilder<'a, T> {
    n: usize,
    m: usize,
    f: &'a [T],
    h: &'a [T],
    x0: Option<&'a [T]>,
    p0: Option<Covariance<'a, T>>,
    q: Option<Covariance<'a, T>>,
    r: Option<Covariance<'a, T>>,
}

impl<'a, T: KalmanScalar> LinearKalmanFilterBuilder<'a, T> {
    /// Initial state (length n). Default: zeros.
    pub fn x0(self, x0: &'a [T]) -> Self {
        Self { x0: Some(x0), ..self }
    }

    /// Initial covariance P0 (n×n). Required.
    pub fn p0(self, p0: Covariance<'a, T>) -> Self {
        Self { p0: Some(p0), ..self }
    }

    /// Process noise Q (n×n). Required: there is no silent `Q = 0`.
    pub fn q(self, q: Covariance<'a, T>) -> Self {
        Self { q: Some(q), ..self }
    }

    /// Measurement noise R (m×m). Required.
    pub fn r(self, r: Covariance<'a, T>) -> Self {
        Self { r: Some(r), ..self }
    }

}

/// Expands the inputs and calls [`LinearKalmanFilter::try_new`].
///
/// Errors: `BuilderIncomplete` (p0, q or r not set), plus exactly those of
/// `try_new`. A wrong `Diagonal` length is a `DimensionMismatch`.
#[cfg(not(feature = "nalgebra"))]
impl<'a, T: KalmanScalar> LinearKalmanFilterBuilder<'a, T> {
    pub fn try_build(self) -> KalmanResult<LinearKalmanFilter<T>> {
        try_build_impl(self)
    }
}

/// Expands the inputs and calls [`LinearKalmanFilter::try_new`].
///
/// Errors: `BuilderIncomplete` (p0, q or r not set), plus exactly those of
/// `try_new`. A wrong `Diagonal` length is a `DimensionMismatch`.
#[cfg(feature = "nalgebra")]
impl<'a, T: KalmanScalar + crate::nalgebra::RealField> LinearKalmanFilterBuilder<'a, T> {
    pub fn try_build(self) -> KalmanResult<LinearKalmanFilter<T>> {
        try_build_impl(self)
    }
}

fn try_build_impl<'a, T: KalmanScalar + FilterScalar>(
    builder: LinearKalmanFilterBuilder<'a, T>,
) -> KalmanResult<LinearKalmanFilter<T>> {
    let LinearKalmanFilterBuilder { n, m, f, h, x0, p0, q, r } = builder;
    let missing = |name: &str| KalmanError::BuilderIncomplete(name.into());
    let p0 = p0.ok_or_else(|| missing("p0"))?;
    let q = q.ok_or_else(|| missing("q"))?;
    let r = r.ok_or_else(|| missing("r"))?;
    // F and H bound n·n and m·n by real slice lengths before anything is
    // allocated from n or m.
    check_dims(n, m)?;
    check_len(f, n, n)?;
    check_len(h, m, n)?;
    let x0 = match x0 {
        Some(x0) => Cow::Borrowed(x0),
        None => Cow::Owned(zeros(n, 1)?),
    };
    let (p0, q, r) = (p0.to_dense(n)?, q.to_dense(n)?, r.to_dense(m)?);
    LinearKalmanFilter::try_new(n, m, &x0, &p0, f, &q, h, &r)
}


#[cfg(not(feature = "nalgebra"))]
impl<T: KalmanScalar> LinearKalmanFilter<T> {
    /// Starts a builder for the model `F` (n×n) and `H` (m×n). Nothing is
    /// checked until [`try_build`](LinearKalmanFilterBuilder::try_build).
    ///
    /// ```
    /// use kalman_filters::linear::{Covariance, LinearKalmanFilter};
    /// let kf = LinearKalmanFilter::<f64>::builder(2, 1, &[1.0, 0.1, 0.0, 1.0], &[1.0, 0.0])
    ///     .p0(Covariance::Scalar(1.0))
    ///     .q(Covariance::Diagonal(&[1e-4, 1e-2]))
    ///     .r(Covariance::Scalar(0.25))
    ///     .try_build()?;
    /// assert_eq!(kf.state(), &[0.0, 0.0]);
    /// # Ok::<(), kalman_filters::KalmanError>(())
    /// ```
    pub fn builder<'a>(
        state_dim: usize,
        measurement_dim: usize,
        f: &'a [T],
        h: &'a [T],
    ) -> LinearKalmanFilterBuilder<'a, T> {
        builder_impl(state_dim, measurement_dim, f, h)
    }

    /// Constant-velocity model with `axes` independent axes, state
    /// `[pos₀, vel₀, pos₁, vel₁, …]` (n = 2·axes), positions measured (m = axes).
    /// Per axis `F = [[1, dt], [0, 1]]`, `Q = accel_var · [[dt⁴/4, dt³/2], [dt³/2, dt²]]`
    /// (white-noise acceleration), `R = meas_var · I`, `x0 = 0`.
    ///
    /// Errors are those of [`try_new`](Self::try_new): e.g. a non-finite `dt`
    /// is `NonFiniteInput`, a negative `accel_var` or `meas_var` is
    /// `InvalidNoiseCovariance`.
    pub fn try_constant_velocity(
        axes: usize,
        dt: T,
        accel_var: T,
        meas_var: T,
        p0: Covariance<'_, T>,
    ) -> KalmanResult<Self> {
        try_constant_velocity_impl(axes, dt, accel_var, meas_var, p0)
    }
}

#[cfg(feature = "nalgebra")]
impl<T: KalmanScalar + crate::nalgebra::RealField> LinearKalmanFilter<T> {
    /// Starts a builder for the model `F` (n×n) and `H` (m×n). Nothing is
    /// checked until [`try_build`](LinearKalmanFilterBuilder::try_build).
    ///
    /// ```
    /// use kalman_filters::linear::{Covariance, LinearKalmanFilter};
    /// let kf = LinearKalmanFilter::<f64>::builder(2, 1, &[1.0, 0.1, 0.0, 1.0], &[1.0, 0.0])
    ///     .p0(Covariance::Scalar(1.0))
    ///     .q(Covariance::Diagonal(&[1e-4, 1e-2]))
    ///     .r(Covariance::Scalar(0.25))
    ///     .try_build()?;
    /// assert_eq!(kf.state(), &[0.0, 0.0]);
    /// # Ok::<(), kalman_filters::KalmanError>(())
    /// ```
    pub fn builder<'a>(
        state_dim: usize,
        measurement_dim: usize,
        f: &'a [T],
        h: &'a [T],
    ) -> LinearKalmanFilterBuilder<'a, T> {
        builder_impl(state_dim, measurement_dim, f, h)
    }

    /// Constant-velocity model with `axes` independent axes, state
    /// `[pos₀, vel₀, pos₁, vel₁, …]` (n = 2·axes), positions measured (m = axes).
    /// Per axis `F = [[1, dt], [0, 1]]`, `Q = accel_var · [[dt⁴/4, dt³/2], [dt³/2, dt²]]`
    /// (white-noise acceleration), `R = meas_var · I`, `x0 = 0`.
    ///
    /// Errors are those of [`try_new`](Self::try_new): e.g. a non-finite `dt`
    /// is `NonFiniteInput`, a negative `accel_var` or `meas_var` is
    /// `InvalidNoiseCovariance`.
    pub fn try_constant_velocity(
        axes: usize,
        dt: T,
        accel_var: T,
        meas_var: T,
        p0: Covariance<'_, T>,
    ) -> KalmanResult<Self> {
        try_constant_velocity_impl(axes, dt, accel_var, meas_var, p0)
    }
}

fn builder_impl<'a, T: KalmanScalar>(
    state_dim: usize,
    measurement_dim: usize,
    f: &'a [T],
    h: &'a [T],
) -> LinearKalmanFilterBuilder<'a, T> {
    LinearKalmanFilterBuilder { n: state_dim, m: measurement_dim, f, h, x0: None, p0: None, q: None, r: None }
}

fn try_constant_velocity_impl<T: KalmanScalar + FilterScalar>(
    axes: usize,
    dt: T,
    accel_var: T,
    meas_var: T,
    p0: Covariance<'_, T>,
) -> KalmanResult<LinearKalmanFilter<T>> {
    let n = axes.checked_mul(2).ok_or(KalmanError::DimensionMismatch { expected: (axes, 2), actual: (0, 1) })?;
    let (mut f, mut q, mut h) = (zeros(n, n)?, zeros(n, n)?, zeros(axes, n)?);
    let half = T::one() / (T::one() + T::one());
    let dt2 = dt * dt;
    let (q_pp, q_pv, q_vv) = (accel_var * dt2 * dt2 * half * half, accel_var * dt2 * dt * half, accel_var * dt2);
    for a in 0..axes {
        let (p, v) = (2 * a, 2 * a + 1);
        f[p * n + p] = T::one();
        f[p * n + v] = dt;
        f[v * n + v] = T::one();
        q[p * n + p] = q_pp;
        q[p * n + v] = q_pv;
        q[v * n + p] = q_pv;
        q[v * n + v] = q_vv;
        h[a * n + p] = T::one();
    }
    LinearKalmanFilter::<T>::builder(n, axes, &f, &h).p0(p0).q(Covariance::Full(&q)).r(Covariance::Scalar(meas_var)).try_build()
}

#[cfg(test)]
mod tests {
    use super::*;

    const F: [f64; 4] = [1.0, 0.1, 0.0, 1.0];
    const H: [f64; 2] = [1.0, 0.0];

    fn same(a: &LinearKalmanFilter<f64>, b: &LinearKalmanFilter<f64>) -> bool {
        a.state() == b.state() && a.covariance() == b.covariance()
    }

    #[test]
    fn builder_equals_try_new_bit_for_bit() {
        let b = LinearKalmanFilter::builder(2, 1, &F, &H)
            .x0(&[1.0, 2.0])
            .p0(Covariance::Scalar(2.0))
            .q(Covariance::Diagonal(&[0.1, 0.2]))
            .r(Covariance::Full(&[0.5]))
            .try_build()
            .unwrap();
        let d = LinearKalmanFilter::try_new(2, 1, &[1.0, 2.0], &[2.0, 0.0, 0.0, 2.0], &F, &[0.1, 0.0, 0.0, 0.2], &H, &[0.5])
            .unwrap();
        assert!(same(&b, &d));
        let (mut b, mut d) = (b, d);
        for kf in [&mut b, &mut d] {
            kf.predict().unwrap();
            kf.update(&[1.5]).unwrap();
        }
        assert!(same(&b, &d));
    }

    #[test]
    fn constant_velocity_equals_try_new_and_tracks_a_line() {
        let dt = 0.5f64; // a power of two: the reference Q below is exact
        let cv = LinearKalmanFilter::try_constant_velocity(1, dt, 0.2, 0.3, Covariance::Scalar(10.0)).unwrap();
        let (q_pp, q_pv, q_vv) = (0.2 * dt.powi(4) / 4.0, 0.2 * dt.powi(3) / 2.0, 0.2 * dt * dt);
        let d = LinearKalmanFilter::try_new(
            2, 1, &[0.0; 2], &[10.0, 0.0, 0.0, 10.0], &[1.0, dt, 0.0, 1.0], &[q_pp, q_pv, q_pv, q_vv], &[1.0, 0.0], &[0.3],
        )
        .unwrap();
        assert!(same(&cv, &d));

        // x(t) = 3 + 2t: the velocity estimate converges to 2.
        let mut kf = LinearKalmanFilter::try_constant_velocity(2, 0.1, 1e-4, 1e-2, Covariance::Scalar(100.0)).unwrap();
        for k in 0..400 {
            let t = k as f64 * 0.1;
            kf.predict().unwrap();
            kf.update(&[3.0 + 2.0 * t, -t]).unwrap();
        }
        let x = kf.state();
        assert!((x[1] - 2.0).abs() < 1e-2 && (x[3] + 1.0).abs() < 1e-2, "{x:?}");
    }

    #[test]
    fn missing_fields_are_builder_incomplete() {
        let b = LinearKalmanFilter::<f64>::builder(2, 1, &F, &H);
        for (kf, field) in [
            (b.q(Covariance::Scalar(1.0)).r(Covariance::Scalar(1.0)), "p0"),
            (b.p0(Covariance::Scalar(1.0)).r(Covariance::Scalar(1.0)), "q"),
            (b.p0(Covariance::Scalar(1.0)).q(Covariance::Scalar(1.0)), "r"),
        ] {
            assert!(matches!(kf.try_build(), Err(KalmanError::BuilderIncomplete(f)) if f == field));
        }
    }

    #[test]
    fn bad_inputs_are_errors_not_panics() {
        let s = Covariance::Scalar(1.0f32);
        // Huge dimensions: rejected by the F/H length checks before allocating.
        let e = LinearKalmanFilter::builder(2, usize::MAX, &[1.0f32; 4], &[1.0; 2]).p0(s).q(s).r(s).try_build();
        assert!(matches!(e, Err(KalmanError::DimensionMismatch { .. })), "{e:?}");
        for axes in [usize::MAX, usize::MAX / 2, 1 << 40] {
            let e = LinearKalmanFilter::try_constant_velocity(axes, 0.1f32, 1.0, 1.0, s);
            assert!(matches!(e, Err(KalmanError::DimensionMismatch { .. })), "{e:?}");
        }
        let e = LinearKalmanFilter::builder(2, 1, &F, &H).p0(Covariance::Diagonal(&[1.0])).q(Covariance::Scalar(1.0))
            .r(Covariance::Scalar(1.0)).try_build();
        assert!(matches!(e, Err(KalmanError::DimensionMismatch { .. })), "{e:?}");
        let e = LinearKalmanFilter::builder(0, 1, &[], &[]).p0(s).q(s).r(s).try_build();
        assert!(matches!(e, Err(KalmanError::ZeroDimension)), "{e:?}");

        let p0 = Covariance::Scalar(1.0);
        let e = LinearKalmanFilter::try_constant_velocity(1, f64::NAN, 1.0, 1.0, p0);
        assert!(matches!(e, Err(KalmanError::NonFiniteInput)), "{e:?}");
        let e = LinearKalmanFilter::try_constant_velocity(1, 0.1, -1.0, 1.0, p0);
        assert!(matches!(e, Err(KalmanError::InvalidNoiseCovariance)), "{e:?}");
        let e = LinearKalmanFilter::try_constant_velocity(1, 0.1, 1.0, 1.0, Covariance::Scalar(-1.0));
        assert!(matches!(e, Err(KalmanError::InvalidCovariance)), "{e:?}");
    }
}
