//! The linear Kalman filter.
//!
//! Two layers:
//!
//! * **Core** (this module's private items): validated, immutable values — [`Model`]
//!   (F, Q, H, R) and [`Estimate`] (x, P) — and the pure step functions [`predict`],
//!   [`update`] and [`correct`]. A step reads a model and a prior estimate and writes
//!   the posterior into a separate `out` estimate, or returns an error. It never
//!   touches the prior, logs nothing and allocates nothing; the scratch it needs is
//!   passed in as a [`Workspace`] whose contents mean nothing between calls.
//! * **Shell** ([`linear::LinearKalmanFilter`]): owns one model, the current estimate,
//!   a spare estimate and a workspace. It runs a step into the spare and swaps it in
//!   only if the step succeeded, so a failed step leaves the filter unchanged by
//!   construction. Logging lives here, around the math.

use crate::error::{KalmanError, KalmanResult};
use crate::matrix;
use crate::types::KalmanScalar;
use crate::validation;
use num_traits::Float;

// ---------------------------------------------------------------------------
// Core: values
// ---------------------------------------------------------------------------

/// A validated linear model: x' = F x + w, z = H x + v, w ~ N(0, Q), v ~ N(0, R).
///
/// Invariants (established by [`Model::new`], immutable afterwards): n, m ≥ 1; all
/// matrices have the right length and finite entries; Q and R are symmetric PSD.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Model<T> {
    n: usize,
    m: usize,
    f: Vec<T>,
    q: Vec<T>,
    h: Vec<T>,
    r: Vec<T>,
}

impl<T: KalmanScalar> Model<T> {
    /// Validates and copies the model matrices (row-major: f, q n×n; h m×n; r m×m).
    pub(crate) fn new(n: usize, m: usize, f: &[T], q: &[T], h: &[T], r: &[T]) -> KalmanResult<Self> {
        validation::dimensions(n, m)?;
        validation::matrix("f", f, n, n)?;
        validation::matrix("q", q, n, n)?;
        validation::matrix("h", h, m, n)?;
        validation::matrix("r", r, m, m)?;
        validation::covariance(q, n, KalmanError::InvalidNoiseCovariance)?;
        validation::covariance(r, m, KalmanError::InvalidNoiseCovariance)?;
        Ok(Self { n, m, f: f.to_vec(), q: q.to_vec(), h: h.to_vec(), r: r.to_vec() })
    }

    pub(crate) fn state_dim(&self) -> usize {
        self.n
    }
    pub(crate) fn measurement_dim(&self) -> usize {
        self.m
    }
    pub(crate) fn f(&self) -> &[T] {
        &self.f
    }
    pub(crate) fn q(&self) -> &[T] {
        &self.q
    }
    pub(crate) fn h(&self) -> &[T] {
        &self.h
    }
    pub(crate) fn r(&self) -> &[T] {
        &self.r
    }
}

/// A Gaussian state estimate: mean x (n) and covariance P (n×n, row-major).
///
/// Invariant: finite, and P symmetric PSD (checked at construction, preserved by the
/// step functions, which only ever produce finite symmetric Joseph-form covariances).
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Estimate<T> {
    x: Vec<T>,
    p: Vec<T>,
}

impl<T: KalmanScalar> Estimate<T> {
    /// Validates and copies an initial estimate for an n-dimensional state.
    pub(crate) fn new(n: usize, x: &[T], p: &[T]) -> KalmanResult<Self> {
        validation::matrix("x0", x, n, 1)?;
        validation::matrix("p0", p, n, n)?;
        validation::covariance(p, n, KalmanError::InvalidCovariance)?;
        Ok(Self { x: x.to_vec(), p: p.to_vec() })
    }

    pub(crate) fn x(&self) -> &[T] {
        &self.x
    }
    pub(crate) fn p(&self) -> &[T] {
        &self.p
    }
}

/// Scratch buffers for one (n, m) pair, so the step functions need not allocate.
/// Holds no information between calls.
#[derive(Debug, Clone)]
pub(crate) struct Workspace<T> {
    fp: Vec<T>,  // n×n  F·P
    a: Vec<T>,   // n×n  I − K·H
    ap: Vec<T>,  // n×n  (I − K·H)·P
    pht: Vec<T>, // n×m  P·Hᵀ
    k: Vec<T>,   // n×m  gain
    kr: Vec<T>,  // n×m  K·R
    s: Vec<T>,   // m×m  innovation covariance
    l: Vec<T>,   // m×m  Cholesky factor of S
    y: Vec<T>,   // m    innovation
}

impl<T: KalmanScalar> Workspace<T> {
    pub(crate) fn new(n: usize, m: usize) -> Self {
        let zeros = |len| vec![T::zero(); len];
        Self {
            fp: zeros(n * n),
            a: zeros(n * n),
            ap: zeros(n * n),
            pht: zeros(n * m),
            k: zeros(n * m),
            kr: zeros(n * m),
            s: zeros(m * m),
            l: zeros(m * m),
            y: zeros(m),
        }
    }
}

// ---------------------------------------------------------------------------
// Core: step functions
// ---------------------------------------------------------------------------
//
// Contract shared by all steps: `prior` and `out` have the model's state dimension,
// `ws` was made for the model's (n, m). On `Ok`, `out` holds a finite posterior with a
// symmetric covariance. On `Err`, `out` holds garbage and must be discarded.

/// Time update: x' = F x, P' = F P Fᵀ + Q.
///
/// Errors: `FilterDivergence` if the result overflows to a non-finite value.
pub(crate) fn predict<T: KalmanScalar>(
    model: &Model<T>,
    prior: &Estimate<T>,
    ws: &mut Workspace<T>,
    out: &mut Estimate<T>,
) -> KalmanResult<()> {
    let n = model.n;
    matrix::mul_vec(&model.f, &prior.x, &mut out.x, n, n);
    matrix::mul(&model.f, &prior.p, &mut ws.fp, n, n, n);
    matrix::mul_bt(&ws.fp, &model.f, &mut out.p, n, n, n);
    for (p, &q) in out.p.iter_mut().zip(&model.q) {
        *p = *p + q;
    }
    matrix::symmetrize(&mut out.p, n);
    require_finite(out, "predict overflowed to a non-finite estimate")
}

/// Measurement update with the model's H and R.
///
/// Errors: `DimensionMismatch` if `z.len() != m`; `InvalidInput` if `z` is not
/// finite; otherwise those of [`correct`].
pub(crate) fn update<T: KalmanScalar>(
    model: &Model<T>,
    prior: &Estimate<T>,
    z: &[T],
    ws: &mut Workspace<T>,
    out: &mut Estimate<T>,
) -> KalmanResult<()> {
    let (n, m) = (model.n, model.m);
    if z.len() != m {
        return Err(KalmanError::DimensionMismatch { expected: (m, 1), actual: (z.len(), 1) });
    }
    validation::finite("z", z)?;
    let mut y = core::mem::take(&mut ws.y);
    matrix::mul_vec(&model.h, &prior.x, &mut y, m, n);
    for (yi, &zi) in y.iter_mut().zip(z) {
        *yi = zi - *yi;
    }
    let result = correct(prior, &model.h, &model.r, &y, ws, out);
    ws.y = y;
    result
}

/// Corrects `prior` with innovation `y` (= z − predicted measurement) through the
/// linearization `h` (m×n) with noise `r` (m×m). Shared by every filter whose update is
/// linear or linearized.
///
///   S = H P Hᵀ + R           solved through its Cholesky factor, never inverted
///   K = P Hᵀ S⁻¹             row i of K solves S kᵢ = (P Hᵀ)ᵢ
///   x' = x + K y
///   P' = (I − K H) P (I − K H)ᵀ + K R Kᵀ     (Joseph form: PSD by construction)
///
/// Errors: `SingularMatrix` if S is singular relative to its own diagonal
/// (scale- and unit-invariant); `FilterDivergence` if the result is non-finite.
pub(crate) fn correct<T: KalmanScalar>(
    prior: &Estimate<T>,
    h: &[T],
    r: &[T],
    y: &[T],
    ws: &mut Workspace<T>,
    out: &mut Estimate<T>,
) -> KalmanResult<()> {
    let (n, m) = (prior.x.len(), y.len());
    if !y.iter().all(|v| v.is_finite()) {
        return Err(divergence("innovation overflowed to a non-finite value"));
    }

    // S = H (P Hᵀ) + R
    matrix::mul_bt(&prior.p, h, &mut ws.pht, n, n, m);
    matrix::mul(h, &ws.pht, &mut ws.s, m, n, m);
    for (s, &ri) in ws.s.iter_mut().zip(r) {
        *s = *s + ri;
    }
    matrix::cholesky(&ws.s, &mut ws.l, m, matrix::relative_tolerance(m))?;

    // K = P Hᵀ S⁻¹, one row at a time (S is symmetric).
    ws.k.copy_from_slice(&ws.pht);
    for row in ws.k.chunks_exact_mut(m) {
        matrix::cholesky_solve(&ws.l, m, row);
    }

    // x' = x + K y
    matrix::mul_vec(&ws.k, y, &mut out.x, n, m);
    for (xo, &xi) in out.x.iter_mut().zip(&prior.x) {
        *xo = *xo + xi;
    }

    // P' = A P Aᵀ + K R Kᵀ, A = I − K H
    matrix::mul(&ws.k, h, &mut ws.a, n, m, n);
    for (idx, a) in ws.a.iter_mut().enumerate() {
        let identity = if idx % (n + 1) == 0 { T::one() } else { T::zero() };
        *a = identity - *a;
    }
    matrix::mul(&ws.a, &prior.p, &mut ws.ap, n, n, n);
    matrix::mul_bt(&ws.ap, &ws.a, &mut out.p, n, n, n);
    matrix::mul(&ws.k, r, &mut ws.kr, n, m, m);
    matrix::add_mul_bt(&ws.kr, &ws.k, &mut out.p, n, m, n);
    matrix::symmetrize(&mut out.p, n);

    require_finite(out, "update produced a non-finite estimate")
}

fn require_finite<T: Float>(e: &Estimate<T>, what: &'static str) -> KalmanResult<()> {
    if e.x.iter().chain(&e.p).all(|v| v.is_finite()) { Ok(()) } else { Err(divergence(what)) }
}

fn divergence(what: &str) -> KalmanError {
    KalmanError::FilterDivergence(what.into())
}

// ---------------------------------------------------------------------------
// Shell
// ---------------------------------------------------------------------------

/// The public linear Kalman filter (`kalman_filters::linear`).
pub mod linear {
    use super::{Estimate, Model, Workspace};
    use crate::error::KalmanResult;
    use crate::types::KalmanScalar;

    /// A linear Kalman filter with dimensions fixed at construction.
    ///
    /// Construction validates everything once; afterwards the model is immutable and
    /// the estimate changes only through [`predict`](Self::predict) and
    /// [`update`](Self::update), each of which either succeeds or leaves the filter
    /// exactly as it was. After construction neither step allocates.
    #[derive(Debug, Clone)]
    pub struct LinearKalmanFilter<T: KalmanScalar> {
        model: Model<T>,
        estimate: Estimate<T>,
        spare: Estimate<T>,
        ws: Workspace<T>,
    }

    impl<T: KalmanScalar> LinearKalmanFilter<T> {
        /// Builds a filter from dense row-major slices: `x0` n, `p0` n×n, `f` n×n,
        /// `q` n×n, `h` m×n, `r` m×m.
        ///
        /// # Errors
        /// * `InvalidInput` — a zero dimension, or a NaN/±∞ in any input;
        /// * `DimensionMismatch` — a slice of the wrong length (including dimension
        ///   products that overflow `usize`);
        /// * `InvalidCovariance` — `p0` is not symmetric positive semi-definite;
        /// * `InvalidNoiseCovariance` — `q` or `r` is not symmetric PSD.
        ///
        /// All PSD and symmetry tolerances are relative to each matrix's own scale.
        #[allow(clippy::too_many_arguments)]
        pub fn new(
            state_dim: usize,
            measurement_dim: usize,
            x0: &[T],
            p0: &[T],
            f: &[T],
            q: &[T],
            h: &[T],
            r: &[T],
        ) -> KalmanResult<Self> {
            let model = Model::new(state_dim, measurement_dim, f, q, h, r)?;
            let estimate = Estimate::new(state_dim, x0, p0)?;
            Ok(Self {
                spare: estimate.clone(),
                ws: Workspace::new(state_dim, measurement_dim),
                model,
                estimate,
            })
        }

        /// State dimension n.
        pub fn state_dim(&self) -> usize {
            self.model.state_dim()
        }

        /// Measurement dimension m.
        pub fn measurement_dim(&self) -> usize {
            self.model.measurement_dim()
        }

        /// Time update: x ← F x, P ← F P Fᵀ + Q.
        ///
        /// # Errors
        /// `FilterDivergence` if the result would be non-finite; the filter is unchanged.
        pub fn predict(&mut self) -> KalmanResult<()> {
            let result = super::predict(&self.model, &self.estimate, &mut self.ws, &mut self.spare);
            self.commit("predict", result)
        }

        /// Measurement update with measurement `z` (length m), Joseph form.
        ///
        /// # Errors
        /// `DimensionMismatch` (wrong length), `InvalidInput` (non-finite `z`),
        /// `SingularMatrix` (innovation covariance numerically singular),
        /// `FilterDivergence` (non-finite result). On any error the filter is unchanged.
        pub fn update(&mut self, z: &[T]) -> KalmanResult<()> {
            let result = super::update(&self.model, &self.estimate, z, &mut self.ws, &mut self.spare);
            self.commit("update", result)
        }

        /// State estimate x (length n).
        pub fn state(&self) -> &[T] {
            self.estimate.x()
        }

        /// State covariance P (n×n, row-major).
        pub fn covariance(&self) -> &[T] {
            self.estimate.p()
        }

        /// The model matrices, for code that converts this filter to another form.
        pub(crate) fn model(&self) -> &Model<T> {
            &self.model
        }

        /// Adopts a successful step's result; reports (and logs) a failed one.
        fn commit(&mut self, step: &str, result: KalmanResult<()>) -> KalmanResult<()> {
            match result {
                Ok(()) => {
                    core::mem::swap(&mut self.estimate, &mut self.spare);
                    Ok(())
                }
                Err(e) => {
                    log::debug!("linear Kalman filter {step} rejected: {e}");
                    Err(e)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    //! The core is tested directly: values in, values out.
    use super::*;

    fn model_1d(q: f64, r: f64) -> Model<f64> {
        Model::new(1, 1, &[1.0], &[q], &[1.0], &[r]).unwrap()
    }

    fn step_update(model: &Model<f64>, prior: &Estimate<f64>, z: &[f64]) -> KalmanResult<Estimate<f64>> {
        let mut ws = Workspace::new(model.state_dim(), model.measurement_dim());
        let mut out = prior.clone();
        update(model, prior, z, &mut ws, &mut out).map(|()| out)
    }

    #[test]
    fn predict_is_f_x_and_f_p_ft_plus_q() {
        let model = Model::new(2, 1, &[1.0, 0.1, 0.0, 1.0], &[0.01, 0.0, 0.0, 0.02], &[1.0, 0.0], &[0.1]).unwrap();
        let prior = Estimate::new(2, &[1.0, 2.0], &[2.0, 0.5, 0.5, 1.0]).unwrap();
        let mut out = prior.clone();
        predict(&model, &prior, &mut Workspace::new(2, 1), &mut out).unwrap();
        assert!(out.x().iter().zip([1.2, 2.0]).all(|(a, b)| (a - b).abs() < 1e-12));
        assert!(out.p().iter().zip([2.12, 0.6, 0.6, 1.02]).all(|(a, b)| (a - b).abs() < 1e-12));
    }

    #[test]
    fn scalar_update_matches_closed_form_and_leaves_prior_alone() {
        let prior = Estimate::new(1, &[0.0], &[1.0]).unwrap();
        let post = step_update(&model_1d(0.0, 0.25), &prior, &[2.0]).unwrap();
        assert!((post.x()[0] - 1.6).abs() < 1e-12);
        assert!((post.p()[0] - 0.2).abs() < 1e-12);
        assert_eq!(prior, Estimate::new(1, &[0.0], &[1.0]).unwrap());
    }

    #[test]
    fn joseph_form_keeps_covariance_psd_where_the_simple_form_does_not() {
        // K = P/(P+R) with P = 1e8, R = 1e-8: the simple form (1−K)P loses all digits;
        // Joseph's (1−K)²P + K²R stays ≈ R.
        let prior = Estimate::new(1, &[0.0], &[1e8]).unwrap();
        let post = step_update(&model_1d(0.0, 1e-8), &prior, &[1.0]).unwrap();
        let p = post.p()[0];
        assert!(p > 0.0 && (p - 1e-8).abs() < 1e-12, "P = {p:e}");
    }

    #[test]
    fn singular_innovation_covariance_is_an_error_value() {
        let model = Model::new(2, 2, &[1.0, 0.0, 0.0, 1.0], &[0.0; 4], &[1.0, 0.0, 0.0, 1.0], &[1.0; 4]).unwrap();
        let prior = Estimate::new(2, &[0.0, 0.0], &[0.0; 4]).unwrap();
        assert!(matches!(step_update(&model, &prior, &[1.0, 2.0]), Err(KalmanError::SingularMatrix)));
    }

    #[test]
    fn bad_measurements_are_rejected() {
        let prior = Estimate::new(1, &[0.0], &[1.0]).unwrap();
        let model = model_1d(0.0, 1.0);
        assert!(matches!(step_update(&model, &prior, &[]), Err(KalmanError::DimensionMismatch { .. })));
        assert!(matches!(step_update(&model, &prior, &[f64::NAN]), Err(KalmanError::InvalidInput("z"))));
        let far = Estimate::new(1, &[-1e308], &[1.0]).unwrap();
        assert!(matches!(step_update(&model, &far, &[1e308]), Err(KalmanError::FilterDivergence(_))));
    }

    #[test]
    fn construction_validates_every_input() {
        let ok = |q: &[f64], r: &[f64]| Model::new(1, 1, &[1.0], q, &[1.0], r);
        assert!(ok(&[0.0], &[1e-300]).is_ok());
        assert!(matches!(ok(&[-1.0], &[1.0]), Err(KalmanError::InvalidNoiseCovariance)));
        assert!(matches!(ok(&[1.0], &[-1.0]), Err(KalmanError::InvalidNoiseCovariance)));
        assert!(matches!(ok(&[f64::NAN], &[1.0]), Err(KalmanError::InvalidInput("q"))));
        assert!(matches!(Estimate::new(1, &[0.0], &[-1.0]), Err(KalmanError::InvalidCovariance)));
        assert!(matches!(Estimate::new(2, &[0.0], &[1.0]), Err(KalmanError::DimensionMismatch { .. })));
    }

    #[test]
    fn shell_commits_only_successful_steps() {
        let mut kf = linear::LinearKalmanFilter::<f32>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[1.0]).unwrap();
        assert!(kf.update(&[f32::NAN]).is_err());
        assert_eq!((kf.state(), kf.covariance()), (&[0.0f32][..], &[1.0f32][..]));
        kf.update(&[2.0]).unwrap();
        assert_eq!((kf.state(), kf.covariance()), (&[1.0f32][..], &[0.5f32][..]));
    }
}
