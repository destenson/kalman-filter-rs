//! Conversion utilities between Kalman Filter and Information Filter forms

use crate::filter::{FilterScalar, LinearKalmanFilter as KalmanFilter};
use crate::information::{InformationFilter, InformationForm, InformationState};
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use log::{debug, info, trace, warn};
use num_traits::Zero;

/// Convert Kalman Filter to Information Filter
///
/// With the `nalgebra` feature, `T` must also implement nalgebra's `RealField`
/// (`f32` and `f64` do).
#[cfg(not(feature = "nalgebra"))]
pub fn kalman_to_information<T: KalmanScalar>(
    kf: &KalmanFilter<T>,
) -> KalmanResult<InformationFilter<T>> {
    kalman_to_information_impl(kf)
}

/// Convert Kalman Filter to Information Filter
///
/// With the `nalgebra` feature, `T` must also implement nalgebra's `RealField`
/// (`f32` and `f64` do).
#[cfg(feature = "nalgebra")]
pub fn kalman_to_information<T: KalmanScalar + crate::nalgebra::RealField>(
    kf: &KalmanFilter<T>,
) -> KalmanResult<InformationFilter<T>> {
    kalman_to_information_impl(kf)
}

fn kalman_to_information_impl<T: KalmanScalar + FilterScalar>(
    kf: &KalmanFilter<T>,
) -> KalmanResult<InformationFilter<T>> {
    let n = kf.state_dim();
    let m = kf.measurement_dim();

    debug!(
        "Converting Kalman Filter to Information Filter: state_dim={}, measurement_dim={}",
        n, m
    );

    // Convert state and covariance to information form
    let info_state = InformationState::from_state_covariance(kf.state(), kf.covariance())?;
    trace!("KF->IF conversion: information state created");

    InformationFilter::initialize(
        n,
        m,
        info_state.Y,
        info_state.y,
        kf.transition().as_ref().to_vec(),
        kf.process_noise().as_ref().to_vec(),
        kf.observation().as_ref().to_vec(),
        kf.measurement_noise().as_ref().to_vec(),
    )
}

/// Convert Information Filter to Kalman Filter
///
/// With the `nalgebra` feature, `T` must also implement nalgebra's `RealField`
/// (`f32` and `f64` do).
#[cfg(not(feature = "nalgebra"))]
pub fn information_to_kalman<T: KalmanScalar>(
    inf: &InformationFilter<T>,
) -> KalmanResult<KalmanFilter<T>> {
    information_to_kalman_impl(inf)
}

/// Convert Information Filter to Kalman Filter
///
/// With the `nalgebra` feature, `T` must also implement nalgebra's `RealField`
/// (`f32` and `f64` do).
#[cfg(feature = "nalgebra")]
pub fn information_to_kalman<T: KalmanScalar + crate::nalgebra::RealField>(
    inf: &InformationFilter<T>,
) -> KalmanResult<KalmanFilter<T>> {
    information_to_kalman_impl(inf)
}

fn information_to_kalman_impl<T: KalmanScalar + FilterScalar>(
    inf: &InformationFilter<T>,
) -> KalmanResult<KalmanFilter<T>> {
    let n = inf.state_dim;
    let m = inf.measurement_dim;

    debug!(
        "Converting Information Filter to Kalman Filter: state_dim={}, measurement_dim={}",
        n, m
    );

    // Recover state and covariance from information form
    let state = inf.state.recover_state()?;
    let covariance = inf.state.recover_covariance()?;
    trace!("IF->KF conversion: state and covariance recovered");

    KalmanFilter::try_new(n, m, &state, &covariance, &inf.F, &inf.Q, &inf.H, &inf.R)
}

/// Hybrid filter that can switch between forms based on sparsity
///
/// With the `nalgebra` feature, `T` must also implement nalgebra's `RealField`
/// (`f32` and `f64` do).
#[cfg(not(feature = "nalgebra"))]
pub struct HybridFilter<T: KalmanScalar> {
    kf: Option<KalmanFilter<T>>,
    inf: Option<InformationFilter<T>>,
    sparsity_threshold: f64,
    current_form: FilterForm,
}

/// Hybrid filter that can switch between forms based on sparsity
///
/// With the `nalgebra` feature, `T` must also implement nalgebra's `RealField`
/// (`f32` and `f64` do).
#[cfg(feature = "nalgebra")]
pub struct HybridFilter<T: KalmanScalar + crate::nalgebra::RealField> {
    kf: Option<KalmanFilter<T>>,
    inf: Option<InformationFilter<T>>,
    sparsity_threshold: f64,
    current_form: FilterForm,
}

/// Representation form for hybrid Kalman/Information filter
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FilterForm {
    /// Standard Kalman filter form using covariance matrix P
    Covariance,
    /// Information filter form using information matrix Y = P^-1
    Information,
}

#[cfg(not(feature = "nalgebra"))]
impl<T: KalmanScalar> HybridFilter<T> {
    /// Create new hybrid filter
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        state_dim: usize,
        measurement_dim: usize,
        initial_state: Vec<T>,
        initial_covariance: Vec<T>,
        F: Vec<T>,
        Q: Vec<T>,
        H: Vec<T>,
        R: Vec<T>,
        sparsity_threshold: f64,
    ) -> KalmanResult<Self> {
        hybrid_new(
            state_dim,
            measurement_dim,
            initial_state,
            initial_covariance,
            F,
            Q,
            H,
            R,
            sparsity_threshold,
        )
    }

    /// Switch to information form
    pub fn to_information_form(&mut self) -> KalmanResult<()> {
        hybrid_to_information_form(self)
    }

    /// Switch to covariance form
    pub fn to_covariance_form(&mut self) -> KalmanResult<()> {
        hybrid_to_covariance_form(self)
    }

    /// Automatically choose form based on measurement sparsity
    pub fn auto_select_form(&mut self, H: &[T]) -> KalmanResult<()> {
        hybrid_auto_select_form(self, H)
    }

    /// Predict step
    pub fn predict(&mut self) -> KalmanResult<()> {
        hybrid_predict(self)
    }

    /// Update step
    pub fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        hybrid_update(self, measurement)
    }

    /// Get current state estimate
    pub fn get_state(&self) -> KalmanResult<Vec<T>> {
        hybrid_get_state(self)
    }

    /// Get current covariance
    pub fn get_covariance(&self) -> KalmanResult<Vec<T>> {
        hybrid_get_covariance(self)
    }
}

#[cfg(feature = "nalgebra")]
impl<T: KalmanScalar + crate::nalgebra::RealField> HybridFilter<T> {
    /// Create new hybrid filter
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        state_dim: usize,
        measurement_dim: usize,
        initial_state: Vec<T>,
        initial_covariance: Vec<T>,
        F: Vec<T>,
        Q: Vec<T>,
        H: Vec<T>,
        R: Vec<T>,
        sparsity_threshold: f64,
    ) -> KalmanResult<Self> {
        hybrid_new(
            state_dim,
            measurement_dim,
            initial_state,
            initial_covariance,
            F,
            Q,
            H,
            R,
            sparsity_threshold,
        )
    }

    /// Switch to information form
    pub fn to_information_form(&mut self) -> KalmanResult<()> {
        hybrid_to_information_form(self)
    }

    /// Switch to covariance form
    pub fn to_covariance_form(&mut self) -> KalmanResult<()> {
        hybrid_to_covariance_form(self)
    }

    /// Automatically choose form based on measurement sparsity
    pub fn auto_select_form(&mut self, H: &[T]) -> KalmanResult<()> {
        hybrid_auto_select_form(self, H)
    }

    /// Predict step
    pub fn predict(&mut self) -> KalmanResult<()> {
        hybrid_predict(self)
    }

    /// Update step
    pub fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        hybrid_update(self, measurement)
    }

    /// Get current state estimate
    pub fn get_state(&self) -> KalmanResult<Vec<T>> {
        hybrid_get_state(self)
    }

    /// Get current covariance
    pub fn get_covariance(&self) -> KalmanResult<Vec<T>> {
        hybrid_get_covariance(self)
    }
}

#[allow(clippy::too_many_arguments)]
fn hybrid_new<T: KalmanScalar + FilterScalar>(
    state_dim: usize,
    measurement_dim: usize,
    initial_state: Vec<T>,
    initial_covariance: Vec<T>,
    F: Vec<T>,
    Q: Vec<T>,
    H: Vec<T>,
    R: Vec<T>,
    sparsity_threshold: f64,
) -> KalmanResult<HybridFilter<T>> {
    debug!(
        "Creating HybridFilter: state_dim={}, measurement_dim={}, sparsity_threshold={:.2}",
        state_dim, measurement_dim, sparsity_threshold
    );
    let kf = KalmanFilter::try_new(
        state_dim,
        measurement_dim,
        &initial_state,
        &initial_covariance,
        &F,
        &Q,
        &H,
        &R,
    )?;

    Ok(HybridFilter {
        kf: Some(kf),
        inf: None,
        sparsity_threshold,
        current_form: FilterForm::Covariance,
    })
}

fn hybrid_to_information_form<T: KalmanScalar + FilterScalar>(
    hybrid: &mut HybridFilter<T>,
) -> KalmanResult<()> {
    if hybrid.current_form == FilterForm::Information {
        trace!("HybridFilter: already in information form");
        return Ok(());
    }

    debug!("HybridFilter: switching from covariance to information form");

    if let Some(kf) = &hybrid.kf {
        hybrid.inf = Some(kalman_to_information(kf)?);
        hybrid.kf = None;
        hybrid.current_form = FilterForm::Information;
        debug!("HybridFilter: successfully switched to information form");
    }

    Ok(())
}

fn hybrid_to_covariance_form<T: KalmanScalar + FilterScalar>(
    hybrid: &mut HybridFilter<T>,
) -> KalmanResult<()> {
    if hybrid.current_form == FilterForm::Covariance {
        trace!("HybridFilter: already in covariance form");
        return Ok(());
    }

    debug!("HybridFilter: switching from information to covariance form");

    if let Some(inf) = &hybrid.inf {
        hybrid.kf = Some(information_to_kalman(inf)?);
        hybrid.inf = None;
        hybrid.current_form = FilterForm::Covariance;
        debug!("HybridFilter: successfully switched to covariance form");
    }

    Ok(())
}

fn hybrid_auto_select_form<T: KalmanScalar + FilterScalar>(
    hybrid: &mut HybridFilter<T>,
    H: &[T],
) -> KalmanResult<()> {
    let sparsity = hybrid_sparsity(H);
    debug!(
        "HybridFilter: auto-selecting form based on sparsity={:.2}% (threshold={:.2}%)",
        sparsity * 100.0,
        hybrid.sparsity_threshold * 100.0
    );

    if sparsity > hybrid.sparsity_threshold {
        debug!("HybridFilter: selecting information form due to high sparsity");
        hybrid_to_information_form(hybrid)?;
    } else {
        debug!("HybridFilter: selecting covariance form due to low sparsity");
        hybrid_to_covariance_form(hybrid)?;
    }

    Ok(())
}

fn hybrid_sparsity<T: KalmanScalar>(matrix: &[T]) -> f64 {
    let total = matrix.len() as f64;
    let zeros = matrix
        .iter()
        .filter(|&&x| <T as num_traits::Float>::abs(x) < <T as KalmanScalar>::epsilon())
        .count() as f64;
    let sparsity = zeros / total;
    trace!(
        "Matrix sparsity: {:.1}% ({} zeros out of {} elements)",
        sparsity * 100.0,
        zeros as usize,
        matrix.len()
    );
    sparsity
}

fn hybrid_predict<T: KalmanScalar + FilterScalar>(hybrid: &mut HybridFilter<T>) -> KalmanResult<()> {
    debug!("HybridFilter predict: using {:?} form", hybrid.current_form);
    match hybrid.current_form {
        FilterForm::Covariance => {
            if let Some(kf) = &mut hybrid.kf {
                kf.predict()?;
            }
        }
        FilterForm::Information => {
            if let Some(inf) = &mut hybrid.inf {
                inf.predict()?;
            }
        }
    }
    Ok(())
}

fn hybrid_update<T: KalmanScalar + FilterScalar>(
    hybrid: &mut HybridFilter<T>,
    measurement: &[T],
) -> KalmanResult<()> {
    debug!("HybridFilter update: using {:?} form", hybrid.current_form);
    match hybrid.current_form {
        FilterForm::Covariance => {
            if let Some(kf) = &mut hybrid.kf {
                kf.update(measurement)?;
            }
        }
        FilterForm::Information => {
            if let Some(inf) = &mut hybrid.inf {
                inf.update(measurement)?;
            }
        }
    }
    Ok(())
}

fn hybrid_get_state<T: KalmanScalar + FilterScalar>(hybrid: &HybridFilter<T>) -> KalmanResult<Vec<T>> {
    match hybrid.current_form {
        FilterForm::Covariance => {
            if let Some(kf) = &hybrid.kf {
                Ok(kf.state().to_vec())
            } else {
                warn!("HybridFilter: no active filter");
                Err(KalmanError::FilterDivergence("No active filter".to_string()))
            }
        }
        FilterForm::Information => {
            if let Some(inf) = &hybrid.inf {
                inf.get_state()
            } else {
                warn!("HybridFilter: no active filter");
                Err(KalmanError::FilterDivergence("No active filter".to_string()))
            }
        }
    }
}

fn hybrid_get_covariance<T: KalmanScalar + FilterScalar>(
    hybrid: &HybridFilter<T>,
) -> KalmanResult<Vec<T>> {
    match hybrid.current_form {
        FilterForm::Covariance => {
            if let Some(kf) = &hybrid.kf {
                Ok(kf.covariance().to_vec())
            } else {
                warn!("HybridFilter: no active filter");
                Err(KalmanError::FilterDivergence("No active filter".to_string()))
            }
        }
        FilterForm::Information => {
            if let Some(inf) = &hybrid.inf {
                inf.get_covariance()
            } else {
                warn!("HybridFilter: no active filter");
                Err(KalmanError::FilterDivergence("No active filter".to_string()))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_information_equivalence() {
        // Create identical filters in both forms
        let mut kf = KalmanFilter::<f64>::try_new(
            2,
            1,
            &[0.0, 0.0],
            &[1.0, 0.0, 0.0, 1.0],
            &[1.0, 0.1, 0.0, 1.0],
            &[0.01, 0.0, 0.0, 0.01],
            &[1.0, 0.0],
            &[0.1],
        )
        .unwrap();

        let mut inf = kalman_to_information(&kf).unwrap();

        // Perform same operations
        kf.predict();
        kf.update(&[1.0]).unwrap();

        inf.predict().unwrap();
        inf.update(&[1.0]).unwrap();

        // Compare states
        let kf_state = kf.state();
        let inf_state = inf.get_state().unwrap();

        for i in 0..2 {
            assert!(
                (kf_state[i] - inf_state[i]).abs() < 1e-10,
                "State mismatch at index {}: KF={}, IF={}",
                i,
                kf_state[i],
                inf_state[i]
            );
        }

        // Compare covariances
        let kf_cov = kf.covariance();
        let inf_cov = inf.get_covariance().unwrap();

        for i in 0..4 {
            assert!(
                (kf_cov[i] - inf_cov[i]).abs() < 1e-10,
                "Covariance mismatch at index {}: KF={}, IF={}",
                i,
                kf_cov[i],
                inf_cov[i]
            );
        }
    }

    #[test]
    fn test_hybrid_filter() {
        let mut hybrid = HybridFilter::new(
            2,
            1,
            vec![0.0, 0.0],
            vec![1.0, 0.0, 0.0, 1.0],
            vec![1.0, 0.1, 0.0, 1.0],
            vec![0.01, 0.0, 0.0, 0.01],
            vec![1.0, 0.0],
            vec![0.1],
            0.5, // Switch to IF when >50% sparse
        )
        .unwrap();

        // Start in covariance form
        assert_eq!(hybrid.current_form, FilterForm::Covariance);

        // Switch to information form
        hybrid.to_information_form().unwrap();
        assert_eq!(hybrid.current_form, FilterForm::Information);

        // Perform operations
        hybrid.predict().unwrap();
        hybrid.update(&[1.0]).unwrap();

        let state = hybrid.get_state().unwrap();
        assert!(state[0] > 0.0);
    }
}
