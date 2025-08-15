//! Conversion utilities between Kalman Filter and Information Filter forms

use crate::filter::{KalmanFilter};
use crate::information::{InformationFilter, InformationState, InformationForm};
use crate::types::{KalmanError, KalmanResult, KalmanScalar};
use num_traits::Zero;

/// Convert Kalman Filter to Information Filter
pub fn kalman_to_information<T: KalmanScalar>(kf: &KalmanFilter<T>) -> KalmanResult<InformationFilter<T>> {
    let n = kf.state_dim;
    let m = kf.measurement_dim;
    
    // Convert state and covariance to information form
    let info_state = InformationState::from_state_covariance(&kf.x, &kf.P)?;
    
    InformationFilter::new(
        n,
        m,
        info_state.Y,
        info_state.y,
        kf.F.clone(),
        kf.Q.clone(),
        kf.H.clone(),
        kf.R.clone(),
    )
}

/// Convert Information Filter to Kalman Filter
pub fn information_to_kalman<T: KalmanScalar>(inf: &InformationFilter<T>) -> KalmanResult<KalmanFilter<T>> {
    let n = inf.state_dim;
    let m = inf.measurement_dim;
    
    // Recover state and covariance from information form
    let state = inf.state.recover_state()?;
    let covariance = inf.state.recover_covariance()?;
    
    KalmanFilter::initialize(
        n,
        m,
        state,
        covariance,
        inf.F.clone(),
        inf.Q.clone(),
        inf.H.clone(),
        inf.R.clone(),
    )
}

/// Hybrid filter that can switch between forms based on sparsity
pub struct HybridFilter<T: KalmanScalar> {
    kf: Option<KalmanFilter<T>>,
    inf: Option<InformationFilter<T>>,
    sparsity_threshold: f64,
    current_form: FilterForm,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FilterForm {
    Covariance,
    Information,
}

impl<T: KalmanScalar> HybridFilter<T> {
    /// Create new hybrid filter
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
        let kf = KalmanFilter::initialize(
            state_dim,
            measurement_dim,
            initial_state,
            initial_covariance,
            F,
            Q,
            H,
            R,
        )?;
        
        Ok(Self {
            kf: Some(kf),
            inf: None,
            sparsity_threshold,
            current_form: FilterForm::Covariance,
        })
    }
    
    /// Switch to information form
    pub fn to_information_form(&mut self) -> KalmanResult<()> {
        if self.current_form == FilterForm::Information {
            return Ok(());
        }
        
        if let Some(kf) = &self.kf {
            self.inf = Some(kalman_to_information(kf)?);
            self.kf = None;
            self.current_form = FilterForm::Information;
        }
        
        Ok(())
    }
    
    /// Switch to covariance form
    pub fn to_covariance_form(&mut self) -> KalmanResult<()> {
        if self.current_form == FilterForm::Covariance {
            return Ok(());
        }
        
        if let Some(inf) = &self.inf {
            self.kf = Some(information_to_kalman(inf)?);
            self.inf = None;
            self.current_form = FilterForm::Covariance;
        }
        
        Ok(())
    }
    
    /// Automatically choose form based on measurement sparsity
    pub fn auto_select_form(&mut self, H: &[T]) -> KalmanResult<()> {
        let sparsity = Self::compute_sparsity(H);
        
        if sparsity > self.sparsity_threshold {
            self.to_information_form()?;
        } else {
            self.to_covariance_form()?;
        }
        
        Ok(())
    }
    
    /// Compute sparsity of a matrix (fraction of zeros)
    fn compute_sparsity(matrix: &[T]) -> f64 {
        let total = matrix.len() as f64;
        let zeros = matrix.iter()
            .filter(|&&x| x.abs() < <T as KalmanScalar>::epsilon())
            .count() as f64;
        zeros / total
    }
    
    /// Predict step
    pub fn predict(&mut self) -> KalmanResult<()> {
        match self.current_form {
            FilterForm::Covariance => {
                if let Some(kf) = &mut self.kf {
                    kf.predict();
                }
            }
            FilterForm::Information => {
                if let Some(inf) = &mut self.inf {
                    inf.predict()?;
                }
            }
        }
        Ok(())
    }
    
    /// Update step
    pub fn update(&mut self, measurement: &[T]) -> KalmanResult<()> {
        match self.current_form {
            FilterForm::Covariance => {
                if let Some(kf) = &mut self.kf {
                    kf.update(measurement)?;
                }
            }
            FilterForm::Information => {
                if let Some(inf) = &mut self.inf {
                    inf.update(measurement)?;
                }
            }
        }
        Ok(())
    }
    
    /// Get current state estimate
    pub fn get_state(&self) -> KalmanResult<Vec<T>> {
        match self.current_form {
            FilterForm::Covariance => {
                if let Some(kf) = &self.kf {
                    Ok(kf.x.clone())
                } else {
                    Err(KalmanError::FilterDivergence("No active filter".to_string()))
                }
            }
            FilterForm::Information => {
                if let Some(inf) = &self.inf {
                    inf.get_state()
                } else {
                    Err(KalmanError::FilterDivergence("No active filter".to_string()))
                }
            }
        }
    }
    
    /// Get current covariance
    pub fn get_covariance(&self) -> KalmanResult<Vec<T>> {
        match self.current_form {
            FilterForm::Covariance => {
                if let Some(kf) = &self.kf {
                    Ok(kf.P.clone())
                } else {
                    Err(KalmanError::FilterDivergence("No active filter".to_string()))
                }
            }
            FilterForm::Information => {
                if let Some(inf) = &self.inf {
                    inf.get_covariance()
                } else {
                    Err(KalmanError::FilterDivergence("No active filter".to_string()))
                }
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
        let mut kf = KalmanFilter::<f64>::initialize(
            2, 1,
            vec![0.0, 0.0],
            vec![1.0, 0.0, 0.0, 1.0],
            vec![1.0, 0.1, 0.0, 1.0],
            vec![0.01, 0.0, 0.0, 0.01],
            vec![1.0, 0.0],
            vec![0.1],
        ).unwrap();
        
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
            assert!((kf_state[i] - inf_state[i]).abs() < 1e-10,
                    "State mismatch at index {}: KF={}, IF={}", 
                    i, kf_state[i], inf_state[i]);
        }
        
        // Compare covariances
        let kf_cov = kf.covariance();
        let inf_cov = inf.get_covariance().unwrap();
        
        for i in 0..4 {
            assert!((kf_cov[i] - inf_cov[i]).abs() < 1e-10,
                    "Covariance mismatch at index {}: KF={}, IF={}", 
                    i, kf_cov[i], inf_cov[i]);
        }
    }
    
    #[test]
    fn test_hybrid_filter() {
        let mut hybrid = HybridFilter::new(
            2, 1,
            vec![0.0, 0.0],
            vec![1.0, 0.0, 0.0, 1.0],
            vec![1.0, 0.1, 0.0, 1.0],
            vec![0.01, 0.0, 0.0, 0.01],
            vec![1.0, 0.0],
            vec![0.1],
            0.5,  // Switch to IF when >50% sparse
        ).unwrap();
        
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
