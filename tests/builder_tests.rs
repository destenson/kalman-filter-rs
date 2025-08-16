//! Tests for all filter builders

use kalman_filter::*;

#[derive(Clone)]
struct TestSystem;

impl NonlinearSystem<f64> for TestSystem {
    fn state_dim(&self) -> usize { 2 }
    fn measurement_dim(&self) -> usize { 1 }
    
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        vec![state[0] + state[1] * dt, state[1]]
    }
    
    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        vec![state[0]]
    }
    
    fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
        vec![1.0, 0.0, 0.0, 1.0]
    }
    
    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        vec![1.0, 0.0]
    }
}

#[test]
fn test_extended_kalman_filter_builder() {
    let system = TestSystem;
    
    let ekf = ExtendedKalmanFilterBuilder::new(system)
        .initial_state(vec![0.0, 1.0])
        .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .build()
        .unwrap();
    
    assert_eq!(ekf.state().len(), 2);
    assert_eq!(ekf.covariance().len(), 4);
}

#[test]
fn test_unscented_kalman_filter_builder() {
    let system = TestSystem;
    
    let ukf = UnscentedKalmanFilterBuilder::new(system)
        .initial_state(vec![0.0, 1.0])
        .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .alpha(0.001)
        .beta(2.0)
        .kappa(0.0)
        .build()
        .unwrap();
    
    assert_eq!(ukf.state().len(), 2);
    assert_eq!(ukf.covariance().len(), 4);
}

#[test]
fn test_cubature_kalman_filter_builder() {
    let system = TestSystem;
    
    let ckf = CubatureKalmanFilterBuilder::new(system)
        .initial_state(vec![0.0, 1.0])
        .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .build()
        .unwrap();
    
    assert_eq!(ckf.state().len(), 2);
    assert_eq!(ckf.covariance().len(), 4);
}

#[test]
fn test_ensemble_kalman_filter_builder() {
    let system = TestSystem;
    
    let enkf = EnsembleKalmanFilterBuilder::new(system)
        .initial_mean(vec![0.0, 1.0])
        .initial_spread(vec![1.0, 1.0])
        .ensemble_size(50)
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .inflation_factor(1.05)
        .build()
        .unwrap();
    
    assert_eq!(enkf.mean().len(), 2);
    assert_eq!(enkf.ensemble_size, 50);
}

#[test]
fn test_information_filter_builder() {
    let if_filter = InformationFilterBuilder::new(2, 1)
        .initial_information_matrix(vec![1.0, 0.0, 0.0, 1.0])
        .initial_information_vector(vec![0.0, 0.0])
        .state_transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![0.1])
        .build()
        .unwrap();
    
    let state = if_filter.get_state().unwrap();
    assert_eq!(state.len(), 2);
}

#[test]
fn test_information_filter_builder_from_state_covariance() {
    let initial_state = vec![1.0, 2.0];
    let initial_covariance = vec![2.0, 0.5, 0.5, 1.0];
    
    let if_filter = InformationFilterBuilder::from_state_covariance(
        initial_state.clone(),
        initial_covariance,
        1,
    )
    .unwrap()
    .state_transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
    .process_noise(vec![0.01, 0.0, 0.0, 0.01])
    .observation_matrix(vec![1.0, 0.0])
    .measurement_noise(vec![0.1])
    .build()
    .unwrap();
    
    let recovered_state = if_filter.get_state().unwrap();
    let diff0: f64 = recovered_state[0] - initial_state[0];
    let diff1: f64 = recovered_state[1] - initial_state[1];
    assert!(diff0.abs() < 1e-10f64);
    assert!(diff1.abs() < 1e-10f64);
}

#[test]
fn test_particle_filter_builder() {
    let pf = ParticleFilterBuilder::new(2, 100)
        .initial_mean(vec![0.0, 1.0])
        .initial_std(vec![1.0, 1.0])
        .process_noise_std(vec![0.1, 0.1])
        .measurement_noise_std(vec![0.5])
        .dt(0.1)
        .ess_threshold(50.0)
        .build()
        .unwrap();
    
    assert_eq!(pf.state_dim, 2);
    assert_eq!(pf.num_particles, 100);
    assert_eq!(pf.ess_threshold, 50.0);
}

#[test]
fn test_builder_incomplete() {
    let system = TestSystem;
    
    // Test missing required parameter
    let result = ExtendedKalmanFilterBuilder::new(system)
        .initial_state(vec![0.0, 1.0])
        // Missing other required fields
        .build();
    
    assert!(result.is_err());
    if let Err(KalmanError::BuilderIncomplete(field)) = result {
        assert_eq!(field, "initial_covariance");
    } else {
        panic!("Expected BuilderIncomplete error");
    }
}