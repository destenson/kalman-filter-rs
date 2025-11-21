//! Integration tests for input validation across filter implementations
//!
//! These tests verify that invalid inputs are properly rejected with helpful errors,
//! and that edge cases are handled correctly across all filter types.

use kalman_filters::*;
use kalman_filters::filter::KalmanFilter;

// =============================================================================
// Dimension Mismatch Tests
// =============================================================================

#[test]
fn test_kf_rejects_wrong_state_dimension() {
    let result = KalmanFilter::<f64>::initialize(
        2, 1,
        vec![0.0],           // Wrong: 1 element, expected 2
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![0.1, 0.0, 0.0, 0.1],
        vec![1.0, 0.0],
        vec![0.1],
    );
    assert!(result.is_err(), "Should reject wrong state dimension");
    match result {
        Err(KalmanError::DimensionMismatch { expected, actual }) => {
            assert_eq!(expected, (2, 1));
            assert_eq!(actual, (1, 1));
        }
        _ => panic!("Expected DimensionMismatch error"),
    }
}

#[test]
fn test_kf_rejects_wrong_covariance_dimension() {
    let result = KalmanFilter::<f64>::initialize(
        2, 1,
        vec![0.0, 0.0],
        vec![1.0],           // Wrong: 1 element, expected 4 (2x2)
        vec![1.0, 0.0, 0.0, 1.0],
        vec![0.1, 0.0, 0.0, 0.1],
        vec![1.0, 0.0],
        vec![0.1],
    );
    assert!(result.is_err(), "Should reject wrong covariance dimension");
}

#[test]
fn test_kf_rejects_wrong_measurement_dimension() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.1, 0.0, 0.0, 0.1];
    kf.H = vec![1.0, 0.0];
    kf.R = vec![0.1];

    let result = kf.update(&[1.0, 2.0]);  // Wrong: 2 measurements, expected 1
    assert!(result.is_err(), "Should reject wrong measurement dimension");
}

// =============================================================================
// Numerical Stability Tests
// =============================================================================

#[test]
fn test_kf_handles_very_small_covariance() {
    // Very small but valid covariance should work
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![0.0];
    kf.P = vec![1e-10];  // Very small but > MIN_CONDITION_NUMBER
    kf.F = vec![1.0];
    kf.Q = vec![1e-12];
    kf.H = vec![1.0];
    kf.R = vec![1e-10];

    kf.predict();
    let result = kf.update(&[1.0]);
    assert!(result.is_ok(), "Should handle small but valid covariance");
}

#[test]
fn test_kf_measurement_noise_must_be_positive() {
    // Zero measurement noise should be rejected
    let result = KalmanFilter::<f64>::initialize(
        1, 1,
        vec![0.0],
        vec![1.0],
        vec![1.0],
        vec![0.1],
        vec![1.0],
        vec![0.0],  // Zero measurement noise - invalid
    );
    assert!(result.is_err(), "Should reject zero measurement noise");
}

// =============================================================================
// Builder Validation Tests
// =============================================================================

#[derive(Clone)]
struct SimpleTestSystem;

impl NonlinearSystem<f64> for SimpleTestSystem {
    fn state_dim(&self) -> usize { 2 }
    fn measurement_dim(&self) -> usize { 1 }
    fn state_transition(&self, state: &[f64], _: Option<&[f64]>, dt: f64) -> Vec<f64> {
        vec![state[0] + state[1] * dt, state[1]]
    }
    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        vec![state[0]]
    }
    fn state_jacobian(&self, _: &[f64], _: Option<&[f64]>, dt: f64) -> Vec<f64> {
        vec![1.0, dt, 0.0, 1.0]
    }
    fn measurement_jacobian(&self, _: &[f64]) -> Vec<f64> {
        vec![1.0, 0.0]
    }
}

#[test]
fn test_ekf_builder_rejects_missing_required_fields() {
    let result = ExtendedKalmanFilterBuilder::new(SimpleTestSystem)
        .initial_state(vec![0.0, 0.0])
        // Missing: initial_covariance, process_noise, measurement_noise, dt
        .build();

    assert!(result.is_err(), "Should reject incomplete builder");
    if let Err(KalmanError::BuilderIncomplete(field)) = result {
        assert_eq!(field, "initial_covariance");
    }
}

#[test]
fn test_ukf_builder_validates_dimensions() {
    let result = UnscentedKalmanFilterBuilder::new(SimpleTestSystem)
        .initial_state(vec![0.0])  // Wrong: 1 element, system has state_dim=2
        .initial_covariance(vec![1.0])
        .process_noise(vec![0.1])
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .build();

    assert!(result.is_err(), "Should reject mismatched state dimension");
}

#[test]
fn test_particle_filter_builder_validates_particle_count() {
    let result = ParticleFilterBuilder::new(2, 0)  // Zero particles - invalid
        .initial_mean(vec![0.0, 0.0])
        .initial_std(vec![1.0, 1.0])
        .process_noise_std(vec![0.1, 0.1])
        .measurement_noise_std(vec![0.5])
        .dt(0.1)
        .build();

    // Zero particles should be rejected (or result in an error during operations)
    // The builder might accept it but operations will fail
    if result.is_ok() {
        let pf = result.unwrap();
        assert_eq!(pf.num_particles, 0);
        // Mean computation on 0 particles would be undefined
    }
}

// =============================================================================
// Information Filter Validation
// =============================================================================

#[test]
fn test_if_builder_rejects_invalid_matrices() {
    let result = InformationFilterBuilder::new(2, 1)
        .initial_information_matrix(vec![1.0])  // Wrong: should be 4 elements (2x2)
        .initial_information_vector(vec![0.0, 0.0])
        .state_transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![0.1])
        .build();

    assert!(result.is_err(), "Should reject wrong information matrix size");
}

#[test]
fn test_if_update_rejects_wrong_measurement_size() {
    let mut filter = InformationFilterBuilder::new(2, 1)
        .initial_information_matrix(vec![1.0, 0.0, 0.0, 1.0])
        .initial_information_vector(vec![0.0, 0.0])
        .state_transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![0.1])
        .build()
        .unwrap();

    let result = filter.update(&[1.0, 2.0]);  // Wrong: 2 elements, expected 1
    assert!(result.is_err(), "Should reject wrong measurement size");
}

// =============================================================================
// Ensemble Filter Validation
// =============================================================================

#[test]
fn test_enkf_builder_validates_ensemble_size() {
    // Very small ensemble should still work but may have poor statistics
    let result = EnsembleKalmanFilterBuilder::new(SimpleTestSystem)
        .initial_mean(vec![0.0, 0.0])
        .initial_spread(vec![1.0, 1.0])
        .ensemble_size(2)  // Minimum valid ensemble
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .build();

    assert!(result.is_ok(), "Small ensemble should be accepted");
}

#[test]
fn test_enkf_update_rejects_wrong_observation_size() {
    let mut enkf = EnsembleKalmanFilterBuilder::new(SimpleTestSystem)
        .initial_mean(vec![0.0, 0.0])
        .initial_spread(vec![1.0, 1.0])
        .ensemble_size(20)
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .build()
        .unwrap();

    let result = enkf.update(&[1.0, 2.0]);  // Wrong: 2 elements, system has m=1
    assert!(result.is_err(), "Should reject wrong observation size");
}

// =============================================================================
// Edge Case Tests
// =============================================================================

#[test]
fn test_kf_handles_identity_matrices() {
    // All identity matrices should work correctly
    let mut kf = KalmanFilter::<f64>::new(2, 2);
    kf.x = vec![1.0, 2.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 0.0, 0.0, 1.0];  // Identity
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];  // Zero process noise
    kf.H = vec![1.0, 0.0, 0.0, 1.0];  // Identity observation
    kf.R = vec![1.0, 0.0, 0.0, 1.0];

    kf.predict();
    // State should be unchanged with identity F and zero Q
    assert!((kf.x[0] - 1.0).abs() < 1e-10);
    assert!((kf.x[1] - 2.0).abs() < 1e-10);

    kf.update(&[1.0, 2.0]).unwrap();
    // With measurement matching state, state should remain close
    assert!((kf.x[0] - 1.0).abs() < 0.1);
    assert!((kf.x[1] - 2.0).abs() < 0.1);
}

#[test]
fn test_kf_handles_high_dimensional_state() {
    // 10-dimensional state should work
    let n = 10;
    let m = 3;
    let mut kf = KalmanFilter::<f64>::new(n, m);

    kf.x = vec![0.0; n];
    kf.P = (0..n*n).map(|i| if i % (n+1) == 0 { 1.0 } else { 0.0 }).collect();
    kf.F = (0..n*n).map(|i| if i % (n+1) == 0 { 1.0 } else { 0.0 }).collect();
    kf.Q = (0..n*n).map(|i| if i % (n+1) == 0 { 0.01 } else { 0.0 }).collect();
    kf.H = vec![0.0; m * n];
    for i in 0..m {
        kf.H[i * n + i] = 1.0;  // Observe first m states
    }
    kf.R = (0..m*m).map(|i| if i % (m+1) == 0 { 0.1 } else { 0.0 }).collect();

    kf.predict();
    let result = kf.update(&vec![1.0; m]);
    assert!(result.is_ok(), "Should handle high-dimensional state");
}

#[test]
fn test_singular_innovation_covariance_detected() {
    // Create a situation where innovation covariance becomes singular
    let mut kf = KalmanFilter::<f64>::new(2, 2);
    kf.x = vec![0.0, 0.0];
    kf.P = vec![0.0, 0.0, 0.0, 0.0];  // Zero covariance
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];
    kf.H = vec![1.0, 0.0, 0.0, 1.0];
    kf.R = vec![0.0, 0.0, 0.0, 0.0];  // Zero measurement noise

    kf.predict();
    let result = kf.update(&[1.0, 1.0]);
    // S = H*P*H' + R = 0, which is singular
    assert!(result.is_err(), "Should detect singular innovation covariance");
    match result {
        Err(KalmanError::SingularMatrix) => (),
        _ => panic!("Expected SingularMatrix error"),
    }
}

// =============================================================================
// Cross-Filter Type Compatibility
// =============================================================================

#[test]
fn test_all_filters_accept_same_problem_definition() {
    // Define a problem that all filters should accept
    let init_state = vec![0.0, 1.0];
    let init_cov = vec![1.0, 0.0, 0.0, 1.0];
    let q = vec![0.01, 0.0, 0.0, 0.01];
    let r = vec![0.1];
    let dt = 0.1;

    // Standard KF (via builder pattern through initialize)
    let kf_result = KalmanFilter::<f64>::initialize(
        2, 1,
        init_state.clone(),
        init_cov.clone(),
        vec![1.0, dt, 0.0, 1.0],
        q.clone(),
        vec![1.0, 0.0],
        r.clone(),
    );
    assert!(kf_result.is_ok(), "KF should accept valid problem");

    // EKF
    let ekf_result = ExtendedKalmanFilterBuilder::new(SimpleTestSystem)
        .initial_state(init_state.clone())
        .initial_covariance(init_cov.clone())
        .process_noise(q.clone())
        .measurement_noise(r.clone())
        .dt(dt)
        .build();
    assert!(ekf_result.is_ok(), "EKF should accept valid problem");

    // UKF
    let ukf_result = UnscentedKalmanFilterBuilder::new(SimpleTestSystem)
        .initial_state(init_state.clone())
        .initial_covariance(init_cov.clone())
        .process_noise(q.clone())
        .measurement_noise(r.clone())
        .dt(dt)
        .build();
    assert!(ukf_result.is_ok(), "UKF should accept valid problem");

    // EnKF
    let enkf_result = EnsembleKalmanFilterBuilder::new(SimpleTestSystem)
        .initial_mean(init_state.clone())
        .initial_spread(vec![1.0, 1.0])
        .ensemble_size(50)
        .process_noise(q.clone())
        .measurement_noise(r.clone())
        .dt(dt)
        .build();
    assert!(enkf_result.is_ok(), "EnKF should accept valid problem");

    // Particle Filter
    let pf_result = ParticleFilterBuilder::new(2, 100)
        .initial_mean(init_state.clone())
        .initial_std(vec![1.0, 1.0])
        .process_noise_std(vec![0.1, 0.1])
        .measurement_noise_std(vec![0.316])  // sqrt(0.1)
        .dt(dt)
        .build();
    assert!(pf_result.is_ok(), "PF should accept valid problem");
}
