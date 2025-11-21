//! Tests for all filter builders

use kalman_filters::*;

#[derive(Clone)]
struct TestSystem;

impl NonlinearSystem<f64> for TestSystem {
    fn state_dim(&self) -> usize {
        2
    }
    fn measurement_dim(&self) -> usize {
        1
    }

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

    // Verify correct dimensions
    assert_eq!(ekf.state().len(), 2);
    assert_eq!(ekf.covariance().len(), 4);

    // Verify actual state values match what we set
    assert_eq!(ekf.state()[0], 0.0, "Initial state[0] should be 0.0");
    assert_eq!(ekf.state()[1], 1.0, "Initial state[1] should be 1.0");

    // Verify covariance was set correctly
    assert_eq!(ekf.covariance()[0], 1.0, "Initial P[0,0] should be 1.0");
    assert_eq!(ekf.covariance()[1], 0.0, "Initial P[0,1] should be 0.0");
    assert_eq!(ekf.covariance()[2], 0.0, "Initial P[1,0] should be 0.0");
    assert_eq!(ekf.covariance()[3], 1.0, "Initial P[1,1] should be 1.0");
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

    // Verify dimensions
    assert_eq!(ukf.state().len(), 2);
    assert_eq!(ukf.covariance().len(), 4);

    // Verify state values
    assert_eq!(ukf.state()[0], 0.0, "Initial state[0] should be 0.0");
    assert_eq!(ukf.state()[1], 1.0, "Initial state[1] should be 1.0");

    // Verify UKF parameters were set correctly (params is a field, not a method)
    assert_eq!(ukf.params.alpha, 0.001, "Alpha should be 0.001");
    assert_eq!(ukf.params.beta, 2.0, "Beta should be 2.0");
    assert_eq!(ukf.params.kappa, 0.0, "Kappa should be 0.0");
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

    // Verify state values
    assert_eq!(ckf.state()[0], 0.0, "Initial state[0] should be 0.0");
    assert_eq!(ckf.state()[1], 1.0, "Initial state[1] should be 1.0");

    // Verify covariance
    assert_eq!(ckf.covariance()[0], 1.0, "P[0,0] should be 1.0");
    assert_eq!(ckf.covariance()[3], 1.0, "P[1,1] should be 1.0");
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

    // Verify dimensions
    assert_eq!(enkf.mean().len(), 2);
    assert_eq!(enkf.ensemble_size, 50);

    // Verify ensemble mean is close to initial mean
    let mean = enkf.mean();
    assert!((mean[0] - 0.0).abs() < 0.5, "Ensemble mean[0] should be near 0.0, got {}", mean[0]);
    assert!((mean[1] - 1.0).abs() < 0.5, "Ensemble mean[1] should be near 1.0, got {}", mean[1]);

    // Verify spread is reasonable (should be around initial_spread)
    let spread = enkf.spread();
    assert!(spread[0] > 0.0, "Spread[0] should be positive");
    assert!(spread[1] > 0.0, "Spread[1] should be positive");
    assert!(spread[0] < 3.0, "Spread[0] shouldn't be too large, got {}", spread[0]);
    assert!(spread[1] < 3.0, "Spread[1] shouldn't be too large, got {}", spread[1]);
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

    // Verify state values: Y = [0, 0], so state = inv(I)*Y = [0, 0]
    assert_eq!(state[0], 0.0, "State[0] should be 0.0");
    assert_eq!(state[1], 0.0, "State[1] should be 0.0");

    // Verify dimensions are correct
    assert_eq!(if_filter.state_dim, 2);
    assert_eq!(if_filter.measurement_dim, 1);
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

    // Verify dimensions
    assert_eq!(pf.state_dim, 2);
    assert_eq!(pf.num_particles, 100);
    assert_eq!(pf.ess_threshold, 50.0);

    // Verify particle mean is close to initial mean
    let mean = pf.mean();
    assert!((mean[0] - 0.0_f64).abs() < 0.5, "Particle mean[0] should be near 0.0, got {}", mean[0]);
    assert!((mean[1] - 1.0_f64).abs() < 0.5, "Particle mean[1] should be near 1.0, got {}", mean[1]);

    // Verify particles were created
    assert_eq!(pf.particles.len(), 100, "Should have 100 particles");

    // Verify all particles have correct state dimension
    for (i, particle) in pf.particles.iter().enumerate() {
        assert_eq!(particle.state.len(), 2, "Particle {} should have 2-D state", i);
        assert!(particle.weight > 0.0, "Particle {} should have positive weight", i);
    }

    // Verify initial weights sum to 1
    let total_weight: f64 = pf.particles.iter().map(|p| p.weight).sum();
    assert!((total_weight - 1.0).abs() < 1e-10, "Weights should sum to 1.0, got {}", total_weight);
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
