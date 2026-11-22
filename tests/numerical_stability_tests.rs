//! Numerical stability and edge case tests
//!
//! These tests verify that filters handle challenging numerical situations correctly,
//! not just that they compile and run with well-conditioned inputs.

use kalman_filters::linear::LinearKalmanFilter as KalmanFilter;
use kalman_filters::{
    ExtendedKalmanFilterBuilder, InformationFilterBuilder, NonlinearSystem,
    UnscentedKalmanFilterBuilder,
};

const EPSILON: f64 = 1e-10;

// =============================================================================
// NUMERICAL CONDITIONING TESTS
// =============================================================================

#[test]
fn test_kf_ill_conditioned_covariance_recovery() {
    // Start with poorly conditioned covariance (high condition number)
    let mut kf = KalmanFilter::<f64>::new(
        2,
        1,
        &[0.0, 0.0],
        &[1000.0, 999.0, 999.0, 1000.0], // Condition number ≈ 1000
        &[1.0, 0.0, 0.0, 1.0],
        &[0.01, 0.0, 0.0, 0.01],
        &[1.0, 0.0],
        &[0.1],
    )
    .unwrap();

    // Multiple updates should regularize the covariance
    for i in 0..100 {
        kf.predict().unwrap();
        let result = kf.update(&[i as f64 * 0.1]);
        assert!(result.is_ok(), "Update {} failed: {:?}", i, result);
    }

    // Covariance should be well-conditioned now
    let det = kf.covariance()[0] * kf.covariance()[3] - kf.covariance()[1] * kf.covariance()[2];
    assert!(det > EPSILON, "Covariance became singular: det={}", det);
    assert!(
        det < 100.0,
        "Covariance still poorly conditioned: det={}",
        det
    );
}

#[test]
fn test_kf_extreme_measurement_noise_ratio() {
    // Test when measurement noise is orders of magnitude different from process noise
    let mut kf = KalmanFilter::<f64>::new(
        2,
        1,
        &[0.0, 0.0],
        &[1.0, 0.0, 0.0, 1.0],
        &[1.0, 0.1, 0.0, 1.0],
        &[1e-12, 0.0, 0.0, 1e-12], // Tiny process noise
        &[1.0, 0.0],
        &[1e6], // Huge measurement noise
    )
    .unwrap();

    // Should still be able to update without numerical issues
    for _ in 0..20 {
        kf.predict().unwrap();
        let result = kf.update(&[1.0]);
        assert!(result.is_ok(), "Update failed with extreme noise ratio");
    }

    // Kalman gain should be tiny (trust prior much more than measurement)
    // After predict: S = H*P*H' + R ≈ R
    // K = P*H'/S ≈ P*H'/R (very small)
    // State should barely move from measurements
    assert!(
        kf.state()[0].abs() < 0.01,
        "State moved too much with huge measurement noise: {}",
        kf.state()[0]
    );
}

#[test]
fn test_kf_perfect_measurements() {
    // When measurements have zero noise (theoretically), covariance should collapse
    // x0 = 5, P0 = 10, F = 1, Q = 0, H = 1, R nearly zero
    let mut kf =
        KalmanFilter::<f64>::new(1, 1, &[5.0], &[10.0], &[1.0], &[0.0], &[1.0], &[1e-15]).unwrap();

    kf.predict().unwrap();
    let result = kf.update(&[0.0]);
    assert!(result.is_ok(), "Should handle near-zero measurement noise");

    // State should jump to measurement
    assert!(
        (kf.state()[0] - 0.0).abs() < 1e-6,
        "State should be at measurement: {}",
        kf.state()[0]
    );

    // Covariance should be very small but not exactly zero (numerical floor)
    assert!(kf.covariance()[0] < 1e-10, "Covariance should be tiny: {}", kf.covariance()[0]);
}

#[test]
fn test_kf_loss_of_observability() {
    // Observable system loses observability mid-flight
    let f = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    let q = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
    let mut kf = KalmanFilter::<f64>::new(
        3,
        2,
        &[0.0, 0.0, 0.0],
        &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        &f,
        &q,
        &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0], // Observe x and y
        &[0.1, 0.0, 0.0, 0.1],
    )
    .unwrap();

    // Run with full observability
    for _ in 0..10 {
        kf.predict().unwrap();
        kf.update(&[1.0, 2.0]).unwrap();
    }
    let p_before = kf.covariance()[8]; // Variance of z (unobservable)

    // Lose observability of y (only observe x now)
    // (the model is fixed per filter, so continue from the current estimate)
    kf = KalmanFilter::<f64>::new(3, 1, kf.state(), kf.covariance(), &f, &q, &[1.0, 0.0, 0.0], &[0.1])
        .unwrap(); // Only observe x

    // Run without y observation
    for _ in 0..20 {
        kf.predict().unwrap();
        kf.update(&[1.0]).unwrap();
    }

    // Uncertainty in z should have grown
    assert!(
        kf.covariance()[8] > p_before,
        "Unobservable state uncertainty should grow: {} <= {}",
        kf.covariance()[8],
        p_before
    );
}

// =============================================================================
// INFORMATION FILTER NUMERICAL TESTS
// =============================================================================

#[test]
fn test_if_near_singular_information_matrix() {
    // Information matrix is nearly singular (very uncertain)
    let Y_init = vec![1e-8, 0.0, 0.0, 1e-8]; // Nearly zero information
    let y_init = vec![0.0, 0.0];

    let mut inf = InformationFilterBuilder::new(2, 1)
        .initial_information_matrix(Y_init)
        .initial_information_vector(y_init)
        .state_transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![0.1])
        .build()
        .unwrap();

    // First measurement should provide most of the information
    let result = inf.update(&[1.0]);
    assert!(
        result.is_ok(),
        "Should handle near-singular initial information"
    );

    // Information matrix should increase significantly
    assert!(
        inf.state.Y[0] > 1e-6,
        "Information should increase after measurement: {}",
        inf.state.Y[0]
    );
}

#[test]
fn test_if_information_accumulation_consistency() {
    // Verify that sequential updates give same result as batch update
    let Y_init = vec![1.0, 0.0, 0.0, 1.0];
    let y_init = vec![0.0, 0.0];

    // Sequential updates
    let mut inf_seq = InformationFilterBuilder::new(2, 1)
        .initial_information_matrix(Y_init.clone())
        .initial_information_vector(y_init.clone())
        .state_transition_matrix(vec![1.0, 0.0, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![0.1])
        .build()
        .unwrap();

    inf_seq.predict().unwrap();
    inf_seq.update(&[1.0]).unwrap();
    inf_seq.predict().unwrap();
    inf_seq.update(&[2.0]).unwrap();

    let state_seq = inf_seq.get_state().unwrap();

    // Batch-like fusion
    let mut inf_batch = InformationFilterBuilder::new(2, 1)
        .initial_information_matrix(Y_init)
        .initial_information_vector(y_init)
        .state_transition_matrix(vec![1.0, 0.0, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![0.1])
        .build()
        .unwrap();

    inf_batch.predict().unwrap();
    inf_batch.predict().unwrap();
    // Fuse both measurements
    inf_batch
        .fuse_measurements(vec![
            (&[1.0][..], &[1.0, 0.0][..], &[0.1][..]),
            (&[2.0][..], &[1.0, 0.0][..], &[0.1][..]),
        ])
        .unwrap();

    let state_batch = inf_batch.get_state().unwrap();

    // States should be similar (not exact due to different prediction ordering)
    let diff_val: f64 = state_seq[0] - state_batch[0];
    let diff = diff_val.abs();
    assert!(
        diff < 0.5_f64,
        "Sequential and batch information fusion should be similar: diff={}",
        diff
    );
}

// =============================================================================
// EXTENDED/UNSCENTED FILTER NONLINEARITY TESTS
// =============================================================================

struct HighlyNonlinearSystem;

impl NonlinearSystem<f64> for HighlyNonlinearSystem {
    fn state_transition(&self, state: &[f64], _: Option<&[f64]>, dt: f64) -> Vec<f64> {
        // Van der Pol oscillator (highly nonlinear)
        let x = state[0];
        let v = state[1];
        let mu = 2.0;
        vec![x + v * dt, v + (mu * (1.0 - x * x) * v - x) * dt]
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        // Nonlinear measurement: range^2 + bearing
        let r2 = state[0] * state[0] + state[1] * state[1];
        let bearing = state[1].atan2(state[0]);
        vec![r2, bearing]
    }

    fn state_jacobian(&self, state: &[f64], _: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let x = state[0];
        let v = state[1];
        let mu = 2.0;
        vec![
            1.0,
            dt,
            (mu * (-2.0 * x * v) - 1.0) * dt,
            1.0 + mu * (1.0 - x * x) * dt,
        ]
    }

    fn measurement_jacobian(&self, state: &[f64]) -> Vec<f64> {
        let x = state[0];
        let y = state[1];
        let r2 = x * x + y * y;
        vec![2.0 * x, 2.0 * y, -y / r2, x / r2]
    }

    fn state_dim(&self) -> usize {
        2
    }
    fn measurement_dim(&self) -> usize {
        2
    }
}

#[test]
fn test_ekf_vs_ukf_on_highly_nonlinear_system() {
    // UKF should outperform EKF on highly nonlinear systems
    let system_ekf = HighlyNonlinearSystem;
    let system_ukf = HighlyNonlinearSystem;

    let init_state = vec![1.0, 0.0];
    let init_cov = vec![0.1, 0.0, 0.0, 0.1];
    let q = vec![0.001, 0.0, 0.0, 0.001];
    let r = vec![0.01, 0.0, 0.0, 0.01];
    let dt = 0.01;

    let mut ekf = ExtendedKalmanFilterBuilder::new(system_ekf)
        .initial_state(init_state.clone())
        .initial_covariance(init_cov.clone())
        .process_noise(q.clone())
        .measurement_noise(r.clone())
        .dt(dt)
        .build()
        .unwrap();

    let mut ukf = UnscentedKalmanFilterBuilder::new(system_ukf)
        .initial_state(init_state.clone())
        .initial_covariance(init_cov.clone())
        .process_noise(q.clone())
        .measurement_noise(r.clone())
        .dt(dt)
        .build()
        .unwrap();

    // Use measurements as proxy for true trajectory
    let measurements = vec![
        vec![1.0, 0.0],
        vec![0.98, 0.1],
        vec![0.94, 0.2],
        vec![0.88, 0.28],
        vec![0.80, 0.35],
    ];

    let mut ekf_errors = Vec::new();
    let mut ukf_errors = Vec::new();

    for meas in measurements {
        ekf.predict();
        ukf.predict().unwrap();

        ekf.update(&meas).unwrap();
        ukf.update(&meas).unwrap();

        // Compute errors (we're using measurements as proxy for truth)
        let ekf_err =
            ((ekf.state()[0] - meas[0]).powi(2) + (ekf.state()[1] - meas[1]).powi(2)).sqrt();
        let ukf_err =
            ((ukf.state()[0] - meas[0]).powi(2) + (ukf.state()[1] - meas[1]).powi(2)).sqrt();

        ekf_errors.push(ekf_err);
        ukf_errors.push(ukf_err);
    }

    // UKF should have lower or comparable error on this nonlinear system
    let ekf_mean_err: f64 = ekf_errors.iter().sum::<f64>() / ekf_errors.len() as f64;
    let ukf_mean_err: f64 = ukf_errors.iter().sum::<f64>() / ukf_errors.len() as f64;

    // UKF shouldn't be significantly worse than EKF (both should converge)
    assert!(
        ukf_mean_err < ekf_mean_err * 2.0_f64,
        "UKF should perform reasonably on nonlinear system: EKF={}, UKF={}",
        ekf_mean_err,
        ukf_mean_err
    );
}

// =============================================================================
// OVERFLOW/UNDERFLOW TESTS
// =============================================================================

#[test]
fn test_kf_prevents_covariance_underflow() {
    // Covariance should not underflow to exactly zero
    // No process noise, tiny measurement noise
    let mut kf =
        KalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[0.0], &[1.0], &[1e-10]).unwrap();

    // Many updates with near-perfect measurements
    for i in 0..1000 {
        kf.predict().unwrap();
        kf.update(&[i as f64]).unwrap();
    }

    // Covariance should remain positive (numerical floor)
    assert!(
        kf.covariance()[0] > 0.0,
        "Covariance should not underflow to zero: {}",
        kf.covariance()[0]
    );
}

#[test]
fn test_kf_prevents_covariance_overflow() {
    // Covariance should not overflow with large process noise
    // Huge process noise, huge measurement noise
    let mut kf =
        KalmanFilter::<f64>::new(1, 1, &[0.0], &[1.0], &[1.0], &[1e6], &[1.0], &[1e6]).unwrap();

    // Many predict steps without update
    for _ in 0..100 {
        kf.predict().unwrap();
    }

    // Covariance should be large but not infinity
    assert!(
        kf.covariance()[0].is_finite(),
        "Covariance should not overflow: {}",
        kf.covariance()[0]
    );
    assert!(kf.covariance()[0] < 1e20, "Covariance too large: {}", kf.covariance()[0]);
}

// =============================================================================
// DEGENERATE INPUT TESTS
// =============================================================================

#[test]
fn test_kf_all_zero_matrices() {
    // Pathological case: all zero matrices except R
    let mut kf = KalmanFilter::<f64>::new(
        2,
        1,
        &[1.0, 2.0],
        &[0.0, 0.0, 0.0, 0.0],
        &[0.0, 0.0, 0.0, 0.0], // System collapses to origin
        &[0.0, 0.0, 0.0, 0.0],
        &[0.0, 0.0], // No observation
        &[1.0],      // Only R is non-zero
    )
    .unwrap();

    kf.predict().unwrap();

    // State should collapse to zero (F=0)
    assert_eq!(kf.state()[0], 0.0, "State should be zero with F=0");
    assert_eq!(kf.state()[1], 0.0, "State should be zero with F=0");

    // Update should fail or do nothing (H=0 means no information)
    let result = kf.update(&[5.0]);
    // Depending on implementation, this might fail or succeed with no change
    if result.is_ok() {
        // State should not change (no observable info)
        assert_eq!(kf.state()[0], 0.0);
        assert_eq!(kf.state()[1], 0.0);
    }
}

#[test]
fn test_kf_identity_system_stability() {
    // Pure identity system should be stable
    let mut kf = KalmanFilter::<f64>::new(
        3,
        1,
        &[1.0, 2.0, 3.0],
        &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        &[0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
        &[1.0, 0.0, 0.0],
        &[0.5],
    )
    .unwrap();

    // Run for many iterations
    for i in 0..100 {
        kf.predict().unwrap();
        kf.update(&[1.0]).unwrap();

        // Check stability (values shouldn't explode)
        for j in 0..3 {
            assert!(
                kf.state()[j].abs() < 1000.0,
                "State {} exploded at iteration {}: {}",
                j,
                i,
                kf.state()[j]
            );
            assert!(
                kf.covariance()[j * 3 + j] < 1000.0,
                "Covariance {} exploded at iteration {}: {}",
                j,
                i,
                kf.covariance()[j * 3 + j]
            );
        }
    }
}
