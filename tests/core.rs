//! Unit tests for core Kalman filter functions
//!
//! These tests verify actual business logic and mathematical correctness
//! of individual functions, not just type system compliance.

use kalman_filters::filter::KalmanFilter;
use kalman_filters::*;

// =============================================================================
// TESTS FOR KALMAN FILTER CORE METHODS
// =============================================================================

#[test]
fn test_state_returns_correct_reference() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![1.5, 2.7];

    let state = kf.state();
    assert_eq!(state.len(), 2);
    assert_eq!(state[0], 1.5);
    assert_eq!(state[1], 2.7);
}

#[test]
fn test_state_is_live_reference() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![1.0, 2.0];

    let state1 = kf.state();
    assert_eq!(state1[0], 1.0);

    // Modify internal state
    kf.x[0] = 99.0;

    let state2 = kf.state();
    assert_eq!(state2[0], 99.0, "state() should return live reference");
}

#[test]
fn test_covariance_returns_correct_reference() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.P = vec![1.0, 0.5, 0.5, 2.0];

    let cov = kf.covariance();
    assert_eq!(cov.len(), 4);
    assert_eq!(cov[0], 1.0);
    assert_eq!(cov[1], 0.5);
    assert_eq!(cov[2], 0.5);
    assert_eq!(cov[3], 2.0);
}

#[test]
fn test_covariance_is_live_reference() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.P = vec![1.0, 0.0, 0.0, 1.0];

    let cov1 = kf.covariance();
    assert_eq!(cov1[0], 1.0);

    // Modify covariance
    kf.P[0] = 5.5;

    let cov2 = kf.covariance();
    assert_eq!(cov2[0], 5.5, "covariance() should return live reference");
}

// =============================================================================
// TESTS FOR INITIALIZE METHOD
// =============================================================================

#[test]
fn test_initialize_with_valid_inputs() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1],
    );

    assert!(result.is_ok());
    let kf = result.unwrap();
    assert_eq!(kf.state_dim, 2);
    assert_eq!(kf.measurement_dim, 1);
}

#[test]
fn test_initialize_state_dimension_mismatch() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0], // Wrong: should be length 2
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1],
    );

    assert!(result.is_err());
    match result {
        Err(KalmanError::DimensionMismatch { expected, actual }) => {
            assert_eq!(expected, (2, 1));
            assert_eq!(actual, (1, 1));
        }
        _ => panic!("Expected DimensionMismatch error"),
    }
}

#[test]
fn test_initialize_covariance_dimension_mismatch() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 1.0],
        vec![1.0, 0.0, 0.0], // Wrong: should be 4 elements (2x2)
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1],
    );

    assert!(result.is_err());
}

#[test]
fn test_initialize_transition_matrix_dimension_mismatch() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1], // Wrong: should be 4 elements (2x2)
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1],
    );

    assert!(result.is_err());
}

#[test]
fn test_initialize_process_noise_dimension_mismatch() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0], // Wrong: should be 4 elements (2x2)
        vec![1.0, 0.0],
        vec![0.1],
    );

    assert!(result.is_err());
}

#[test]
fn test_initialize_observation_matrix_dimension_mismatch() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0], // Wrong: should be 2 elements (1x2)
        vec![0.1],
    );

    assert!(result.is_err());
}

#[test]
fn test_initialize_measurement_noise_dimension_mismatch() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1, 0.2], // Wrong: should be 1 element (1x1)
    );

    assert!(result.is_err());
}

#[test]
fn test_initialize_negative_measurement_noise() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![-0.1], // Invalid: measurement noise must be positive
    );

    assert!(result.is_err());
    match result {
        Err(KalmanError::InvalidNoiseCovariance) => {}
        _ => panic!("Expected InvalidNoiseCovariance error"),
    }
}

#[test]
fn test_initialize_zero_measurement_noise() {
    let result = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 1.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.0], // Invalid: measurement noise must be positive
    );

    assert!(result.is_err());
}

// =============================================================================
// TESTS FOR PREDICT METHOD CORRECTNESS
// =============================================================================

#[test]
fn test_predict_updates_state_correctly() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    // Simple constant velocity model
    kf.x = vec![0.0, 1.0]; // position=0, velocity=1
    kf.F = vec![1.0, 1.0, 0.0, 1.0]; // dt=1
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];

    kf.predict();

    // After prediction: position should be 0 + 1*1 = 1
    assert!((kf.x[0] - 1.0).abs() < 1e-10, "Position should be 1.0, got {}", kf.x[0]);
    assert!((kf.x[1] - 1.0).abs() < 1e-10, "Velocity should remain 1.0, got {}", kf.x[1]);
}

#[test]
fn test_predict_increases_covariance_with_process_noise() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 0.0, 0.0, 1.0]; // Identity
    kf.Q = vec![0.1, 0.0, 0.0, 0.1]; // Process noise

    let initial_trace = kf.P[0] + kf.P[3];

    kf.predict();

    let final_trace = kf.P[0] + kf.P[3];

    assert!(
        final_trace > initial_trace,
        "Process noise should increase covariance: {} <= {}",
        final_trace,
        initial_trace
    );
}

#[test]
fn test_predict_with_zero_process_noise_maintains_covariance() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 0.0, 0.0, 1.0]; // Identity
    kf.Q = vec![0.0, 0.0, 0.0, 0.0]; // No process noise

    let initial_p = kf.P.clone();

    kf.predict();

    // For identity F and zero Q, P should remain unchanged
    for i in 0..4 {
        assert!(
            (kf.P[i] - initial_p[i]).abs() < 1e-10,
            "Covariance element {} changed with identity F and zero Q",
            i
        );
    }
}

#[test]
fn test_predict_with_deterministic_dynamics() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    // Simple 1D case: x[k+1] = 2 * x[k]
    kf.x = vec![5.0];
    kf.F = vec![2.0];
    kf.P = vec![1.0];
    kf.Q = vec![0.0];

    kf.predict();

    assert_eq!(kf.x[0], 10.0, "State should double");
    assert_eq!(kf.P[0], 4.0, "Variance should be multiplied by F^2 = 4");
}

// =============================================================================
// TESTS FOR UPDATE METHOD CORRECTNESS
// =============================================================================

#[test]
fn test_update_pulls_state_toward_measurement() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![0.0]; // Prior estimate: 0
    kf.P = vec![1.0]; // Prior uncertainty
    kf.H = vec![1.0]; // Observe state directly
    kf.R = vec![1.0]; // Measurement uncertainty
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    let measurement = vec![10.0];
    kf.update(&measurement).unwrap();

    // With equal uncertainties, posterior should be halfway between prior and measurement
    assert!(
        kf.x[0] > 0.0 && kf.x[0] < 10.0,
        "State should be between prior (0) and measurement (10), got {}",
        kf.x[0]
    );
    assert!(
        (kf.x[0] - 5.0).abs() < 1.0,
        "With equal uncertainties, should be near midpoint, got {}",
        kf.x[0]
    );
}

#[test]
fn test_update_reduces_uncertainty() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![0.0];
    kf.P = vec![10.0]; // High uncertainty
    kf.H = vec![1.0];
    kf.R = vec![1.0]; // Lower measurement uncertainty
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    let prior_uncertainty = kf.P[0];

    kf.update(&vec![5.0]).unwrap();

    assert!(
        kf.P[0] < prior_uncertainty,
        "Measurement should reduce uncertainty: {} >= {}",
        kf.P[0],
        prior_uncertainty
    );
}

#[test]
fn test_update_with_perfect_measurement_eliminates_uncertainty() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![0.0];
    kf.P = vec![10.0];
    kf.H = vec![1.0];
    kf.R = vec![0.0001]; // Nearly perfect measurement
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    kf.update(&vec![7.0]).unwrap();

    // With very accurate measurement, posterior should be very close to measurement
    assert!(
        (kf.x[0] - 7.0).abs() < 0.1,
        "With accurate measurement, state should be close to measurement value, got {}",
        kf.x[0]
    );

    // Uncertainty should be dramatically reduced
    assert!(
        kf.P[0] < 0.01,
        "With accurate measurement, uncertainty should be very low, got {}",
        kf.P[0]
    );
}

#[test]
fn test_update_with_noisy_measurement_stays_near_prior() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![5.0]; // Confident prior
    kf.P = vec![0.01]; // Very certain of prior
    kf.H = vec![1.0];
    kf.R = vec![100.0]; // Very noisy measurement
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    kf.update(&vec![50.0]).unwrap(); // Measurement very different from prior

    // With noisy measurement, should stay close to prior
    assert!(
        (kf.x[0] - 5.0).abs() < 2.0,
        "With noisy measurement, should trust prior more, got {}",
        kf.x[0]
    );
}

#[test]
fn test_update_measurement_dimension_mismatch() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.H = vec![1.0, 0.0];
    kf.R = vec![1.0];
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];

    // Wrong measurement dimension
    let result = kf.update(&vec![1.0, 2.0]);

    assert!(result.is_err());
    match result {
        Err(KalmanError::DimensionMismatch { expected, actual }) => {
            assert_eq!(expected, (1, 1));
            assert_eq!(actual, (2, 1));
        }
        _ => panic!("Expected DimensionMismatch error"),
    }
}

#[test]
fn test_update_with_zero_innovation() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![5.0];
    kf.P = vec![1.0];
    kf.H = vec![1.0];
    kf.R = vec![0.1];
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    let initial_state = kf.x[0];

    // Measurement equals prediction
    kf.update(&vec![5.0]).unwrap();

    // State should barely change (innovation is zero)
    assert!(
        (kf.x[0] - initial_state).abs() < 0.5,
        "With zero innovation, state should barely change"
    );
}

// =============================================================================
// TESTS FOR MULTI-DIMENSIONAL SYSTEMS
// =============================================================================

#[test]
fn test_2d_position_velocity_tracking() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    let dt = 0.1;
    kf.x = vec![0.0, 10.0]; // position=0, velocity=10
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, dt, 0.0, 1.0]; // Constant velocity model
    kf.H = vec![1.0, 0.0]; // Observe position only
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.R = vec![0.1];

    // Predict
    kf.predict();

    // Position should have moved by velocity * dt
    assert!(
        (kf.x[0] - 1.0).abs() < 0.01,
        "Position should be ~1.0 after dt=0.1 with v=10, got {}",
        kf.x[0]
    );

    // Update with measurement
    let measurement = vec![0.95]; // Slightly off from prediction
    kf.update(&measurement).unwrap();

    // State should incorporate measurement
    assert!(
        (kf.x[0] - 0.95).abs() < 0.1,
        "Position should be near measurement after update"
    );
}

#[test]
fn test_3d_state_2d_measurement() {
    let mut kf = KalmanFilter::<f64>::new(3, 2);

    kf.x = vec![1.0, 2.0, 3.0];
    kf.P = vec![
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ];
    kf.F = vec![
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ];
    kf.Q = vec![
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.01,
    ];
    // Observe first two states only
    kf.H = vec![
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
    ];
    kf.R = vec![0.1, 0.0, 0.0, 0.1];

    kf.predict();

    // Measurement of first two states
    kf.update(&vec![0.9, 2.1]).unwrap();

    // First two states should be near measurements
    assert!((kf.x[0] - 0.9).abs() < 0.2);
    assert!((kf.x[1] - 2.1).abs() < 0.2);
    // Third state should be largely unchanged (unobserved)
    assert!((kf.x[2] - 3.0).abs() < 0.5);
}

// =============================================================================
// TESTS FOR NUMERICAL EDGE CASES
// =============================================================================

#[test]
fn test_very_small_values_dont_cause_nan() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![1e-8];
    kf.P = vec![1e-10];
    kf.F = vec![1.0];
    kf.H = vec![1.0];
    kf.Q = vec![1e-12];
    kf.R = vec![1e-10];

    kf.predict();
    let result = kf.update(&vec![1e-8]);

    // Should either work or fail gracefully, not produce NaN
    if result.is_ok() {
        assert!(!kf.x[0].is_nan(), "State should not be NaN");
        assert!(!kf.P[0].is_nan(), "Covariance should not be NaN");
    }
}

#[test]
fn test_very_large_values_dont_overflow() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![1e10];
    kf.P = vec![1e10];
    kf.F = vec![1.0];
    kf.H = vec![1.0];
    kf.Q = vec![1.0];
    kf.R = vec![1.0];

    kf.predict();
    kf.update(&vec![1e10]).unwrap();

    assert!(!kf.x[0].is_infinite(), "State should not overflow to infinity");
    assert!(!kf.P[0].is_infinite(), "Covariance should not overflow");
}

// =============================================================================
// TESTS FOR MATRIX OPERATIONS CORRECTNESS
// =============================================================================

#[test]
fn test_matrix_multiplication_in_predict_is_correct() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    // Use a known matrix multiplication
    kf.x = vec![2.0, 3.0];
    kf.F = vec![1.0, 2.0, 3.0, 4.0]; // F = [[1, 2], [3, 4]]
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];

    kf.predict();

    // F * x = [[1, 2], [3, 4]] * [[2], [3]] = [[8], [18]]
    assert!((kf.x[0] - 8.0).abs() < 1e-10, "Expected x[0]=8, got {}", kf.x[0]);
    assert!((kf.x[1] - 18.0).abs() < 1e-10, "Expected x[1]=18, got {}", kf.x[1]);
}

#[test]
fn test_observation_matrix_multiplication_in_update() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![5.0, 3.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.H = vec![2.0, 1.0]; // H = [2, 1]
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];
    kf.R = vec![0.01]; // Very confident measurement

    // H * x = [2, 1] * [5, 3] = 13
    // So innovation for measurement z=15 should be 15 - 13 = 2

    kf.update(&vec![15.0]).unwrap();

    // With very confident measurement (small R), state should move toward measurement
    // H * x_new should be close to 15
    let predicted_measurement = kf.H[0] * kf.x[0] + kf.H[1] * kf.x[1];
    assert!(
        (predicted_measurement - 15.0).abs() < 1.0,
        "H*x should be close to measurement after update, got {}",
        predicted_measurement
    );
}

#[test]
fn test_symmetric_covariance_maintained_after_predict() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.5, 0.5, 2.0]; // Symmetric
    kf.F = vec![1.0, 0.1, 0.0, 1.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];

    kf.predict();

    // Check symmetry: P[0,1] should equal P[1,0]
    assert!(
        (kf.P[1] - kf.P[2]).abs() < 1e-10,
        "Covariance should remain symmetric: P[0,1]={} != P[1,0]={}",
        kf.P[1],
        kf.P[2]
    );
}

#[test]
fn test_symmetric_covariance_maintained_after_update() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.3, 0.3, 1.0]; // Symmetric
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.H = vec![1.0, 0.5];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];
    kf.R = vec![0.1];

    kf.update(&vec![1.5]).unwrap();

    // Check symmetry
    assert!(
        (kf.P[1] - kf.P[2]).abs() < 1e-10,
        "Covariance should remain symmetric after update: P[0,1]={} != P[1,0]={}",
        kf.P[1],
        kf.P[2]
    );
}

// =============================================================================
// TESTS FOR KALMAN GAIN CALCULATION
// =============================================================================

#[test]
fn test_kalman_gain_computation_1d() {
    // For 1D case, we can compute Kalman gain analytically
    // K = P * H^T * (H * P * H^T + R)^-1
    // With H=1, this simplifies to: K = P / (P + R)

    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![0.0];
    kf.P = vec![2.0]; // Prior uncertainty = 2
    kf.H = vec![1.0];
    kf.R = vec![3.0]; // Measurement noise = 3
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    let prior_state = kf.x[0];
    let measurement = 10.0;

    kf.update(&vec![measurement]).unwrap();

    // Expected Kalman gain: K = P / (P + R) = 2 / (2 + 3) = 0.4
    // Expected state update: x_new = x_old + K * (z - x_old)
    //                              = 0 + 0.4 * (10 - 0) = 4.0

    let expected_state = prior_state + 0.4 * (measurement - prior_state);
    assert!(
        (kf.x[0] - expected_state).abs() < 1e-10,
        "Kalman gain calculation error: expected {}, got {}",
        expected_state,
        kf.x[0]
    );
}

#[test]
fn test_kalman_gain_with_high_measurement_noise() {
    // When R >> P, Kalman gain should be near 0 (don't trust measurement)
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![5.0];
    kf.P = vec![1.0];
    kf.H = vec![1.0];
    kf.R = vec![1000.0]; // Very high measurement noise
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    let prior_state = kf.x[0];
    kf.update(&vec![100.0]).unwrap();

    // State should barely change because measurement is unreliable
    assert!(
        (kf.x[0] - prior_state).abs() < 0.5,
        "With high R, state should barely change from prior"
    );
}

#[test]
fn test_kalman_gain_with_high_prior_uncertainty() {
    // When P >> R, Kalman gain should be near 1 (trust measurement fully)
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![0.0];
    kf.P = vec![1000.0]; // Very uncertain prior
    kf.H = vec![1.0];
    kf.R = vec![0.1]; // Confident measurement
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    kf.update(&vec![42.0]).unwrap();

    // State should be very close to measurement
    assert!(
        (kf.x[0] - 42.0).abs() < 1.0,
        "With high P, state should be close to measurement, got {}",
        kf.x[0]
    );
}

// =============================================================================
// TESTS FOR INNOVATION CALCULATION
// =============================================================================

#[test]
fn test_innovation_calculation() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![10.0, 5.0]; // position=10, velocity=5
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.H = vec![1.0, 0.5]; // Observe position + 0.5*velocity
    kf.R = vec![0.1];
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];

    // Predicted measurement: H * x = 1*10 + 0.5*5 = 12.5
    // Actual measurement: 15.0
    // Expected innovation: 15.0 - 12.5 = 2.5

    let initial_prediction = kf.H[0] * kf.x[0] + kf.H[1] * kf.x[1];
    let measurement = 15.0;
    let expected_innovation = measurement - initial_prediction;

    assert!(
        (expected_innovation - 2.5).abs() < 1e-10,
        "Innovation should be 2.5, calculated {}",
        expected_innovation
    );

    kf.update(&vec![measurement]).unwrap();

    // After update, predicted measurement should be closer to actual measurement
    let final_prediction = kf.H[0] * kf.x[0] + kf.H[1] * kf.x[1];
    assert!(
        (final_prediction - measurement).abs() < (initial_prediction - measurement).abs(),
        "Update should bring prediction closer to measurement"
    );
}

// =============================================================================
// TESTS FOR COVARIANCE PROPERTIES
// =============================================================================

#[test]
fn test_covariance_positive_definite_after_predict() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.2, 0.2, 1.0]; // Positive definite
    kf.F = vec![1.0, 0.1, 0.0, 1.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];

    kf.predict();

    // Check positive definite: diagonal elements positive and det > 0
    assert!(kf.P[0] > 0.0, "P[0,0] should be positive");
    assert!(kf.P[3] > 0.0, "P[1,1] should be positive");

    let det = kf.P[0] * kf.P[3] - kf.P[1] * kf.P[2];
    assert!(det > 0.0, "Determinant should be positive for positive definite matrix");
}

#[test]
fn test_covariance_decreases_after_update() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![10.0, 0.0, 0.0, 10.0];
    kf.H = vec![1.0, 0.0];
    kf.R = vec![1.0];
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];

    let prior_trace = kf.P[0] + kf.P[3];

    kf.update(&vec![5.0]).unwrap();

    let posterior_trace = kf.P[0] + kf.P[3];

    assert!(
        posterior_trace < prior_trace,
        "Trace of covariance should decrease after update: {} >= {}",
        posterior_trace,
        prior_trace
    );
}

//  =============================================================================
// TESTS FOR MATRIX OPERATIONS WITH KNOWN RESULTS
// =============================================================================

#[test]
fn test_matrix_vector_multiply_3x3() {
    // Test via predict step: x' = F * x
    let mut kf = KalmanFilter::<f64>::new(3, 1);

    kf.x = vec![1.0, 2.0, 3.0];
    // F = [[2, 0, 1],
    //      [0, 3, 0],
    //      [1, 0, 2]]
    kf.F = vec![
        2.0, 0.0, 1.0,
        0.0, 3.0, 0.0,
        1.0, 0.0, 2.0,
    ];
    kf.P = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0; 9];

    kf.predict();

    // F * x = [[2, 0, 1],   [[1],    [[2*1 + 0*2 + 1*3],    [[5],
    //          [0, 3, 0],  *  [2],  =  [0*1 + 3*2 + 0*3],  =  [6],
    //          [1, 0, 2]]     [3]]     [1*1 + 0*2 + 2*3]]     [7]]
    assert_eq!(kf.x[0], 5.0, "Matrix-vector multiply failed for row 0");
    assert_eq!(kf.x[1], 6.0, "Matrix-vector multiply failed for row 1");
    assert_eq!(kf.x[2], 7.0, "Matrix-vector multiply failed for row 2");
}

#[test]
fn test_matrix_matrix_multiply_in_covariance_predict() {
    let mut kf = KalmanFilter::<f64>::new(2, 1);

    kf.x = vec![0.0, 0.0];
    // F = [[2, 1],
    //      [0, 3]]
    kf.F = vec![2.0, 1.0, 0.0, 3.0];
    // P = [[1, 0],
    //      [0, 1]]
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];

    kf.predict();

    // P' = F * P * F^T + Q
    // F * P = [[2, 1],   [[1, 0],    [[2, 1],
    //          [0, 3]]  *  [0, 1]]  =  [0, 3]]
    //
    // F*P*F^T = [[2, 1],   [[2, 0],    [[2*2+1*1, 2*0+1*3],    [[5, 3],
    //            [0, 3]]  *  [1, 3]]  =  [0*2+3*1, 0*0+3*3]]  =  [3, 9]]

    assert_eq!(kf.P[0], 5.0, "P[0,0] should be 5");
    assert_eq!(kf.P[1], 3.0, "P[0,1] should be 3");
    assert_eq!(kf.P[2], 3.0, "P[1,0] should be 3 (symmetric)");
    assert_eq!(kf.P[3], 9.0, "P[1,1] should be 9");
}

// =============================================================================
// TESTS FOR EDGE CASES AND ERROR CONDITIONS
// =============================================================================

#[test]
fn test_singular_innovation_covariance_handling() {
    // This tests what happens when innovation covariance S = H*P*H^T + R becomes singular
    // This shouldn't happen with proper R, but we test robustness

    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 0.0];
    kf.P = vec![0.0, 0.0, 0.0, 0.0]; // Degenerate covariance
    kf.H = vec![1.0, 0.0];
    kf.R = vec![0.0001]; // Small but non-zero
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];

    // Should not crash or produce NaN
    let result = kf.update(&vec![1.0]);
    if result.is_ok() {
        assert!(!kf.x[0].is_nan(), "State should not be NaN");
        assert!(!kf.P[0].is_nan(), "Covariance should not be NaN");
    }
}

#[test]
fn test_identity_observation_matrix() {
    // Test when we directly observe the entire state
    let mut kf = KalmanFilter::<f64>::new(2, 2);

    kf.x = vec![1.0, 2.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.H = vec![1.0, 0.0, 0.0, 1.0]; // Identity observation
    kf.R = vec![0.1, 0.0, 0.0, 0.1];
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];

    kf.update(&vec![3.0, 4.0]).unwrap();

    // With direct observation and low R, state should move toward measurement
    assert!((kf.x[0] - 3.0).abs() < 1.0);
    assert!((kf.x[1] - 4.0).abs() < 1.0);
}

#[test]
fn test_multiple_sequential_predict_steps() {
    // Test that multiple predictions work correctly
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![1.0];
    kf.F = vec![2.0]; // State doubles each step
    kf.P = vec![1.0];
    kf.Q = vec![0.0];

    // x1 = 2*1 = 2
    kf.predict();
    assert_eq!(kf.x[0], 2.0);

    // x2 = 2*2 = 4
    kf.predict();
    assert_eq!(kf.x[0], 4.0);

    // x3 = 2*4 = 8
    kf.predict();
    assert_eq!(kf.x[0], 8.0);
}

#[test]
fn test_multiple_sequential_update_steps() {
    // Test that multiple measurements continue to refine estimate
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![0.0];
    kf.P = vec![10.0];
    kf.H = vec![1.0];
    kf.R = vec![1.0];
    kf.F = vec![1.0];
    kf.Q = vec![0.0];

    let measurements = vec![5.0, 5.1, 4.9, 5.0, 5.05];

    for &z in &measurements {
        kf.update(&vec![z]).unwrap();
    }

    // After many consistent measurements around 5.0, estimate should converge there
    assert!(
        (kf.x[0] - 5.0).abs() < 0.5,
        "After multiple measurements, state should be near 5.0, got {}",
        kf.x[0]
    );

    // Uncertainty should have decreased significantly
    assert!(
        kf.P[0] < 1.0,
        "Uncertainty should decrease with measurements, got {}",
        kf.P[0]
    );
}

// =============================================================================
// TESTS FOR REAL-WORLD FAILURE MODES
// These test actual bugs that occur in practice, not just "does math work"
// =============================================================================

#[test]
fn test_contradictory_measurements_dont_crash() {
    // Real scenario: GPS says you're moving north, compass says you're moving south
    // Bad filters crash or produce NaN. Good filters handle this gracefully.
    
    let mut kf = KalmanFilter::<f64>::new(2, 2);
    kf.x = vec![0.0, 0.0]; // position, velocity
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 1.0, 0.0, 1.0]; // dt=1
    kf.H = vec![1.0, 0.0, 0.0, 1.0]; // measure both position and velocity
    kf.Q = vec![0.1, 0.0, 0.0, 0.1];
    kf.R = vec![0.1, 0.0, 0.0, 0.1];

    // Measurement 1: position=10, velocity=5 (moving forward)
    kf.predict();
    kf.update(&vec![10.0, 5.0]).unwrap();

    // Measurement 2: position=11, velocity=-5 (suddenly reversed?!)
    kf.predict();
    let result = kf.update(&vec![11.0, -5.0]);

    // Should not crash or produce NaN
    assert!(result.is_ok(), "Filter should handle contradictory measurements");
    assert!(!kf.x[0].is_nan(), "State should not be NaN");
    assert!(!kf.x[1].is_nan(), "Velocity should not be NaN");
    assert!(!kf.P[0].is_nan(), "Covariance should not be NaN");
}

#[test]
fn test_measurement_far_from_prediction_doesnt_explode_covariance() {
    // Real scenario: GPS glitch reports you teleported 1000km
    // Bad filters: covariance explodes, filter never recovers
    // Good filters: large innovation, but bounded response
    
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![0.0];
    kf.P = vec![1.0];
    kf.F = vec![1.0];
    kf.H = vec![1.0];
    kf.Q = vec![0.01];
    kf.R = vec![1.0];

    // Normal measurements
    for i in 0..10 {
        kf.predict();
        kf.update(&vec![i as f64]).unwrap();
    }

    let stable_P = kf.P[0];

    // Massive outlier measurement
    kf.predict();
    kf.update(&vec![1000.0]).unwrap();

    // Covariance should not explode
    assert!(
        kf.P[0] < stable_P * 100.0,
        "Covariance exploded from {} to {} after outlier",
        stable_P,
        kf.P[0]
    );

    // Should still be numerically stable
    assert!(!kf.P[0].is_infinite(), "Covariance went to infinity");
    assert!(!kf.P[0].is_nan(), "Covariance became NaN");
}

#[test]
fn test_repeated_predict_without_update_increases_uncertainty() {
    // Real scenario: Sensor fails, you only have predictions (dead reckoning)
    // Uncertainty should grow WITHOUT BOUND (not saturate incorrectly)
    
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![0.0];
    kf.P = vec![1.0];
    kf.F = vec![1.0];
    kf.H = vec![1.0];
    kf.Q = vec![0.1]; // Process noise adds uncertainty each step
    kf.R = vec![1.0];

    let mut uncertainties = vec![];
    
    // Predict 100 times without any measurements
    for _ in 0..100 {
        kf.predict();
        uncertainties.push(kf.P[0]);
    }

    // Uncertainty should monotonically increase
    for i in 1..uncertainties.len() {
        assert!(
            uncertainties[i] > uncertainties[i-1],
            "Uncertainty should increase at step {}: {} <= {}",
            i, uncertainties[i], uncertainties[i-1]
        );
    }

    // Final uncertainty should be much larger than initial
    assert!(
        kf.P[0] > 5.0,
        "After 100 predictions with Q=0.1, uncertainty should be >5, got {}",
        kf.P[0]
    );
}

#[test]
fn test_zero_process_noise_with_model_error_causes_overconfidence() {
    // Real scenario: Your model is F=identity but actual system is F=1.1*x
    // With Q=0, filter becomes overconfident and ignores measurements
    // This is a FEATURE, not a bug, but good to document
    
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![1.0];
    kf.P = vec![1.0];
    kf.F = vec![1.0]; // Model: state stays constant
    kf.H = vec![1.0];
    kf.Q = vec![0.0]; // NO process noise (overconfident model)
    kf.R = vec![1.0];

    // Actual system: state grows by 10% each step
    let mut true_state = 1.0;
    
    for _ in 0..10 {
        true_state *= 1.1; // Actual dynamics
        
        kf.predict();
        kf.update(&vec![true_state]).unwrap();
    }

    // With Q=0, covariance should approach zero
    // (Filter becomes certain it knows the state, even though model is wrong)
    assert!(
        kf.P[0] < 0.5,
        "With Q=0, covariance should shrink despite model error, got {}",
        kf.P[0]
    );

    // State estimate will lag behind true state
    let error = (kf.x[0] - true_state).abs();
    assert!(
        error > 0.1,
        "Filter should have model error with Q=0, error = {}",
        error
    );
}

#[test]
fn test_near_singular_innovation_covariance_uses_regularization() {
    // Real scenario: R and H*P*H' cancel out, making S singular
    // This tests that we don't crash when inverting S
    
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 0.0];
    
    // Set up P and H so that H*P*H' ≈ 0
    kf.P = vec![
        1e-15, 0.0,
        0.0, 1.0
    ];
    kf.H = vec![1.0, 0.0]; // Only observes first state (which has tiny variance)
    kf.R = vec![1e-15]; // Also tiny
    
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.0, 0.0, 0.0, 0.0];

    // S = H*P*H' + R ≈ 1e-15 + 1e-15 = 2e-15 (nearly singular)
    let result = kf.update(&vec![1.0]);

    // Should handle gracefully (either succeed or fail gracefully)
    if result.is_ok() {
        assert!(!kf.x[0].is_nan(), "State should not be NaN after near-singular update");
        assert!(!kf.x[0].is_infinite(), "State should not be infinite");
    }
    // If it fails, that's also acceptable - just shouldn't crash
}

#[test]
fn test_large_dt_in_constant_velocity_model() {
    // Real scenario: Timestep varies (missed frames, slow sensor)
    // Large dt can make F*P*F' explode if not handled correctly
    
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 1.0]; // position=0, velocity=1
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.H = vec![1.0, 0.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.R = vec![0.1];

    // Normal dt=0.1
    kf.F = vec![1.0, 0.1, 0.0, 1.0];
    kf.predict();
    let normal_P = kf.P[0];

    // Very large dt=100 (sensor offline for a while)
    kf.F = vec![1.0, 100.0, 0.0, 1.0];
    kf.predict();

    // Uncertainty in position should grow dramatically
    assert!(
        kf.P[0] > normal_P * 10.0,
        "Position uncertainty should grow with large dt: {} vs {}",
        kf.P[0], normal_P
    );

    // But should not overflow
    assert!(!kf.P[0].is_infinite(), "Covariance should not overflow with large dt");
}

#[test]
fn test_asymmetric_covariance_stays_symmetric() {
    // Real scenario: Floating point errors accumulate and break symmetry
    // This can cause eigenvalue computation to fail later
    
    let mut kf = KalmanFilter::<f64>::new(3, 1);
    kf.x = vec![0.0, 0.0, 0.0];
    kf.P = vec![
        1.0, 0.1, 0.2,
        0.1, 1.0, 0.3,
        0.2, 0.3, 1.0,
    ];
    kf.F = vec![
        0.9, 0.1, 0.0,
        0.0, 0.9, 0.1,
        0.0, 0.0, 0.9,
    ];
    kf.H = vec![1.0, 0.0, 0.0];
    kf.Q = vec![
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.01,
    ];
    kf.R = vec![0.1];

    // Run many predict-update cycles
    for i in 0..100 {
        kf.predict();
        kf.update(&vec![i as f64 * 0.1]).unwrap();
    }

    // Check symmetry of final covariance
    for i in 0..3 {
        for j in i+1..3 {
            let diff = (kf.P[i * 3 + j] - kf.P[j * 3 + i]).abs();
            assert!(
                diff < 1e-10,
                "Covariance asymmetry at ({},{}): {} vs {} (diff={})",
                i, j, kf.P[i * 3 + j], kf.P[j * 3 + i], diff
            );
        }
    }
}
