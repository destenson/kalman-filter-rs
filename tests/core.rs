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
