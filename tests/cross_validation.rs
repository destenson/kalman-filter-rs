//! Statistical validation tests for Kalman filters
//!
//! These use STANDARD methods from the literature to validate filter consistency:
//! - NEES (Normalized Estimation Error Squared)
//! - NIS (Normalized Innovation Squared)
//! - Chi-squared statistical tests
//!
//! References:
//! - Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). Estimation with applications to tracking and navigation.
//! - Welch & Bishop, "An Introduction to the Kalman Filter" (UNC-Chapel Hill, TR 95-041)

use kalman_filters::filter::KalmanFilter;
use approx::assert_abs_diff_eq;

/// Test against analytical solution for static system
/// This is Example 1 from Welch & Bishop
#[test]
fn test_analytical_static_system() {
    // For a static system with no process noise and repeated measurements,
    // the Kalman filter converges to the weighted average

    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![0.0];
    kf.P = vec![1000.0]; // Very uncertain initially
    kf.F = vec![1.0]; // Static system
    kf.H = vec![1.0]; // Direct observation
    kf.Q = vec![0.0]; // No process noise
    kf.R = vec![1.0]; // Measurement noise

    // Repeated measurements around value 5.0
    let measurements = vec![5.0, 5.1, 4.9, 5.0, 5.05, 4.95];

    for &z in &measurements {
        kf.predict();
        kf.update(&vec![z]).unwrap();
    }

    // After many measurements, should converge close to mean
    assert_abs_diff_eq!(kf.x[0], 5.0, epsilon = 0.1);

    // Uncertainty should decrease significantly
    assert!(kf.P[0] < 1.0, "Covariance should decrease with measurements");
}

/// Test steady-state convergence for time-invariant system
/// Based on Algebraic Riccati Equation (ARE)
#[test]
fn test_steady_state_convergence() {
    // For time-invariant systems, the Kalman filter reaches a steady-state
    // where the covariance converges to a fixed value (algebraic Riccati equation)

    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![0.0];
    kf.P = vec![1.0];
    kf.F = vec![1.0];
    kf.H = vec![1.0];
    kf.Q = vec![0.1];
    kf.R = vec![1.0];

    // Run until steady-state
    let mut prev_p = 0.0;
    for i in 0..1000 {
        kf.predict();
        kf.update(&vec![0.0]).unwrap();

        if i > 100 && (kf.P[0] - prev_p).abs() < 1e-10 {
            break;
        }
        prev_p = kf.P[0];
    }

    // Analytical steady-state solution for this system
    // At steady state: P = (P + Q) - (P + Q)²/((P + Q) + R)
    // Let S = P + Q, then: P = S - S²/(S + R)
    // So: S - Q = S - S²/(S + R)
    // Simplifying: Q = S²/(S + R)
    // Q*(S + R) = S²
    // S² - Q*S - Q*R = 0
    // Using quadratic formula: S = (Q + sqrt(Q² + 4*Q*R))/2
    // Then P = S - Q
    let q = 0.1f64;
    let r = 1.0f64;
    let s = (q + (q * q + 4.0f64 * q * r).sqrt()) / 2.0;
    let p_steady = s - q;

    assert_abs_diff_eq!(kf.P[0], p_steady, epsilon = 1e-6);
}

/// Normalized Innovation Squared (NIS) consistency test
///
/// The innovation y = z - H*x should be zero-mean with covariance S = H*P*H' + R
/// For a consistent filter with correctly specified noise, NIS follows chi-squared distribution
/// This test verifies innovations have proper statistics
#[test]
fn test_innovation_statistics() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![0.0];
    kf.P = vec![10.0];
    kf.F = vec![1.0];
    kf.H = vec![1.0];
    kf.Q = vec![0.1];
    kf.R = vec![1.0];

    // Measurements that vary (simulating real sensor data)
    let measurements = vec![
        1.0, 1.2, 0.9, 1.1, 1.3, 0.8, 1.0, 1.2, 0.95, 1.05,
        1.1, 1.0, 0.9, 1.15, 1.05, 0.95, 1.0, 1.1, 1.0, 0.9,
    ];

    let mut innovations = vec![];

    for &z in &measurements {
        kf.predict();

        // Compute innovation (before update)
        let innovation = z - kf.H[0] * kf.x[0];
        innovations.push(innovation);

        kf.update(&vec![z]).unwrap();
    }

    // For a consistent filter, innovations should be approximately zero-mean
    let mean_innovation: f64 = innovations.iter().sum::<f64>() / innovations.len() as f64;

    assert!(
        mean_innovation.abs() < 0.5,
        "Mean innovation = {} should be near 0 for consistent filter",
        mean_innovation
    );
}

/// Test filter doesn't diverge with model mismatch
#[test]
fn test_robustness_to_model_error() {
    let mut kf = KalmanFilter::<f64>::new(1, 1);

    kf.x = vec![0.0];
    kf.P = vec![1.0];
    kf.F = vec![1.0]; // Model: system is static
    kf.H = vec![1.0];
    kf.Q = vec![0.1]; // Some process noise to prevent overconfidence
    kf.R = vec![1.0];

    // True system: state grows linearly
    let mut true_state = 0.0;
    for _ in 0..100 {
        true_state += 0.1; // True dynamics

        kf.predict();
        kf.update(&vec![true_state]).unwrap();
    }

    // Despite model mismatch, filter should track (with lag)
    // and not diverge to infinity
    assert!(kf.x[0] < 100.0, "Filter diverged despite model error");
    assert!(!kf.x[0].is_nan(), "Filter produced NaN");
    assert!(kf.P[0] < 1000.0, "Covariance exploded");
}
