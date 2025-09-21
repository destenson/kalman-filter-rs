// Algorithm validation tests for Kalman filter correctness
use approx::assert_abs_diff_eq;
use kalman_filters::filter::KalmanFilter;

// Test 1: Static system convergence - simplest validation
// A constant value should converge when measured repeatedly
#[test]
fn test_static_convergence() {
    let true_value = 10.0_f64;
    let mut kf = KalmanFilter::new(1, 1);

    // Initial guess (wrong)
    kf.x = vec![0.0];
    kf.P = vec![100.0]; // Large initial uncertainty

    // Static system: x[k+1] = x[k]
    kf.F = vec![1.0];
    kf.H = vec![1.0];
    kf.Q = vec![0.0]; // No process noise (value is constant)
    kf.R = vec![1.0]; // Measurement noise

    // Feed many measurements of the true value
    for _ in 0..50 {
        kf.predict();
        kf.update(&vec![true_value]).unwrap();
    }

    // Should converge to true value
    assert_abs_diff_eq!(kf.x[0], true_value, epsilon = 0.1);

    // Uncertainty should decrease
    assert!(kf.P[0] < 1.0, "Uncertainty didn't decrease: {}", kf.P[0]);
}

// Test 2: Constant velocity model - test dynamics
#[test]
fn test_constant_velocity_tracking() {
    let dt = 0.1_f64;
    let true_velocity = 1.0;

    let mut kf = KalmanFilter::new(2, 1);

    // State: [position, velocity]
    kf.x = vec![0.0, 0.0];
    kf.P = vec![10.0, 0.0, 0.0, 10.0];

    // Constant velocity dynamics
    kf.F = vec![1.0, dt, 0.0, 1.0];

    // Observe position only
    kf.H = vec![1.0, 0.0];

    // Small process noise
    kf.Q = vec![
        0.01 * dt.powi(3) / 3.0,
        0.01 * dt.powi(2) / 2.0,
        0.01 * dt.powi(2) / 2.0,
        0.01 * dt,
    ];

    kf.R = vec![0.1];

    // Track object moving at constant velocity
    for i in 1..=20 {
        kf.predict();

        let true_position = true_velocity * (i as f64) * dt;
        kf.update(&vec![true_position]).unwrap();
    }

    // Should estimate velocity correctly
    assert_abs_diff_eq!(kf.x[1], true_velocity, epsilon = 0.1);
}

// Test 3: Covariance remains positive semi-definite
#[test]
fn test_covariance_positive_definite() {
    let mut kf = KalmanFilter::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];

    kf.F = vec![1.0, 0.1, 0.0, 1.0];

    kf.H = vec![1.0, 0.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.R = vec![1.0];

    for _ in 0..100 {
        kf.predict();
        kf.update(&vec![0.0]).unwrap();

        // Check diagonal elements are positive
        assert!(kf.P[0] > 0.0, "P[0,0] not positive: {}", kf.P[0]);
        assert!(kf.P[3] > 0.0, "P[1,1] not positive: {}", kf.P[3]);

        // Check determinant is positive (for 2x2)
        let det = kf.P[0] * kf.P[3] - kf.P[1] * kf.P[2];
        assert!(det > 0.0, "Covariance determinant not positive: {}", det);

        // Check symmetry
        assert_abs_diff_eq!(kf.P[1], kf.P[2], epsilon = 1e-10);
    }
}

// Test 4: Measurement reduces uncertainty
#[test]
fn test_measurement_reduces_uncertainty() {
    let mut kf = KalmanFilter::new(1, 1);

    kf.x = vec![0.0];
    kf.P = vec![10.0];
    kf.F = vec![1.0];
    kf.H = vec![1.0];
    kf.Q = vec![0.1];
    kf.R = vec![1.0];

    kf.predict();
    let p_after_predict = kf.P[0];

    kf.update(&vec![5.0]).unwrap();
    let p_after_update = kf.P[0];

    // Measurement should reduce uncertainty
    assert!(
        p_after_update < p_after_predict,
        "Measurement didn't reduce uncertainty: {} >= {}",
        p_after_update,
        p_after_predict
    );
}

// Test 5: Steady-state convergence
#[test]
fn test_steady_state() {
    let mut kf = KalmanFilter::new(2, 1);

    kf.x = vec![0.0, 0.0];
    kf.P = vec![100.0, 0.0, 0.0, 100.0];

    kf.F = vec![1.0, 0.1, 0.0, 1.0];

    kf.H = vec![1.0, 0.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.R = vec![1.0];

    let mut prev_p = kf.P.clone();
    let mut converged = false;

    for i in 0..500 {
        kf.predict();
        kf.update(&vec![0.0]).unwrap();

        // Check if covariance has converged
        let max_diff =
            kf.P.iter()
                .zip(prev_p.iter())
                .map(|(a, b): (&f64, &f64)| (a - b).abs())
                .fold(0.0_f64, f64::max);

        if max_diff < 1e-8 {
            converged = true;
            println!("Converged at iteration {}", i);
            break;
        }

        prev_p = kf.P.clone();
    }

    assert!(converged, "Filter didn't reach steady state");
}

// Test 6: Numerical stability with extreme values
#[test]
fn test_numerical_stability() {
    // Test with very small values - but not so small they cause singularity
    let mut kf = KalmanFilter::new(1, 1);
    kf.x = vec![1e-8f64];
    kf.P = vec![1e-10f64]; // Small but not singular
    kf.F = vec![1.0f64];
    kf.H = vec![1.0f64];
    kf.Q = vec![1e-12f64];
    kf.R = vec![1e-10f64]; // Measurement noise can't be too small

    for _ in 0..10 {
        kf.predict();
        let result = kf.update(&vec![1e-8]);

        // It's OK if update fails with singular matrix for extreme values
        // The important thing is it doesn't panic or produce NaN
        if result.is_ok() {
            assert!(!kf.x[0].is_nan(), "State became NaN");
            assert!(!kf.P[0].is_nan(), "Covariance became NaN");
        }
    }

    // Test with very large values
    let mut kf = KalmanFilter::new(1, 1);
    kf.x = vec![1e10f64];
    kf.P = vec![1e10f64];
    kf.F = vec![1.0f64];
    kf.H = vec![1.0f64];
    kf.Q = vec![1.0f64];
    kf.R = vec![1.0f64];

    kf.predict();
    kf.update(&vec![1e10]).unwrap();

    assert!(!kf.x[0].is_infinite(), "State became infinite");
}

// Test 7: Unobservable states don't converge
#[test]
fn test_unobservable_state() {
    let mut kf: KalmanFilter<f64> = KalmanFilter::new(2, 1);

    // Wrong initial guess
    kf.x = vec![10.0, 20.0];
    kf.P = vec![100.0, 0.0, 0.0, 100.0];

    // Identity dynamics
    kf.F = vec![1.0, 0.0, 0.0, 1.0];

    // Only observe first state
    kf.H = vec![1.0, 0.0];

    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.R = vec![0.1];

    let initial_p22 = kf.P[3];

    // Many updates with true state [0, 0]
    for _ in 0..100 {
        kf.predict();
        kf.update(&vec![0.0]).unwrap();
    }

    // First state should converge
    assert_abs_diff_eq!(kf.x[0], 0.0, epsilon = 1.0);

    // Second state should not converge (unobservable)
    assert!(
        kf.x[1].abs() > 5.0,
        "Unobservable state converged: {}",
        kf.x[1]
    );

    // Uncertainty of unobservable state should grow
    assert!(
        kf.P[3] > initial_p22,
        "Unobservable uncertainty didn't grow"
    );
}
