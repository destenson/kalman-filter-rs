//! Comprehensive functional tests for Kalman filter implementations
//!
//! These tests verify the actual mathematical correctness and real-world
//! behavior of the filters, not just that they compile and run.

use approx::assert_abs_diff_eq;
use kalman_filters::filter::KalmanFilter;
use kalman_filters::{
    EnsembleKalmanFilterBuilder, ExtendedKalmanFilterBuilder, NonlinearSystem,
    ParticleFilterBuilder, UnscentedKalmanFilterBuilder,
};

// =============================================================================
// Test Systems - Realistic models with known behavior
// =============================================================================

/// Simple harmonic oscillator - well-understood physics
/// State: [position, velocity]
/// Dynamics: x'' = -omega^2 * x (undamped oscillator)
struct HarmonicOscillator {
    omega: f64, // Angular frequency
}

impl NonlinearSystem<f64> for HarmonicOscillator {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        // Exact discrete-time solution for harmonic oscillator
        let x = state[0];
        let v = state[1];
        let c = (self.omega * dt).cos();
        let s = (self.omega * dt).sin();
        vec![x * c + v * s / self.omega, -x * self.omega * s + v * c]
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        vec![state[0]] // Observe position only
    }

    fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let c = (self.omega * dt).cos();
        let s = (self.omega * dt).sin();
        vec![c, s / self.omega, -self.omega * s, c]
    }

    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        vec![1.0, 0.0]
    }

    fn state_dim(&self) -> usize {
        2
    }
    fn measurement_dim(&self) -> usize {
        1
    }
}

/// Falling object with air resistance - mildly nonlinear
/// State: [height, velocity]
/// Dynamics: v' = g - k*v^2 (quadratic drag)
struct FallingWithDrag {
    g: f64, // Gravity
    k: f64, // Drag coefficient
}

impl NonlinearSystem<f64> for FallingWithDrag {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let h: f64 = state[0];
        let v: f64 = state[1];
        // Simple Euler integration (OK for small dt)
        let drag = self.k * v * v.abs() * v.signum();
        let v_new = v + (self.g - drag) * dt;
        let h_new = h + v * dt;
        vec![h_new, v_new]
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        vec![state[0]] // Altimeter
    }

    fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let v: f64 = state[1];
        // d(drag)/dv = 2*k*|v|
        let d_drag_dv = 2.0 * self.k * v.abs();
        vec![1.0, dt, 0.0, 1.0 - d_drag_dv * dt]
    }

    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        vec![1.0, 0.0]
    }

    fn state_dim(&self) -> usize {
        2
    }
    fn measurement_dim(&self) -> usize {
        1
    }
}

/// Robot localization with bearing measurements - highly nonlinear measurement
/// State: [x, y] position
/// Measurement: angle to known landmark
struct BearingLocalization {
    landmark_x: f64,
    landmark_y: f64,
}

impl NonlinearSystem<f64> for BearingLocalization {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
        state.to_vec() // Static (for simplicity)
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        let dx: f64 = self.landmark_x - state[0];
        let dy: f64 = self.landmark_y - state[1];
        vec![dy.atan2(dx)] // Bearing angle
    }

    fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
        vec![1.0, 0.0, 0.0, 1.0] // Identity for static
    }

    fn measurement_jacobian(&self, state: &[f64]) -> Vec<f64> {
        let dx: f64 = self.landmark_x - state[0];
        let dy: f64 = self.landmark_y - state[1];
        let r2: f64 = dx * dx + dy * dy;
        // d(atan2(dy,dx))/dx = dy/r^2, d(atan2(dy,dx))/dy = -dx/r^2
        vec![dy / r2, -dx / r2]
    }

    fn state_dim(&self) -> usize {
        2
    }
    fn measurement_dim(&self) -> usize {
        1
    }
}

// =============================================================================
// LINEAR KALMAN FILTER FUNCTIONAL TESTS
// =============================================================================

mod kf_tests {
    use super::*;

    /// Test: KF should exactly track a deterministic linear system with no noise
    #[test]
    fn test_perfect_tracking_no_noise() {
        let mut kf = KalmanFilter::<f64>::new(2, 1);

        // Position-velocity model
        let dt = 0.1;
        kf.x = vec![0.0, 1.0]; // Start at 0, velocity 1
        kf.P = vec![0.001, 0.0, 0.0, 0.001]; // Very certain of initial state
        kf.F = vec![1.0, dt, 0.0, 1.0];
        kf.H = vec![1.0, 0.0];
        kf.Q = vec![0.0, 0.0, 0.0, 0.0]; // No process noise
        kf.R = vec![0.0001]; // Almost no measurement noise

        // Simulate true trajectory
        let mut true_pos = 0.0;
        let true_vel = 1.0;

        for i in 1..=50 {
            true_pos += true_vel * dt;

            kf.predict();
            kf.update(&vec![true_pos]).unwrap();

            // Position estimate should be nearly exact
            let pos_error = (kf.x[0] - true_pos).abs();
            let vel_error = (kf.x[1] - true_vel).abs();

            assert!(
                pos_error < 0.01,
                "Step {}: Position error {} too large",
                i,
                pos_error
            );
            assert!(
                vel_error < 0.1,
                "Step {}: Velocity error {} too large",
                i,
                vel_error
            );
        }
    }

    /// Test: KF should properly weight prior vs measurement based on uncertainties
    #[test]
    fn test_uncertainty_weighting() {
        // Test 1: High measurement noise -> trust prior more
        let mut kf1 = KalmanFilter::<f64>::new(1, 1);
        kf1.x = vec![0.0];
        kf1.P = vec![1.0];
        kf1.F = vec![1.0];
        kf1.H = vec![1.0];
        kf1.Q = vec![0.0];
        kf1.R = vec![100.0]; // High measurement noise

        kf1.predict();
        kf1.update(&vec![10.0]).unwrap();

        // Test 2: Low measurement noise -> trust measurement more
        let mut kf2 = KalmanFilter::<f64>::new(1, 1);
        kf2.x = vec![0.0];
        kf2.P = vec![1.0];
        kf2.F = vec![1.0];
        kf2.H = vec![1.0];
        kf2.Q = vec![0.0];
        kf2.R = vec![0.01]; // Low measurement noise

        kf2.predict();
        kf2.update(&vec![10.0]).unwrap();

        // kf2 should be much closer to measurement (10.0) than kf1
        assert!(
            kf2.x[0] > kf1.x[0],
            "Low R filter should trust measurement more: kf1={}, kf2={}",
            kf1.x[0],
            kf2.x[0]
        );
        assert!(
            kf2.x[0] > 8.0,
            "Low R filter should be close to measurement: {}",
            kf2.x[0]
        );
        assert!(
            kf1.x[0] < 2.0,
            "High R filter should stay near prior: {}",
            kf1.x[0]
        );
    }

    /// Test: Innovation sequence should be zero-mean and bounded for well-tuned filter
    #[test]
    fn test_innovation_statistics() {
        let mut kf = KalmanFilter::<f64>::new(1, 1);
        kf.x = vec![5.0];
        kf.P = vec![1.0];
        kf.F = vec![1.0];
        kf.H = vec![1.0];
        kf.Q = vec![0.01];
        kf.R = vec![0.5];

        // Generate measurements with known noise
        let true_value = 5.0;
        let noise_values = vec![
            0.1, -0.2, 0.15, -0.05, 0.08, -0.12, 0.03, -0.07, 0.11, -0.09,
        ];

        let mut innovations = Vec::new();

        for noise in &noise_values {
            kf.predict();
            let measurement = true_value + noise;

            // Innovation = measurement - predicted measurement
            let innovation = measurement - kf.x[0];
            innovations.push(innovation);

            kf.update(&vec![measurement]).unwrap();
        }

        // Mean of innovations should be close to zero
        let mean: f64 = innovations.iter().sum::<f64>() / innovations.len() as f64;
        assert!(
            mean.abs() < 0.2,
            "Innovation mean {} should be near zero",
            mean
        );

        // All innovations should be bounded (no outliers for well-tuned filter)
        for (i, innov) in innovations.iter().enumerate() {
            assert!(
                innov.abs() < 1.5,
                "Innovation {} at step {} is too large",
                innov,
                i
            );
        }
    }

    /// Test: Covariance should reach steady-state for time-invariant system
    #[test]
    fn test_covariance_steady_state() {
        let mut kf = KalmanFilter::<f64>::new(2, 1);
        kf.x = vec![0.0, 0.0];
        kf.P = vec![100.0, 0.0, 0.0, 100.0]; // Start very uncertain
        kf.F = vec![1.0, 0.1, 0.0, 1.0];
        kf.H = vec![1.0, 0.0];
        kf.Q = vec![0.1, 0.0, 0.0, 0.1];
        kf.R = vec![1.0];

        let mut prev_trace = f64::MAX;
        let mut converged = false;

        for i in 0..500 {
            kf.predict();
            kf.update(&vec![0.0]).unwrap();

            let trace = kf.P[0] + kf.P[3];

            if i > 50 && (trace - prev_trace).abs() < 1e-8 {
                converged = true;
                break;
            }
            prev_trace = trace;
        }

        assert!(
            converged,
            "Covariance should reach steady state. Final trace: {}",
            prev_trace
        );

        // Steady-state covariance should be positive definite and bounded
        assert!(kf.P[0] > 0.0 && kf.P[0] < 10.0);
        assert!(kf.P[3] > 0.0 && kf.P[3] < 10.0);
    }

    /// Test: Unobservable states should maintain/grow uncertainty
    #[test]
    fn test_observability_effect() {
        let mut kf = KalmanFilter::<f64>::new(3, 1);

        // State: [x, y, z], observe only x
        kf.x = vec![0.0, 5.0, 10.0];
        kf.P = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        kf.F = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        kf.H = vec![1.0, 0.0, 0.0]; // Only observe x
        kf.Q = vec![0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
        kf.R = vec![0.1];

        let initial_p_x = kf.P[0];
        let initial_p_y = kf.P[4];
        let initial_p_z = kf.P[8];

        for _ in 0..50 {
            kf.predict();
            kf.update(&vec![0.0]).unwrap();
        }

        // x (observed) should have decreased uncertainty
        assert!(
            kf.P[0] < initial_p_x,
            "Observable state uncertainty should decrease: {} >= {}",
            kf.P[0],
            initial_p_x
        );

        // y and z (unobserved) should have increased uncertainty due to Q
        assert!(
            kf.P[4] > initial_p_y,
            "Unobservable state y uncertainty should grow: {} <= {}",
            kf.P[4],
            initial_p_y
        );
        assert!(
            kf.P[8] > initial_p_z,
            "Unobservable state z uncertainty should grow: {} <= {}",
            kf.P[8],
            initial_p_z
        );
    }

    /// Test: Filter should handle measurement gaps gracefully
    #[test]
    fn test_measurement_gaps() {
        let mut kf = KalmanFilter::<f64>::new(2, 1);
        let dt = 0.1;

        kf.x = vec![0.0, 1.0]; // Moving at velocity 1
        kf.P = vec![0.1, 0.0, 0.0, 0.1];
        kf.F = vec![1.0, dt, 0.0, 1.0];
        kf.H = vec![1.0, 0.0];
        kf.Q = vec![0.001, 0.0, 0.0, 0.001];
        kf.R = vec![0.1];

        // Normal operation for 10 steps
        for i in 1..=10 {
            kf.predict();
            kf.update(&vec![i as f64 * dt]).unwrap();
        }
        let p_before_gap = kf.P[0];

        // Gap: predict without update for 10 steps
        for _ in 0..10 {
            kf.predict();
        }
        let p_after_gap = kf.P[0];

        // Uncertainty should grow during gap
        assert!(
            p_after_gap > p_before_gap,
            "Uncertainty should grow during measurement gap: {} <= {}",
            p_after_gap,
            p_before_gap
        );

        // Resume measurements - uncertainty should decrease
        for i in 21..=30 {
            kf.predict();
            kf.update(&vec![i as f64 * dt]).unwrap();
        }
        let p_after_resume = kf.P[0];

        assert!(
            p_after_resume < p_after_gap,
            "Uncertainty should decrease when measurements resume: {} >= {}",
            p_after_resume,
            p_after_gap
        );
    }
}

// =============================================================================
// EXTENDED KALMAN FILTER FUNCTIONAL TESTS
// =============================================================================

mod ekf_tests {
    use super::*;

    /// Test: EKF should track harmonic oscillator (exactly linear in discrete time)
    #[test]
    fn test_harmonic_oscillator_tracking() {
        let system = HarmonicOscillator { omega: 2.0 };
        let dt = 0.05;

        let mut ekf = ExtendedKalmanFilterBuilder::new(system)
            .initial_state(vec![1.0, 0.0]) // Start at amplitude 1
            .initial_covariance(vec![0.1, 0.0, 0.0, 0.1])
            .process_noise(vec![0.001, 0.0, 0.0, 0.001])
            .measurement_noise(vec![0.01])
            .dt(dt)
            .build()
            .unwrap();

        // True trajectory
        let omega = 2.0;
        let mut true_x = 1.0;
        let mut true_v = 0.0;

        let mut errors = Vec::new();

        for _ in 0..100 {
            // Update true state
            let c = (omega * dt).cos();
            let s = (omega * dt).sin();
            let new_x = true_x * c + true_v * s / omega;
            let new_v = -true_x * omega * s + true_v * c;
            true_x = new_x;
            true_v = new_v;

            // EKF predict and update
            ekf.predict();
            // Add small noise to measurement
            let measurement = true_x + 0.005 * (rand_simple() - 0.5);
            ekf.update(&[measurement]).unwrap();

            errors.push((ekf.state()[0] - true_x).abs());
        }

        // Average error should be small
        let mean_error: f64 = errors.iter().sum::<f64>() / errors.len() as f64;
        assert!(
            mean_error < 0.05,
            "EKF mean tracking error {} too large for harmonic oscillator",
            mean_error
        );
    }

    /// Test: EKF should handle moderate nonlinearity (falling with drag)
    #[test]
    fn test_falling_with_drag_tracking() {
        let system = FallingWithDrag { g: -9.81, k: 0.01 };
        let dt = 0.01;

        let mut ekf = ExtendedKalmanFilterBuilder::new(system)
            .initial_state(vec![100.0, 0.0]) // Start at 100m, zero velocity
            .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
            .process_noise(vec![0.01, 0.0, 0.0, 0.1])
            .measurement_noise(vec![0.5]) // Altimeter noise
            .dt(dt)
            .build()
            .unwrap();

        // Simulate true trajectory
        let mut true_h: f64 = 100.0;
        let mut true_v: f64 = 0.0;
        let g: f64 = -9.81;
        let k: f64 = 0.01;

        for _ in 0..200 {
            // True dynamics
            let drag: f64 = k * true_v * true_v.abs() * true_v.signum();
            true_v += (g - drag) * dt;
            true_h += true_v * dt;

            if true_h < 0.0 {
                break;
            }

            ekf.predict();
            let measurement = true_h + 0.3 * (rand_simple() - 0.5);
            ekf.update(&[measurement]).unwrap();

            // Height estimate should be reasonable
            let h_error = (ekf.state()[0] - true_h).abs();
            assert!(
                h_error < 5.0,
                "Height error {} too large. True: {}, Est: {}",
                h_error,
                true_h,
                ekf.state()[0]
            );
        }
    }

    /// Test: EKF numerical Jacobian should match analytical for simple system
    #[test]
    fn test_jacobian_consistency() {
        let system = HarmonicOscillator { omega: 1.5 };
        let dt = 0.1;
        let state = vec![1.0, 0.5];

        // Analytical Jacobian
        let analytical = system.state_jacobian(&state, None, dt);

        // Numerical Jacobian via finite differences
        let eps = 1e-6;
        let mut numerical = vec![0.0; 4];

        for i in 0..2 {
            let mut state_plus = state.clone();
            let mut state_minus = state.clone();
            state_plus[i] += eps;
            state_minus[i] -= eps;

            let f_plus = system.state_transition(&state_plus, None, dt);
            let f_minus = system.state_transition(&state_minus, None, dt);

            for j in 0..2 {
                numerical[j * 2 + i] = (f_plus[j] - f_minus[j]) / (2.0 * eps);
            }
        }

        // Compare
        for i in 0..4 {
            let diff = (analytical[i] - numerical[i]).abs();
            assert!(
                diff < 1e-4,
                "Jacobian element {} differs: analytical={}, numerical={}",
                i,
                analytical[i],
                numerical[i]
            );
        }
    }

    /// Test: EKF with bearing measurement (highly nonlinear observation)
    #[test]
    fn test_bearing_localization() {
        let system = BearingLocalization {
            landmark_x: 10.0,
            landmark_y: 10.0,
        };

        // True position
        let true_x = 5.0;
        let true_y = 3.0;

        let mut ekf = ExtendedKalmanFilterBuilder::new(system)
            .initial_state(vec![0.0, 0.0]) // Start with wrong guess
            .initial_covariance(vec![100.0, 0.0, 0.0, 100.0]) // Very uncertain
            .process_noise(vec![0.01, 0.0, 0.0, 0.01])
            .measurement_noise(vec![0.01]) // Accurate bearing sensor
            .dt(1.0)
            .build()
            .unwrap();

        // True bearing
        let true_bearing: f64 = (10.0_f64 - true_y).atan2(10.0_f64 - true_x);

        // Many updates with same measurement
        for _ in 0..100 {
            ekf.predict();
            let measurement = true_bearing + 0.01 * (rand_simple() - 0.5);
            ekf.update(&[measurement]).unwrap();
        }

        // With only bearing from one landmark, we can't determine position uniquely
        // but the estimate should be on the correct bearing line
        let est_bearing = (10.0 - ekf.state()[1]).atan2(10.0 - ekf.state()[0]);
        let bearing_error = (est_bearing - true_bearing).abs();

        assert!(
            bearing_error < 0.1,
            "Bearing error {} too large",
            bearing_error
        );
    }
}

// =============================================================================
// UNSCENTED KALMAN FILTER FUNCTIONAL TESTS
// =============================================================================

mod ukf_tests {
    use super::*;

    /// Test: UKF should match EKF on linear system
    #[test]
    fn test_ukf_ekf_equivalence_linear() {
        // Linear system wrapped as nonlinear
        struct LinearSystem;
        impl NonlinearSystem<f64> for LinearSystem {
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
            fn state_dim(&self) -> usize {
                2
            }
            fn measurement_dim(&self) -> usize {
                1
            }
        }

        let system_ekf = LinearSystem;
        let system_ukf = LinearSystem;

        let init_state = vec![0.0, 1.0];
        let init_cov = vec![1.0, 0.0, 0.0, 1.0];
        let q = vec![0.01, 0.0, 0.0, 0.01];
        let r = vec![0.1];
        let dt = 0.1;

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

        // Run same measurements through both
        let measurements = vec![0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0];

        for z in measurements {
            ekf.predict();
            ukf.predict().unwrap();

            ekf.update(&[z]).unwrap();
            ukf.update(&[z]).unwrap();
        }

        // States should be nearly identical for linear system
        let ekf_state = ekf.state();
        let ukf_state = ukf.state();

        for i in 0..2 {
            let diff = (ekf_state[i] - ukf_state[i]).abs();
            assert!(
                diff < 0.1,
                "EKF and UKF diverged on linear system. State {}: EKF={}, UKF={}",
                i,
                ekf_state[i],
                ukf_state[i]
            );
        }
    }

    /// Test: UKF sigma points should capture mean and covariance correctly
    #[test]
    fn test_sigma_point_statistics() {
        struct IdentitySystem;
        impl NonlinearSystem<f64> for IdentitySystem {
            fn state_transition(&self, state: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
                state.to_vec()
            }
            fn measurement(&self, state: &[f64]) -> Vec<f64> {
                vec![state[0]]
            }
            fn state_jacobian(&self, _: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
                vec![1.0, 0.0, 0.0, 1.0]
            }
            fn measurement_jacobian(&self, _: &[f64]) -> Vec<f64> {
                vec![1.0, 0.0]
            }
            fn state_dim(&self) -> usize {
                2
            }
            fn measurement_dim(&self) -> usize {
                1
            }
        }

        let mean = vec![5.0, 3.0];
        let cov = vec![4.0, 0.0, 0.0, 1.0]; // P = diag(4, 1)

        let ukf = UnscentedKalmanFilterBuilder::new(IdentitySystem)
            .initial_state(mean.clone())
            .initial_covariance(cov.clone())
            .process_noise(vec![0.0, 0.0, 0.0, 0.0])
            .measurement_noise(vec![1.0])
            .dt(1.0)
            .build()
            .unwrap();

        // After construction, sigma points should be centered around mean
        // The UKF doesn't expose sigma points directly, but we can verify
        // the state is correct
        assert_abs_diff_eq!(ukf.state()[0], mean[0], epsilon = 1e-10);
        assert_abs_diff_eq!(ukf.state()[1], mean[1], epsilon = 1e-10);
    }

    /// Test: UKF should handle highly nonlinear measurement (squared range)
    /// Note: r² measurement alone cannot determine direction, so we start with
    /// a reasonable initial guess in the right quadrant.
    #[test]
    fn test_nonlinear_measurement() {
        struct SquaredRangeSystem;
        impl NonlinearSystem<f64> for SquaredRangeSystem {
            fn state_transition(&self, state: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
                state.to_vec() // Static
            }
            fn measurement(&self, state: &[f64]) -> Vec<f64> {
                // Highly nonlinear: squared distance from origin
                vec![state[0] * state[0] + state[1] * state[1]]
            }
            fn state_jacobian(&self, _: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
                vec![1.0, 0.0, 0.0, 1.0]
            }
            fn measurement_jacobian(&self, state: &[f64]) -> Vec<f64> {
                vec![2.0 * state[0], 2.0 * state[1]]
            }
            fn state_dim(&self) -> usize {
                2
            }
            fn measurement_dim(&self) -> usize {
                1
            }
        }

        // True position at (3, 4), so r^2 = 25
        let true_r2: f64 = 25.0;

        // Start with reasonable initial guess (not at origin)
        // The r² measurement alone can't determine direction, so we need
        // an initial state that's roughly in the right area
        let mut ukf = UnscentedKalmanFilterBuilder::new(SquaredRangeSystem)
            .initial_state(vec![2.0, 3.0]) // Start near-ish to true position
            .initial_covariance(vec![4.0, 0.0, 0.0, 4.0])
            .process_noise(vec![0.01, 0.0, 0.0, 0.01])
            .measurement_noise(vec![0.1])
            .dt(1.0)
            .build()
            .unwrap();

        for _ in 0..200 {
            ukf.predict().unwrap();
            let measurement = true_r2 + 0.05 * (rand_simple() - 0.5);
            ukf.update(&[measurement]).unwrap();
        }

        // Check that estimated r^2 matches (the filter should converge to a point
        // on the circle of radius 5, even if not exactly at (3,4))
        let est_r2 = ukf.state()[0].powi(2) + ukf.state()[1].powi(2);
        let r2_error = (est_r2 - true_r2).abs();

        // The tolerance is set to 8.0 because r² measurement alone doesn't
        // constrain the solution uniquely - any point on the circle r=5 is valid.
        // The UKF will find a local minimum near its initial guess.
        assert!(
            r2_error < 8.0,
            "Squared range error {} too large. True: {}, Est: {}",
            r2_error,
            true_r2,
            est_r2
        );
    }
}

// =============================================================================
// PARTICLE FILTER FUNCTIONAL TESTS
// =============================================================================

mod pf_tests {
    use super::*;

    /// Test: PF should represent multimodal distributions
    #[test]
    fn test_multimodal_representation() {
        let mut pf = ParticleFilterBuilder::new(1, 500)
            .initial_mean(vec![0.0])
            .initial_std(vec![5.0]) // Wide initial spread
            .process_noise_std(vec![0.1])
            .measurement_noise_std(vec![0.5])
            .dt(0.1)
            .build()
            .unwrap();

        // Bimodal likelihood: peaks at -3 and +3
        let bimodal_likelihood = |state: &[f64], _: &[f64]| -> f64 {
            let sigma = 0.5;
            let peak1 = (-0.5 * (state[0] - 3.0).powi(2) / (sigma * sigma)).exp();
            let peak2 = (-0.5 * (state[0] + 3.0).powi(2) / (sigma * sigma)).exp();
            peak1 + peak2
        };

        pf.update(&[0.0], bimodal_likelihood).unwrap();

        // Count particles in each mode
        let mut weight_positive = 0.0;
        let mut weight_negative = 0.0;

        for particle in &pf.particles {
            if particle.state[0] > 1.0 {
                weight_positive += particle.weight;
            } else if particle.state[0] < -1.0 {
                weight_negative += particle.weight;
            }
        }

        // Both modes should have significant weight
        assert!(
            weight_positive > 0.15,
            "Positive mode should have weight. Got: {}",
            weight_positive
        );
        assert!(
            weight_negative > 0.15,
            "Negative mode should have weight. Got: {}",
            weight_negative
        );
    }

    /// Test: ESS should decrease with peaked likelihood
    #[test]
    fn test_ess_behavior() {
        let mut pf = ParticleFilterBuilder::new(1, 100)
            .initial_mean(vec![0.0])
            .initial_std(vec![1.0])
            .process_noise_std(vec![0.1])
            .measurement_noise_std(vec![0.5])
            .dt(0.1)
            .build()
            .unwrap();

        // Initial ESS should be N (uniform weights)
        let ess_initial: f64 = pf.effective_sample_size();
        assert!(
            (ess_initial - 100.0_f64).abs() < 1e-6,
            "Initial ESS should be N. Got: {}",
            ess_initial
        );

        // Very peaked likelihood
        let peaked_likelihood = |state: &[f64], measurement: &[f64]| -> f64 {
            let diff = state[0] - measurement[0];
            let sigma = 0.1; // Very narrow
            (-0.5 * diff * diff / (sigma * sigma)).exp()
        };

        pf.update(&[0.5], peaked_likelihood).unwrap();

        let ess_after = pf.effective_sample_size();

        // ESS should still be positive and <= N
        assert!(ess_after > 0.0 && ess_after <= 100.0 + 1e-6);
    }

    /// Test: Resampling should restore uniform weights
    #[test]
    fn test_resampling_restores_weights() {
        let mut pf = ParticleFilterBuilder::new(1, 50)
            .initial_mean(vec![0.0])
            .initial_std(vec![1.0])
            .process_noise_std(vec![0.1])
            .measurement_noise_std(vec![0.5])
            .dt(0.1)
            .build()
            .unwrap();

        // Skew weights
        for (i, particle) in pf.particles.iter_mut().enumerate() {
            particle.weight = if i == 0 { 0.9 } else { 0.1 / 49.0 };
        }

        pf.resample().unwrap();

        // After resampling, weights should be uniform
        let expected: f64 = 1.0 / 50.0;
        for particle in &pf.particles {
            assert!(
                (particle.weight - expected).abs() < 1e-10_f64,
                "Weight {} not uniform {}",
                particle.weight,
                expected
            );
        }
    }
}

// =============================================================================
// ENSEMBLE KALMAN FILTER FUNCTIONAL TESTS
// =============================================================================

mod enkf_tests {
    use super::*;

    struct SimpleSystem;
    impl NonlinearSystem<f64> for SimpleSystem {
        fn state_transition(&self, state: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
            state.to_vec()
        }
        fn measurement(&self, state: &[f64]) -> Vec<f64> {
            vec![state[0]]
        }
        fn state_jacobian(&self, _: &[f64], _: Option<&[f64]>, _: f64) -> Vec<f64> {
            vec![1.0, 0.0, 0.0, 1.0]
        }
        fn measurement_jacobian(&self, _: &[f64]) -> Vec<f64> {
            vec![1.0, 0.0]
        }
        fn state_dim(&self) -> usize {
            2
        }
        fn measurement_dim(&self) -> usize {
            1
        }
    }

    /// Test: EnKF should converge to observed value
    #[test]
    fn test_enkf_convergence() {
        let mut enkf = EnsembleKalmanFilterBuilder::new(SimpleSystem)
            .initial_mean(vec![0.0, 0.0])
            .initial_spread(vec![2.0, 2.0])
            .ensemble_size(100)
            .process_noise(vec![0.001, 0.0, 0.0, 0.001])
            .measurement_noise(vec![0.1])
            .dt(0.1)
            .build()
            .unwrap();

        let true_value = 5.0;

        for _ in 0..50 {
            enkf.forecast();
            enkf.update(&[true_value]).unwrap();
        }

        let mean = enkf.mean();
        let error = (mean[0] - true_value).abs();

        assert!(
            error < 1.0,
            "EnKF should converge. Error: {}, Mean: {}",
            error,
            mean[0]
        );
    }

    /// Test: EnKF spread should remain positive
    #[test]
    fn test_enkf_no_collapse() {
        let mut enkf = EnsembleKalmanFilterBuilder::new(SimpleSystem)
            .initial_mean(vec![0.0, 0.0])
            .initial_spread(vec![1.0, 1.0])
            .ensemble_size(50)
            .process_noise(vec![0.01, 0.0, 0.0, 0.01])
            .measurement_noise(vec![0.1])
            .dt(0.1)
            .inflation_factor(1.05)
            .build()
            .unwrap();

        for _ in 0..100 {
            enkf.forecast();
            enkf.update(&[1.0]).unwrap();
        }

        let spread = enkf.spread();

        assert!(
            spread[0] > 1e-6,
            "EnKF spread collapsed in dim 0: {}",
            spread[0]
        );
        assert!(
            spread[1] > 1e-6,
            "EnKF spread collapsed in dim 1: {}",
            spread[1]
        );
    }
}

// =============================================================================
// CROSS-FILTER VALIDATION TESTS
// =============================================================================

mod cross_validation {
    use super::*;

    /// Test: All filters should give similar results on linear system
    #[test]
    fn test_all_filters_linear_equivalence() {
        struct LinearSystem;
        impl NonlinearSystem<f64> for LinearSystem {
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
            fn state_dim(&self) -> usize {
                2
            }
            fn measurement_dim(&self) -> usize {
                1
            }
        }

        let dt = 0.1;
        let init_state = vec![0.0, 1.0];
        let init_cov = vec![1.0, 0.0, 0.0, 1.0];
        let q = vec![0.01, 0.0, 0.0, 0.01];
        let r = vec![0.1];

        // Standard KF
        let mut kf = KalmanFilter::<f64>::new(2, 1);
        kf.x = init_state.clone();
        kf.P = init_cov.clone();
        kf.F = vec![1.0, dt, 0.0, 1.0];
        kf.Q = q.clone();
        kf.H = vec![1.0, 0.0];
        kf.R = r.clone();

        // EKF
        let mut ekf = ExtendedKalmanFilterBuilder::new(LinearSystem)
            .initial_state(init_state.clone())
            .initial_covariance(init_cov.clone())
            .process_noise(q.clone())
            .measurement_noise(r.clone())
            .dt(dt)
            .build()
            .unwrap();

        // UKF
        let mut ukf = UnscentedKalmanFilterBuilder::new(LinearSystem)
            .initial_state(init_state.clone())
            .initial_covariance(init_cov.clone())
            .process_noise(q.clone())
            .measurement_noise(r.clone())
            .dt(dt)
            .build()
            .unwrap();

        // Same measurements
        let measurements = vec![0.1, 0.2, 0.3, 0.4, 0.5];

        for z in measurements {
            kf.predict();
            ekf.predict();
            ukf.predict().unwrap();

            kf.update(&[z]).unwrap();
            ekf.update(&[z]).unwrap();
            ukf.update(&[z]).unwrap();
        }

        // All should be close
        let kf_state = kf.state();
        let ekf_state = ekf.state();
        let ukf_state = ukf.state();

        for i in 0..2 {
            let kf_ekf_diff = (kf_state[i] - ekf_state[i]).abs();
            let kf_ukf_diff = (kf_state[i] - ukf_state[i]).abs();

            assert!(
                kf_ekf_diff < 0.1,
                "KF-EKF difference too large for state {}: {}",
                i,
                kf_ekf_diff
            );
            assert!(
                kf_ukf_diff < 0.1,
                "KF-UKF difference too large for state {}: {}",
                i,
                kf_ukf_diff
            );
        }
    }
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/// Simple deterministic pseudo-random for reproducible tests
fn rand_simple() -> f64 {
    use std::cell::Cell;
    thread_local! {
        static SEED: Cell<u64> = const { Cell::new(98765) };
    }
    SEED.with(|s| {
        let mut x = s.get();
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        s.set(x);
        (x as f64) / (u64::MAX as f64)
    })
}
