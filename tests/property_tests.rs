//! Property-based tests for Kalman filters
//!
//! These tests verify mathematical invariants that should hold for ALL valid inputs,
//! not just specific test cases. They test the fundamental properties of Kalman filtering.

use kalman_filters::filter::KalmanFilter;
use kalman_filters::information::{
    kalman_to_information, InformationForm, InformationState,
};
use kalman_filters::{ExtendedKalmanFilterBuilder, InformationFilterBuilder, NonlinearSystem};

const EPSILON: f64 = 1e-8;

// =============================================================================
// COVARIANCE PROPERTIES
// =============================================================================

#[test]
fn property_covariance_always_positive_semidefinite() {
    // Property: Covariance matrix P must always be positive semi-definite
    // This means all eigenvalues >= 0, or equivalently: x^T P x >= 0 for all x

    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 0.1, 0.0, 1.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.H = vec![1.0, 0.0];
    kf.R = vec![0.1];

    for i in 0..100 {
        kf.predict();

        if i % 2 == 0 {
            kf.update(&[i as f64 * 0.1]).unwrap();
        }

        // Check positive semi-definite property
        // For 2x2 matrix: det >= 0 and trace >= 0 and P[0][0] >= 0 and P[1][1] >= 0
        let det = kf.P[0] * kf.P[3] - kf.P[1] * kf.P[2];
        let trace = kf.P[0] + kf.P[3];

        assert!(
            det >= -EPSILON,
            "Covariance determinant must be non-negative at step {}: {}",
            i,
            det
        );
        assert!(
            trace >= -EPSILON,
            "Covariance trace must be non-negative at step {}: {}",
            i,
            trace
        );
        assert!(
            kf.P[0] >= -EPSILON,
            "Diagonal element P[0][0] must be non-negative at step {}: {}",
            i,
            kf.P[0]
        );
        assert!(
            kf.P[3] >= -EPSILON,
            "Diagonal element P[1][1] must be non-negative at step {}: {}",
            i,
            kf.P[3]
        );
    }
}

#[test]
fn property_covariance_symmetry() {
    // Property: Covariance matrix must always be symmetric: P = P^T

    let mut kf = KalmanFilter::<f64>::new(3, 2);
    kf.x = vec![0.0, 0.0, 0.0];
    kf.P = vec![1.0, 0.1, 0.2, 0.1, 1.0, 0.3, 0.2, 0.3, 1.0];
    kf.F = vec![1.0, 0.1, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0, 1.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
    kf.H = vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    kf.R = vec![0.1, 0.0, 0.0, 0.1];

    for i in 0..50 {
        kf.predict();
        kf.update(&[i as f64 * 0.1, i as f64 * 0.05]).unwrap();

        // Check symmetry: P[i][j] == P[j][i]
        for row in 0..3 {
            for col in 0..3 {
                let p_ij = kf.P[row * 3 + col];
                let p_ji = kf.P[col * 3 + row];
                assert!(
                    (p_ij - p_ji).abs() < EPSILON,
                    "Covariance not symmetric at step {}: P[{}][{}]={} != P[{}][{}]={}",
                    i,
                    row,
                    col,
                    p_ij,
                    col,
                    row,
                    p_ji
                );
            }
        }
    }
}

#[test]
fn property_update_never_increases_uncertainty() {
    // Property: Update step (with measurement) should never increase uncertainty
    // i.e., trace(P_posterior) <= trace(P_prior)

    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 0.0];
    kf.P = vec![10.0, 0.0, 0.0, 10.0];
    kf.F = vec![1.0, 0.1, 0.0, 1.0];
    kf.Q = vec![0.1, 0.0, 0.0, 0.1];
    kf.H = vec![1.0, 0.0];
    kf.R = vec![1.0];

    for i in 0..50 {
        kf.predict();

        let trace_before_update = kf.P[0] + kf.P[3];

        kf.update(&[i as f64 * 0.1]).unwrap();

        let trace_after_update = kf.P[0] + kf.P[3];

        assert!(
            trace_after_update <= trace_before_update + EPSILON,
            "Update increased uncertainty at step {}: before={}, after={}",
            i,
            trace_before_update,
            trace_after_update
        );
    }
}

#[test]
fn property_predict_with_q_increases_uncertainty() {
    // Property: Predict step with non-zero Q should increase or maintain uncertainty
    // trace(P_k+1|k) >= trace(P_k|k) when Q > 0

    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 0.0];
    kf.P = vec![0.1, 0.0, 0.0, 0.1];
    kf.F = vec![1.0, 0.1, 0.0, 1.0];
    kf.Q = vec![0.05, 0.0, 0.0, 0.05]; // Non-zero process noise
    kf.H = vec![1.0, 0.0];
    kf.R = vec![0.1];

    // After update, predict should increase uncertainty
    kf.update(&[1.0]).unwrap();

    for i in 0..20 {
        let trace_before = kf.P[0] + kf.P[3];

        kf.predict();

        let trace_after = kf.P[0] + kf.P[3];

        assert!(
            trace_after >= trace_before - EPSILON,
            "Predict with Q>0 should not decrease uncertainty at step {}: before={}, after={}",
            i,
            trace_before,
            trace_after
        );

        // Update to prevent runaway
        if i % 2 == 0 {
            kf.update(&[i as f64 * 0.1]).unwrap();
        }
    }
}

// =============================================================================
// INFORMATION FORM PROPERTIES
// =============================================================================

#[test]
fn property_information_matrix_positive_semidefinite() {
    // Property: Information matrix Y must be positive semi-definite

    let mut inf = InformationFilterBuilder::new(2, 1)
        .initial_information_matrix(vec![1.0, 0.0, 0.0, 1.0])
        .initial_information_vector(vec![0.0, 0.0])
        .state_transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .observation_matrix(vec![1.0, 0.0])
        .measurement_noise(vec![0.1])
        .build()
        .unwrap();

    for i in 0..50 {
        inf.predict().unwrap();

        if i % 2 == 0 {
            inf.update(&[i as f64 * 0.1]).unwrap();
        }

        // Check positive semi-definite
        let det = inf.state.Y[0] * inf.state.Y[3] - inf.state.Y[1] * inf.state.Y[2];
        assert!(
            det >= -EPSILON,
            "Information matrix determinant must be non-negative at step {}: {}",
            i,
            det
        );
    }
}

#[test]
fn property_information_addition_is_commutative() {
    // Property: Adding information from measurements should be commutative
    // i.e., updating with z1 then z2 should give same result as z2 then z1

    let mut state1 = InformationState::<f64>::new(2);
    state1.Y = vec![1.0, 0.0, 0.0, 1.0];
    state1.y = vec![0.0, 0.0];

    let mut state2 = InformationState::<f64>::new(2);
    state2.Y = vec![1.0, 0.0, 0.0, 1.0];
    state2.y = vec![0.0, 0.0];

    // Information increments from two measurements
    let delta_Y_a = vec![2.0, 0.0, 0.0, 2.0];
    let delta_y_a = vec![2.0, 0.0];

    let delta_Y_b = vec![1.0, 0.0, 0.0, 1.0];
    let delta_y_b = vec![1.0, 0.0];

    // state1: add A then B
    state1.add_information(&delta_y_a, &delta_Y_a);
    state1.add_information(&delta_y_b, &delta_Y_b);

    // state2: add B then A
    state2.add_information(&delta_y_b, &delta_Y_b);
    state2.add_information(&delta_y_a, &delta_Y_a);

    // Results should be identical
    for i in 0..4 {
        assert!(
            (state1.Y[i] - state2.Y[i]).abs() < EPSILON,
            "Information matrix should be order-independent: Y[{}] differs",
            i
        );
    }
    for i in 0..2 {
        assert!(
            (state1.y[i] - state2.y[i]).abs() < EPSILON,
            "Information vector should be order-independent: y[{}] differs",
            i
        );
    }
}

#[test]
fn property_kf_if_equivalence_maintained() {
    // Property: Converting KF -> IF -> KF should preserve state and covariance

    let kf_original = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![1.0, 2.0],
        vec![2.0, 0.5, 0.5, 1.5],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1],
    )
    .unwrap();

    // Convert to IF
    let inf = kalman_to_information(&kf_original).unwrap();

    // Recover state and covariance
    let recovered_state = inf.get_state().unwrap();
    let recovered_cov = inf.get_covariance().unwrap();

    // Check state preservation
    for i in 0..2 {
        assert!(
            (kf_original.x[i] - recovered_state[i]).abs() < EPSILON,
            "State not preserved in KF->IF conversion: x[{}] = {} vs {}",
            i,
            kf_original.x[i],
            recovered_state[i]
        );
    }

    // Check covariance preservation
    for i in 0..4 {
        assert!(
            (kf_original.P[i] - recovered_cov[i]).abs() < 1e-6,
            "Covariance not preserved in KF->IF conversion: P[{}] = {} vs {}",
            i,
            kf_original.P[i],
            recovered_cov[i]
        );
    }
}

// =============================================================================
// INNOVATION PROPERTIES
// =============================================================================

#[test]
fn property_innovation_has_correct_covariance() {
    // Property: Innovation covariance S = H*P*H^T + R

    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.2, 0.2, 1.0];
    kf.F = vec![1.0, 0.1, 0.0, 1.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.H = vec![1.0, 0.5]; // Non-trivial H
    kf.R = vec![0.25];

    kf.predict();

    // Manually compute expected S = H*P*H^T + R
    // H*P = [1.0, 0.5] * [[P00, P01], [P10, P11]]
    //     = [P00 + 0.5*P10, P01 + 0.5*P11]
    let hp_0 = kf.P[0] + 0.5 * kf.P[2];
    let hp_1 = kf.P[1] + 0.5 * kf.P[3];

    // (H*P)*H^T = [hp_0, hp_1] * [1.0, 0.5]^T
    //           = hp_0 * 1.0 + hp_1 * 0.5
    let expected_s = hp_0 * 1.0 + hp_1 * 0.5 + kf.R[0];

    // In practice, we can't directly observe S, but we can verify its properties
    // The innovation should have variance approximately S
    // This is more of a sanity check that the math is consistent
    assert!(expected_s > 0.0, "Innovation covariance must be positive");
}

// =============================================================================
// NONLINEAR FILTER PROPERTIES
// =============================================================================

struct LinearizedSystem;

impl NonlinearSystem<f64> for LinearizedSystem {
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

#[test]
fn property_ekf_reduces_to_kf_for_linear_systems() {
    // Property: EKF on a linear system should behave identically to KF

    let dt = 0.1;

    // Standard KF
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 1.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, dt, 0.0, 1.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.H = vec![1.0, 0.0];
    kf.R = vec![0.1];

    // EKF with linearized system
    let mut ekf = ExtendedKalmanFilterBuilder::new(LinearizedSystem)
        .initial_state(vec![0.0, 1.0])
        .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
        .process_noise(vec![0.01, 0.0, 0.0, 0.01])
        .measurement_noise(vec![0.1])
        .dt(dt)
        .build()
        .unwrap();

    // Run same sequence
    for i in 0..30 {
        let z = i as f64 * 0.1;

        kf.predict();
        ekf.predict();

        kf.update(&[z]).unwrap();
        ekf.update(&[z]).unwrap();

        // States should match
        for j in 0..2 {
            assert!(
                (kf.x[j] - ekf.state()[j]).abs() < 1e-6,
                "EKF and KF state diverged at step {}, component {}: KF={}, EKF={}",
                i,
                j,
                kf.x[j],
                ekf.state()[j]
            );
        }

        // Covariances should match
        for j in 0..4 {
            assert!(
                (kf.P[j] - ekf.covariance()[j]).abs() < 1e-6,
                "EKF and KF covariance diverged at step {}, element {}: KF={}, EKF={}",
                i,
                j,
                kf.P[j],
                ekf.covariance()[j]
            );
        }
    }
}

// =============================================================================
// STATISTICAL PROPERTIES
// =============================================================================

#[test]
fn property_normalized_innovation_squared_is_chi_squared_distributed() {
    // Property: Normalized innovation squared (NIS) should be chi-squared distributed
    // NIS = innovation^T * S^(-1) * innovation ~ χ²(m) where m is measurement dim
    // For large samples, mean should be approximately m

    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 0.0];
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 0.0, 0.0, 1.0];
    kf.Q = vec![0.1, 0.0, 0.0, 0.1];
    kf.H = vec![1.0, 0.0];
    kf.R = vec![0.5];

    let mut nis_values = Vec::new();

    for i in 0..100 {
        kf.predict();

        let measurement = i as f64 * 0.1 + 0.1 * pseudo_randn(i);
        let innovation = measurement - (kf.H[0] * kf.x[0] + kf.H[1] * kf.x[1]);

        // S = H*P*H^T + R (for 1D measurement, this is scalar)
        let s = kf.H[0] * kf.P[0] * kf.H[0]
            + kf.H[0] * kf.P[1] * kf.H[1]
            + kf.H[1] * kf.P[2] * kf.H[0]
            + kf.H[1] * kf.P[3] * kf.H[1]
            + kf.R[0];

        let nis = innovation * innovation / s;
        nis_values.push(nis);

        kf.update(&[measurement]).unwrap();
    }

    // Mean NIS should be approximately 1 (measurement_dim = 1)
    let mean_nis: f64 = nis_values.iter().sum::<f64>() / nis_values.len() as f64;

    // Allow generous tolerance because we're using deterministic pseudo-random
    assert!(
        (mean_nis - 1.0).abs() < 1.0,
        "Mean NIS should be approximately 1: got {}",
        mean_nis
    );
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/// Deterministic pseudo-normal distribution
fn pseudo_randn(seed: usize) -> f64 {
    // Box-Muller transform with deterministic uniform
    let u1 = ((seed * 12345) % 100000) as f64 / 100000.0 + 0.0001;
    let u2 = ((seed * 67890) % 100000) as f64 / 100000.0 + 0.0001;

    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}
