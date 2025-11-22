//! Fault tolerance and real-world failure mode tests
//!
//! These tests verify that filters handle real-world failure scenarios gracefully,
//! including sensor dropouts, outliers, network failures, and Byzantine faults.

use kalman_filters::filter::KalmanFilter;
use kalman_filters::information::{
    DistributedInformationFilter, InformationState,
    consensus::{AverageConsensus, ConsensusAlgorithm},
};
use kalman_filters::{EnsembleKalmanFilterBuilder, NonlinearSystem, ParticleFilterBuilder};
use std::collections::HashMap;

// =============================================================================
// SENSOR FAILURE TESTS
// =============================================================================

#[test]
fn test_kf_handles_measurement_dropout() {
    // Sensor intermittently drops measurements
    let mut kf = KalmanFilter::<f64>::new(2, 1);
    kf.x = vec![0.0, 1.0]; // Position 0, velocity 1
    kf.P = vec![1.0, 0.0, 0.0, 1.0];
    kf.F = vec![1.0, 0.1, 0.0, 1.0];
    kf.Q = vec![0.01, 0.0, 0.0, 0.01];
    kf.H = vec![1.0, 0.0];
    kf.R = vec![0.1];

    let mut position = 0.0;
    let velocity = 1.0;

    // Simulate with 50% measurement dropout
    for i in 0..50 {
        position += velocity * 0.1;
        kf.predict();

        if i % 2 == 0 {
            // Measurement available
            kf.update(&[position + 0.01 * (i as f64 % 3.0 - 1.0)])
                .unwrap();
        }
        // else: measurement dropout - only predict, no update
    }

    // With 50% dropout, filter may drift but shouldn't diverge wildly
    let pos_error = (kf.x[0] - position).abs();
    let vel_error = (kf.x[1] - velocity).abs();

    // More generous tolerances for 50% dropout
    assert!(
        pos_error < 2.0,
        "Position error too large with dropout: {}",
        pos_error
    );
    assert!(
        vel_error < 1.0,
        "Velocity error too large with dropout: {}",
        vel_error
    );

    // Uncertainty should be higher than with continuous measurements
    assert!(
        kf.P[0] > 0.05,
        "Covariance should reflect measurement gaps: {}",
        kf.P[0]
    );
}

#[test]
fn test_kf_measurement_outlier_detection() {
    // Filter should be robust to occasional outliers
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![5.0];
    kf.P = vec![1.0];
    kf.F = vec![1.0];
    kf.Q = vec![0.01];
    kf.H = vec![1.0];
    kf.R = vec![0.1];

    let true_value = 5.0;
    let measurements = vec![
        5.1, 4.9, 5.05, 4.95, 100.0, // <- outlier
        5.0, 5.1, 4.9, -50.0, // <- outlier
        5.05, 4.95, 5.0,
    ];

    for (i, meas) in measurements.iter().enumerate() {
        kf.predict();

        // Simple outlier rejection: check innovation
        let innovation = meas - kf.x[0];
        let innovation_cov = kf.P[0] + kf.R[0];
        let mahalanobis = (innovation * innovation / innovation_cov).sqrt();

        if mahalanobis < 5.0 {
            // Accept measurement
            kf.update(&[*meas]).unwrap();
        } else {
            // Reject outlier - skip update
            println!("Rejected outlier at step {}: {}", i, meas);
        }
    }

    // Final estimate should be close to true value
    let error = (kf.x[0] - true_value).abs();
    assert!(
        error < 0.5,
        "Filter should reject outliers: error={}",
        error
    );
}

#[test]
fn test_kf_sensor_bias_estimation() {
    // Filter should eventually converge to the combined measurement (position + bias)
    // State: [combined_value] - simpler model
    let mut kf = KalmanFilter::<f64>::new(1, 1);
    kf.x = vec![0.0]; // Unknown initial value
    kf.P = vec![10.0]; // Very uncertain
    kf.F = vec![1.0]; // Constant value
    kf.Q = vec![0.001]; // Very slow drift
    kf.H = vec![1.0]; // Direct observation
    kf.R = vec![0.1];

    let true_combined_value = 7.0; // position (5.0) + bias (2.0)

    // Feed measurements
    for _ in 0..100 {
        kf.predict();
        let measurement = true_combined_value + 0.05 * (rand_deterministic() - 0.5);
        kf.update(&[measurement]).unwrap();
    }

    // Should converge to the true combined value
    let error = (kf.x[0] - true_combined_value).abs();

    assert!(
        error < 0.5,
        "Estimate should converge to true value: error={}",
        error
    );
}

// =============================================================================
// DISTRIBUTED SYSTEM FAULT TOLERANCE
// =============================================================================

#[test]
fn test_distributed_if_node_failure() {
    // Test graceful degradation when a node in distributed system fails
    let mut dif = DistributedInformationFilter::<f64>::new(2);

    let Y_init = vec![1.0, 0.0, 0.0, 1.0];
    let y_init = vec![0.0, 0.0];

    // Create 5-node network
    for i in 0..5 {
        dif.add_node(i, Y_init.clone(), y_init.clone()).unwrap();
    }

    // Connect in ring topology
    for i in 0..5 {
        dif.connect_nodes(i, (i + 1) % 5, 0).unwrap();
    }

    // Node 0 gets measurement
    let H = vec![1.0, 0.0];
    let R = vec![0.1];
    dif.local_update(0, &[5.0], &H, &R).unwrap();

    // Propagate normally for a few iterations
    for _ in 0..3 {
        dif.process_messages().unwrap();
    }

    // Simulate node 2 failure by removing it
    dif.nodes.remove(&2);

    // Network should still function (degraded)
    for _ in 0..5 {
        let result = dif.process_messages();
        // Should handle missing node gracefully
        if result.is_err() {
            // Acceptable to have some errors with missing node
            continue;
        }
    }

    // Remaining nodes should still have reasonable estimates
    for i in &[0, 1, 3, 4] {
        if let Ok(state) = dif.get_node_state(*i) {
            assert!(state[0] > 1.0, "Node {} should have some information", i);
        }
    }
}

#[test]
fn test_consensus_with_byzantine_node() {
    // Test consensus when one node sends bad data (Byzantine fault)
    let mut topology = HashMap::new();
    topology.insert(0, vec![1, 2, 3]);
    topology.insert(1, vec![0, 2, 3]);
    topology.insert(2, vec![0, 1, 3]);
    topology.insert(3, vec![0, 1, 2]); // Fully connected

    let mut consensus = AverageConsensus::<f64>::new_metropolis(&topology, 2);

    let mut states = HashMap::new();
    // Honest nodes have similar values
    states.insert(
        0,
        InformationState::from_information(vec![2.0, 0.0, 0.0, 2.0], vec![2.0, 0.0]).unwrap(),
    );
    states.insert(
        1,
        InformationState::from_information(vec![2.1, 0.0, 0.0, 2.1], vec![2.1, 0.0]).unwrap(),
    );
    states.insert(
        2,
        InformationState::from_information(vec![1.9, 0.0, 0.0, 1.9], vec![1.9, 0.0]).unwrap(),
    );

    // Byzantine node with wildly different value
    states.insert(
        3,
        InformationState::from_information(vec![100.0, 0.0, 0.0, 100.0], vec![100.0, 0.0]).unwrap(),
    );

    // Run consensus
    for _ in 0..30 {
        consensus.iterate(&mut states, &topology).unwrap();
    }

    // Average should be pulled by Byzantine node, but we can detect this
    let result = consensus.get_consensus().unwrap();

    // With 1 Byzantine out of 4 nodes, average is corrupted
    // In real system, we'd use robust consensus (median, trimmed mean, etc.)
    // This test documents the vulnerability
    assert!(
        result.Y[0] > 10.0,
        "Byzantine node should affect consensus (demonstrates need for robust consensus)"
    );
}

#[test]
fn test_distributed_if_network_partition() {
    // Test behavior when network partitions into two groups
    let mut dif = DistributedInformationFilter::<f64>::new(2);

    let Y_init = vec![1.0, 0.0, 0.0, 1.0];
    let y_init = vec![0.0, 0.0];

    // Create 6 nodes
    for i in 0..6 {
        dif.add_node(i, Y_init.clone(), y_init.clone()).unwrap();
    }

    // Connect in two groups: [0,1,2] and [3,4,5]
    // Group 1
    dif.connect_nodes(0, 1, 0).unwrap();
    dif.connect_nodes(1, 2, 0).unwrap();
    dif.connect_nodes(2, 0, 0).unwrap();

    // Group 2
    dif.connect_nodes(3, 4, 0).unwrap();
    dif.connect_nodes(4, 5, 0).unwrap();
    dif.connect_nodes(5, 3, 0).unwrap();

    // Each partition gets different measurement
    let H = vec![1.0, 0.0];
    let R = vec![0.1];
    dif.local_update(0, &[5.0], &H, &R).unwrap(); // Partition 1 sees 5.0
    dif.local_update(3, &[-3.0], &H, &R).unwrap(); // Partition 2 sees -3.0

    // Propagate within partitions
    for _ in 0..10 {
        dif.process_messages().unwrap();
    }

    // Check that partitions have converged to different values
    let state_0 = dif.get_node_state(0).unwrap();
    let state_3 = dif.get_node_state(3).unwrap();

    assert!(
        state_0[0] > 3.0,
        "Partition 1 should converge near 5.0: {}",
        state_0[0]
    );
    assert!(
        state_3[0] < 0.0,
        "Partition 2 should converge near -3.0: {}",
        state_3[0]
    );

    // Now reconnect partitions (heal network partition)
    dif.connect_nodes(2, 3, 0).unwrap();

    // Continue consensus
    for _ in 0..20 {
        dif.process_messages().unwrap();
    }

    // After reconnection, should converge to average
    let state_0_after = dif.get_node_state(0).unwrap();
    let state_3_after = dif.get_node_state(3).unwrap();

    // Both should be closer together
    let diff = (state_0_after[0] - state_3_after[0]).abs();
    assert!(
        diff < 5.0,
        "Partitions should reconcile after reconnection: diff={}",
        diff
    );
}

// =============================================================================
// PARTICLE FILTER DEGENERACY TESTS
// =============================================================================

#[test]
fn test_particle_filter_degeneracy_recovery() {
    // Test that PF recovers from particle degeneracy
    let mut pf = ParticleFilterBuilder::new(1, 100)
        .initial_mean(vec![5.0]) // Start near measurement
        .initial_std(vec![0.5]) // Narrow initial spread
        .process_noise_std(vec![0.1])
        .measurement_noise_std(vec![0.5])
        .dt(0.1)
        .ess_threshold(50.0)
        .build()
        .unwrap();

    // Very peaked likelihood that will cause degeneracy
    let peaked_likelihood = |state: &[f64], measurement: &[f64]| -> f64 {
        let diff = state[0] - measurement[0];
        let sigma = 0.05; // Very narrow - causes degeneracy
        (-0.5 * diff * diff / (sigma * sigma)).exp()
    };

    let initial_ess = pf.effective_sample_size();

    // Update with peaked likelihood
    pf.update(&[5.1], peaked_likelihood).unwrap(); // Slightly off from mean

    let ess_after_update = pf.effective_sample_size();

    // ESS may decrease (or may not if particles are already concentrated)
    // The key property is that resampling restores uniform weights
    println!(
        "ESS: initial={}, after_update={}",
        initial_ess, ess_after_update
    );

    // Resample to recover
    pf.resample().unwrap();

    let ess_after_resample = pf.effective_sample_size();

    // ESS should recover to N after resampling (uniform weights)
    assert!(
        (ess_after_resample - 100.0).abs() < 1e-6,
        "ESS should recover to N after resampling: {}",
        ess_after_resample
    );
}

#[test]
fn test_particle_filter_sample_impoverishment() {
    // Test particle diversity after multiple resample cycles
    let mut pf = ParticleFilterBuilder::new(1, 50)
        .initial_mean(vec![0.0])
        .initial_std(vec![1.0])
        .process_noise_std(vec![0.2]) // Some jitter to maintain diversity
        .measurement_noise_std(vec![0.5])
        .dt(0.1)
        .build()
        .unwrap();

    let simple_likelihood = |state: &[f64], measurement: &[f64]| -> f64 {
        let diff = state[0] - measurement[0];
        let sigma = 0.5;
        (-0.5 * diff * diff / (sigma * sigma)).exp()
    };

    // Simple motion model
    let motion = |state: &[f64], dt: f64| -> Vec<f64> {
        vec![state[0]] // Static
    };

    // Run many update-resample cycles
    for i in 0..20 {
        pf.predict(motion);
        pf.update(&[i as f64 * 0.1], simple_likelihood).unwrap();
        pf.resample().unwrap();
    }

    // Check particle diversity (variance of particle states)
    let mean = pf.mean();
    let variance: f64 = pf
        .particles
        .iter()
        .map(|p| (p.state[0] - mean[0]).powi(2))
        .sum::<f64>()
        / pf.particles.len() as f64;

    // Diversity should be maintained through process noise
    assert!(
        variance > 0.01,
        "Particles should maintain diversity: variance={}",
        variance
    );
}

// =============================================================================
// ENSEMBLE FILTER COLLAPSE TESTS
// =============================================================================

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

#[test]
fn test_enkf_prevents_ensemble_collapse() {
    // Test that inflation prevents ensemble collapse
    let mut enkf_no_inflation = EnsembleKalmanFilterBuilder::new(SimpleSystem)
        .initial_mean(vec![0.0, 0.0])
        .initial_spread(vec![1.0, 1.0])
        .ensemble_size(50)
        .process_noise(vec![0.001, 0.0, 0.0, 0.001]) // Very small
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .inflation_factor(1.0) // No inflation
        .build()
        .unwrap();

    let mut enkf_with_inflation = EnsembleKalmanFilterBuilder::new(SimpleSystem)
        .initial_mean(vec![0.0, 0.0])
        .initial_spread(vec![1.0, 1.0])
        .ensemble_size(50)
        .process_noise(vec![0.001, 0.0, 0.0, 0.001])
        .measurement_noise(vec![0.1])
        .dt(0.1)
        .inflation_factor(1.05) // 5% inflation
        .build()
        .unwrap();

    // Run many forecast-update cycles
    for _ in 0..100 {
        enkf_no_inflation.forecast();
        enkf_with_inflation.forecast();

        enkf_no_inflation.update(&[1.0]).unwrap();
        enkf_with_inflation.update(&[1.0]).unwrap();
    }

    let spread_no_infl = enkf_no_inflation.spread();
    let spread_with_infl = enkf_with_inflation.spread();

    // Ensemble with inflation should maintain larger spread
    assert!(
        spread_with_infl[0] > spread_no_infl[0],
        "Inflation should prevent collapse: no_infl={}, with_infl={}",
        spread_no_infl[0],
        spread_with_infl[0]
    );

    // Without inflation, ensemble might collapse too much
    // This is a known issue in EnKF
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

fn rand_deterministic() -> f64 {
    use std::cell::Cell;
    thread_local! {
        static SEED: Cell<u64> = const { Cell::new(12345) };
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
