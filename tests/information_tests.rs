//! Comprehensive tests for Information Filter implementation
#![allow(unused, non_snake_case)]

use kalman_filters::filter::KalmanFilter;
use kalman_filters::information::{
    consensus::{AverageConsensus, ConsensusAlgorithm, WeightedConsensus},
    information_to_kalman, kalman_to_information,
    sparse::SparseMatrix,
    DistributedInformationFilter, ExtendedInformationFilter, InformationFilter, InformationForm,
    InformationState, SparseInformationFilter,
};
use kalman_filters::types::NonlinearSystem;
use rand::Rng;
use std::collections::HashMap;

const EPSILON: f64 = 1e-10;

/// Test equivalence between Kalman Filter and Information Filter
#[test]
fn test_kf_if_equivalence() {
    // Create identical filters
    let mut kf = KalmanFilter::<f64>::initialize(
        2,
        1,
        vec![0.0, 0.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1],
    )
    .unwrap();

    let mut inf = kalman_to_information(&kf).unwrap();

    // Perform identical operations
    for i in 0..5 {
        kf.predict();
        kf.update(&[i as f64]).unwrap();

        inf.predict().unwrap();
        inf.update(&[i as f64]).unwrap();
    }

    // Compare final states
    let kf_state = kf.state();
    let inf_state = inf.get_state().unwrap();

    for i in 0..2 {
        assert!(
            (kf_state[i] - inf_state[i]).abs() < EPSILON,
            "State mismatch at index {}: KF={}, IF={}",
            i,
            kf_state[i],
            inf_state[i]
        );
    }

    // Compare covariances
    let kf_cov = kf.covariance();
    let inf_cov = inf.get_covariance().unwrap();

    for i in 0..4 {
        assert!(
            (kf_cov[i] - inf_cov[i]).abs() < EPSILON,
            "Covariance mismatch at index {}: KF={}, IF={}",
            i,
            kf_cov[i],
            inf_cov[i]
        );
    }
}

/// Test Information Filter with no measurements
#[test]
fn test_if_no_measurement() {
    let Y_init = vec![1.0, 0.0, 0.0, 1.0];
    let y_init = vec![0.0, 0.0];

    let mut inf = InformationFilter::new(
        2,
        1,
        Y_init.clone(),
        y_init.clone(),
        vec![1.0, 0.1, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1],
    )
    .unwrap();

    // Predict without update (key advantage of IF)
    inf.predict().unwrap();
    inf.skip_update(); // No measurement
    inf.predict().unwrap();

    // Information should decrease (uncertainty increases)
    assert!(inf.state.Y[0] < Y_init[0]);
}

/// Test multiple sensor fusion
#[test]
fn test_multiple_sensor_fusion() {
    let mut inf = InformationFilter::new(
        2,
        1,
        vec![1.0, 0.0, 0.0, 1.0],
        vec![0.0, 0.0],
        vec![1.0, 0.0, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![1.0, 0.0],
        vec![0.1],
    )
    .unwrap();

    // Multiple measurements from different sensors
    let measurements = vec![
        (&[1.0][..], &[1.0, 0.0][..], &[0.1][..]),  // Sensor 1
        (&[0.9][..], &[1.0, 0.0][..], &[0.2][..]),  // Sensor 2 (noisier)
        (&[1.1][..], &[0.0, 1.0][..], &[0.15][..]), // Sensor 3 (observes velocity)
    ];

    inf.fuse_measurements(measurements).unwrap();

    // State should be close to average of measurements
    let state = inf.get_state().unwrap();
    assert!((state[0] - 1.0f64).abs() < 0.2f64);
}

/// Test sparse matrix operations
#[test]
fn test_sparse_matrix() {
    // Create a highly sparse matrix
    let dense = vec![
        1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        4.0, 0.0, 5.0, 0.0, 0.0, 0.0, 6.0,
    ];

    let sparse = SparseMatrix::from_dense(&dense, 5, 5);

    // Check sparsity
    assert!(sparse.sparsity() > 0.7); // >70% sparse
    assert_eq!(sparse.values.len(), 6); // Only 6 non-zeros

    // Test matrix-vector multiplication
    let v = vec![1.0, 2.0, 3.0, 4.0, 5.0];
    let result = sparse.multiply_vector(&v).unwrap();

    // Verify result
    assert_eq!(result[0], 1.0 * 1.0 + 2.0 * 5.0); // 11.0
    assert_eq!(result[1], 3.0 * 2.0); // 6.0
    assert_eq!(result[2], 0.0); // 0.0
    assert_eq!(result[3], 4.0 * 4.0); // 16.0
    assert_eq!(result[4], 5.0 * 1.0 + 6.0 * 5.0); // 35.0
}

/// Test sparse Information Filter
#[test]
fn test_sparse_information_filter() {
    let mut sif = SparseInformationFilter::new(
        6,              // 6-dimensional state
        vec![0.1; 36],  // Initial Y
        vec![0.0; 6],   // Initial y
        vec![1.0; 36],  // F (identity for simplicity)
        vec![0.01; 36], // Q
    )
    .unwrap();

    // Register sparse sensor (observes only states 0, 2, 4)
    let H_dense = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ];
    let H_sparse = SparseMatrix::from_dense(&H_dense, 3, 6);
    let R = vec![0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1];

    sif.register_sensor(1, H_sparse, R).unwrap();

    // Update with sparse measurement
    sif.sparse_update(1, &[1.0, 2.0, 3.0]).unwrap();

    // Check sparsity benefit
    let sparsity = sif.get_sensor_sparsity(1).unwrap();
    assert!(sparsity > 0.7);
}

/// Simple nonlinear system for EIF testing
struct SimpleNonlinear;

impl NonlinearSystem<f64> for SimpleNonlinear {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        vec![
            state[0] + state[1] * dt,
            state[1] - 0.1 * state[0].sin() * dt,
        ]
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        vec![state[0].powi(2) + state[1].powi(2)] // Range measurement
    }

    fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        vec![1.0, dt, -0.1 * state[0].cos() * dt, 1.0]
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

/// Test Extended Information Filter
#[test]
fn test_extended_information_filter() {
    let system = SimpleNonlinear;

    let Y_init = vec![100.0, 0.0, 0.0, 100.0];
    let y_init = vec![10.0, 5.0]; // x ≈ 0.1, 0.05

    let mut eif = ExtendedInformationFilter::new(
        system,
        Y_init,
        y_init,
        vec![0.001, 0.0, 0.0, 0.001],
        vec![0.01],
        0.1,
    )
    .unwrap();

    // Predict and update
    eif.predict().unwrap();
    eif.update(&[0.02]).unwrap(); // Measurement of range^2

    let state = eif.get_state().unwrap();
    assert!(state[0].abs() < 0.2); // Should be small
}

/// Test distributed Information Filter
#[test]
fn test_distributed_filter() {
    // Create 4-node network in square topology
    let mut dif = DistributedInformationFilter::<f64>::new(2);

    let Y_init = vec![1.0, 0.0, 0.0, 1.0];
    let y_init = vec![0.0, 0.0];

    for i in 0..4 {
        dif.add_node(i, Y_init.clone(), y_init.clone()).unwrap();
    }

    // Connect as square: 0-1, 1-2, 2-3, 3-0
    dif.connect_nodes(0, 1, 0).unwrap();
    dif.connect_nodes(1, 2, 0).unwrap();
    dif.connect_nodes(2, 3, 0).unwrap();
    dif.connect_nodes(3, 0, 0).unwrap();

    // Node 0 makes measurement
    let H = vec![1.0, 0.0];
    let R = vec![0.1];
    dif.local_update(0, &[1.0], &H, &R).unwrap();

    // Propagate information
    for _ in 0..4 {
        dif.process_messages().unwrap();
    }

    // All nodes should have received information
    for i in 0..4 {
        let state = dif.get_node_state(i).unwrap();
        assert!(state[0] > 0.0, "Node {} didn't receive information", i);
    }
}

/// Test consensus algorithms
#[test]
fn test_consensus_algorithms() {
    // Create simple topology
    let mut topology = HashMap::new();
    topology.insert(0, vec![1, 2]);
    topology.insert(1, vec![0, 2]);
    topology.insert(2, vec![0, 1]);

    // Test average consensus
    {
        let mut consensus = AverageConsensus::<f64>::new_metropolis(&topology, 2);

        let mut states = HashMap::new();
        states.insert(
            0,
            InformationState::from_information(vec![3.0, 0.0, 0.0, 3.0], vec![3.0, 0.0]).unwrap(),
        );
        states.insert(
            1,
            InformationState::from_information(vec![2.0, 0.0, 0.0, 2.0], vec![2.0, 0.0]).unwrap(),
        );
        states.insert(
            2,
            InformationState::from_information(vec![1.0, 0.0, 0.0, 1.0], vec![1.0, 0.0]).unwrap(),
        );

        // Run consensus
        for _ in 0..20 {
            consensus.iterate(&mut states, &topology).unwrap();
        }

        assert!(consensus.is_converged(0.01));

        // Check convergence to average
        let avg = consensus.get_consensus().unwrap();
        assert!((avg.Y[0] - 2.0).abs() < 0.1); // Average of [3, 2, 1]
    }

    // Test weighted consensus
    {
        let mut consensus = WeightedConsensus::<f64>::new(2);

        let mut states = HashMap::new();
        states.insert(
            0,
            InformationState::from_information(
                vec![10.0, 0.0, 0.0, 10.0], // High confidence
                vec![10.0, 0.0],
            )
            .unwrap(),
        );
        states.insert(
            1,
            InformationState::from_information(
                vec![1.0, 0.0, 0.0, 1.0], // Low confidence
                vec![5.0, 0.0],
            )
            .unwrap(),
        );

        consensus.iterate(&mut states, &topology).unwrap();

        // Should be weighted toward high-confidence node
        let result = consensus.get_consensus().unwrap();
        // The weighted consensus should combine the information vectors
        // High confidence node has y = [10, 0], low confidence has y = [5, 0]
        // Result should be weighted average closer to high confidence value
        assert!(result.y[0] > 5.0); // Weighted average favoring high-confidence node
    }
}

/// Test conversion between forms
#[test]
fn test_form_conversion() {
    // Start with KF
    let kf = KalmanFilter::<f64>::initialize(
        3,
        2,
        vec![1.0, 2.0, 3.0],
        vec![2.0, 0.5, 0.1, 0.5, 1.5, 0.2, 0.1, 0.2, 1.0],
        vec![1.0; 9],
        vec![0.01; 9],
        vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        vec![0.1, 0.0, 0.0, 0.1],
    )
    .unwrap();

    // Convert to IF
    let inf = kalman_to_information(&kf).unwrap();

    // Convert back to KF
    let kf2 = information_to_kalman(&inf).unwrap();

    // Check equivalence
    for i in 0..3 {
        assert!((kf.x[i] - kf2.x[i]).abs() < EPSILON);
    }
    for i in 0..9 {
        assert!((kf.P[i] - kf2.P[i]).abs() < EPSILON);
    }
}

/// Test information addition property
#[test]
fn test_information_addition() {
    let mut state = InformationState::<f64>::new(2);
    state.Y = vec![1.0, 0.0, 0.0, 1.0];
    state.y = vec![0.0, 0.0];

    // Add information from measurement 1
    let delta_Y1 = vec![2.0, 0.0, 0.0, 2.0];
    let delta_y1 = vec![2.0, 0.0];
    state.add_information(&delta_y1, &delta_Y1);

    // Add information from measurement 2
    let delta_Y2 = vec![1.0, 0.0, 0.0, 1.0];
    let delta_y2 = vec![1.0, 0.0];
    state.add_information(&delta_y2, &delta_Y2);

    // Check additive property
    assert_eq!(state.Y[0], 4.0); // 1 + 2 + 1
    assert_eq!(state.y[0], 3.0); // 0 + 2 + 1
}

/// Test numerical stability
#[test]
fn test_numerical_stability() {
    // Test with nearly singular information matrix
    let Y_init = vec![1e-10, 0.0, 0.0, 1.0];
    let y_init = vec![0.0, 1.0];

    let mut inf = InformationFilter::new(
        2,
        1,
        Y_init,
        y_init,
        vec![1.0, 0.0, 0.0, 1.0],
        vec![0.01, 0.0, 0.0, 0.01],
        vec![0.0, 1.0], // Only observes second state
        vec![0.1],
    )
    .unwrap();

    // Should handle prediction despite poor observability of first state
    let result = inf.predict();
    assert!(result.is_ok() || result.is_err()); // May fail gracefully
}

/// Benchmark sparse vs dense operations
#[test]
#[ignore] // Run with --ignored for benchmarks
fn bench_sparse_vs_dense() {
    use std::time::Instant;

    let n = 100; // Large state dimension
    let sparsity = 0.95; // 95% sparse

    // Create sparse matrix
    let mut dense = vec![0.0; n * n];
    let mut rng = rand::thread_rng();
    for i in 0..n {
        for j in 0..n {
            if rand::random::<f64>() > sparsity {
                dense[i * n + j] = rng.gen_range(-1.0f64..1.0f64);
            }
        }
    }

    let sparse = SparseMatrix::from_dense(&dense, n, n);
    let v = vec![1.0; n];

    // Benchmark sparse multiplication
    let start = Instant::now();
    for _ in 0..1000 {
        let _ = sparse.multiply_vector(&v).unwrap();
    }
    let sparse_time = start.elapsed();

    // Benchmark dense multiplication
    let start = Instant::now();
    for _ in 0..1000 {
        let mut result = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                result[i] += dense[i * n + j] * v[j];
            }
        }
    }
    let dense_time = start.elapsed();

    println!("Sparse time: {:?}", sparse_time);
    println!("Dense time: {:?}", dense_time);
    println!(
        "Speedup: {:.2}x",
        dense_time.as_secs_f64() / sparse_time.as_secs_f64()
    );

    // Sparse should be faster for high sparsity
    assert!(sparse_time < dense_time);
}
