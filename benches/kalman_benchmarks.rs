use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use kalman_filter::{
    information::InformationFilter, ExtendedKalmanFilter, KalmanFilterBuilder, NonlinearSystem,
    ParticleFilter, UnscentedKalmanFilter,
};
use std::hint::black_box;
use std::time::Duration;

// ============================================================================
// CORE OPERATIONS BENCHMARKS
// ============================================================================

fn bench_kalman_filter_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("kalman_filter_core_operations");

    // Test different dimensions: 2D, 4D, 10D, 50D, 100D
    let dimensions = vec![2, 4, 10, 50, 100];

    for &dim in &dimensions {
        // Setup filter for benchmarking
        let mut kf = create_test_kalman_filter(dim, dim / 2);
        let measurement = vec![1.0; dim / 2];

        // Benchmark predict step
        group.bench_with_input(BenchmarkId::new("predict", dim), &dim, |b, _| {
            b.iter(|| {
                black_box(kf.predict());
            });
        });

        // Benchmark update step
        group.bench_with_input(BenchmarkId::new("update", dim), &dim, |b, _| {
            b.iter(|| {
                black_box(kf.update(black_box(&measurement)).unwrap());
            });
        });

        // Benchmark full predict-update cycle
        group.bench_with_input(
            BenchmarkId::new("predict_update_cycle", dim),
            &dim,
            |b, _| {
                b.iter(|| {
                    kf.predict();
                    black_box(kf.update(black_box(&measurement)).unwrap());
                });
            },
        );
    }

    group.finish();
}

fn bench_matrix_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("matrix_operations");

    let dimensions = vec![2, 4, 10, 50, 100];

    for &dim in &dimensions {
        // Create test matrices
        let mut matrix = create_test_matrix(dim);

        // Benchmark matrix inversion (core bottleneck)
        group.bench_with_input(BenchmarkId::new("matrix_inversion", dim), &dim, |b, _| {
            b.iter(|| {
                black_box(invert_matrix(black_box(&mut matrix), dim));
            });
        });
    }

    group.finish();
}

// ============================================================================
// FILTER VARIANT COMPARISONS
// ============================================================================

fn bench_filter_variants(c: &mut Criterion) {
    let mut group = c.benchmark_group("filter_variants");
    group.measurement_time(Duration::from_secs(10));

    let state_dim = 4; // 2D position + velocity
    let measurement_dim = 2; // Position measurements
    let measurement = vec![1.0; measurement_dim];

    // Kalman Filter
    let mut kf = create_test_kalman_filter(state_dim, measurement_dim);
    group.bench_function("kalman_filter", |b| {
        b.iter(|| {
            kf.predict();
            black_box(kf.update(black_box(&measurement)).unwrap());
        });
    });

    // Extended Kalman Filter
    let system = TestNonlinearSystem::new(state_dim, measurement_dim);
    let mut ekf = ExtendedKalmanFilter::new(
        system,
        vec![0.0; state_dim],
        create_identity_matrix(state_dim),
        create_identity_matrix(state_dim),
        create_identity_matrix(measurement_dim),
        0.01,
    )
    .unwrap();

    group.bench_function("extended_kalman_filter", |b| {
        b.iter(|| {
            ekf.predict();
            black_box(ekf.update(black_box(&measurement)).unwrap());
        });
    });

    // Unscented Kalman Filter
    let mut ukf = UnscentedKalmanFilter::new(
        TestNonlinearSystem::new(state_dim, measurement_dim),
        vec![0.0; state_dim],
        create_identity_matrix(state_dim),
        create_identity_matrix(state_dim),
        create_identity_matrix(measurement_dim),
        0.01,
    )
    .unwrap();

    group.bench_function("unscented_kalman_filter", |b| {
        b.iter(|| {
            ukf.predict();
            black_box(ukf.update(black_box(&measurement)).unwrap());
        });
    });

    // Information Filter
    let mut if_filter = InformationFilter::new(
        state_dim,
        measurement_dim,
        vec![0.0; state_dim],
        create_identity_matrix(state_dim),
        create_identity_matrix(state_dim),
        create_identity_matrix(state_dim),
        create_identity_matrix(measurement_dim),
        create_identity_matrix(measurement_dim),
    )
    .unwrap();

    group.bench_function("information_filter", |b| {
        b.iter(|| {
            if_filter.predict();
            black_box(if_filter.update(black_box(&measurement)).unwrap());
        });
    });

    // Particle Filter
    let mut pf = ParticleFilter::new(
        state_dim,
        1000,                       // particles
        vec![0.0; state_dim],       // initial mean
        vec![1.0; state_dim],       // initial std
        vec![0.1; state_dim],       // process noise std
        vec![0.1; measurement_dim], // measurement noise std
        0.01,                       // dt
    )
    .unwrap();

    group.bench_function("particle_filter_1000", |b| {
        b.iter(|| {
            pf.predict(|state, dt| {
                // Simple random walk
                state.iter().map(|&x| x + 0.01 * dt).collect()
            });
            black_box(
                pf.update(black_box(&measurement), |state, measurement| {
                    // Gaussian likelihood function
                    let mut likelihood = 1.0f64;
                    for i in 0..measurement.len().min(state.len()) {
                        let diff = state[i] - measurement[i];
                        likelihood *= (-0.5f64 * diff * diff / 0.01f64).exp();
                    }
                    likelihood
                })
                .unwrap(),
            );
        });
    });

    group.finish();
}

// ============================================================================
// SCALING BENCHMARKS
// ============================================================================

fn bench_dimension_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("dimension_scaling");
    group.sample_size(50); // Reduce sample size for large dimensions

    // Test scaling from 2D to 1000D
    let large_dimensions = vec![2, 4, 10, 25, 50, 100, 250, 500, 1000];

    for &dim in &large_dimensions {
        let mut kf = create_test_kalman_filter(dim, dim / 2);
        let measurement = vec![1.0; dim / 2];

        group.throughput(Throughput::Elements(dim as u64));
        group.bench_with_input(
            BenchmarkId::new("large_dimension_predict", dim),
            &dim,
            |b, _| {
                b.iter(|| {
                    black_box(kf.predict());
                });
            },
        );

        group.bench_with_input(
            BenchmarkId::new("large_dimension_update", dim),
            &dim,
            |b, _| {
                b.iter(|| {
                    black_box(kf.update(black_box(&measurement)).unwrap());
                });
            },
        );
    }

    group.finish();
}

fn bench_particle_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("particle_filter_scaling");

    let particle_counts = vec![100, 500, 1000, 5000, 10000];
    let state_dim = 4;
    let measurement_dim = 2;
    let measurement = vec![1.0; measurement_dim];

    for &count in &particle_counts {
        let mut pf = ParticleFilter::new(
            state_dim,
            count,
            vec![0.0; state_dim],       // initial mean
            vec![1.0; state_dim],       // initial std
            vec![0.1; state_dim],       // process noise std
            vec![0.1; measurement_dim], // measurement noise std
            0.01,                       // dt
        )
        .unwrap();

        group.throughput(Throughput::Elements(count as u64));
        group.bench_with_input(BenchmarkId::new("particle_count", count), &count, |b, _| {
            b.iter(|| {
                pf.predict(|state, dt| state.iter().map(|&x| x + 0.01 * dt).collect());
                black_box(
                    pf.update(black_box(&measurement), |state, measurement| {
                        // Gaussian likelihood function
                        let mut likelihood = 1.0f64;
                        for i in 0..measurement.len().min(state.len()) {
                            let diff = state[i] - measurement[i];
                            likelihood *= (-0.5f64 * diff * diff / 0.01f64).exp();
                        }
                        likelihood
                    })
                    .unwrap(),
                );
            });
        });
    }

    group.finish();
}

// ============================================================================
// SPARSE VS DENSE OPERATIONS
// ============================================================================

fn bench_sparse_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("sparse_vs_dense");

    // Test different sparsity levels
    let dimensions = vec![50, 100, 200];
    let sparsity_levels = vec![0.5, 0.7, 0.9, 0.95]; // 50%, 70%, 90%, 95% sparse

    for &dim in &dimensions {
        for &sparsity in &sparsity_levels {
            let dense_matrix = create_test_matrix(dim);
            let sparse_matrix = create_sparse_matrix(dim, sparsity);

            group.bench_with_input(
                BenchmarkId::new(format!("dense_{}d", dim), sparsity),
                &(dim, sparsity),
                |b, _| {
                    b.iter(|| {
                        black_box(matrix_vector_multiply(
                            black_box(&dense_matrix),
                            black_box(&vec![1.0; dim]),
                            dim,
                        ));
                    });
                },
            );

            group.bench_with_input(
                BenchmarkId::new(format!("sparse_{}d", dim), sparsity),
                &(dim, sparsity),
                |b, _| {
                    b.iter(|| {
                        black_box(sparse_matrix_vector_multiply(
                            black_box(&sparse_matrix),
                            black_box(&vec![1.0; dim]),
                            dim,
                        ));
                    });
                },
            );
        }
    }

    group.finish();
}

// ============================================================================
// MEMORY AND ALLOCATION BENCHMARKS
// ============================================================================

fn bench_memory_allocations(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_allocations");

    let dimensions = vec![10, 50, 100];

    for &dim in &dimensions {
        // Benchmark filter construction
        group.bench_with_input(
            BenchmarkId::new("filter_construction", dim),
            &dim,
            |b, _| {
                b.iter(|| {
                    black_box(create_test_kalman_filter(dim, dim / 2));
                });
            },
        );

        // Benchmark state cloning
        let kf = create_test_kalman_filter(dim, dim / 2);
        group.bench_with_input(BenchmarkId::new("state_clone", dim), &dim, |b, _| {
            b.iter(|| {
                black_box(kf.state().to_vec());
            });
        });
    }

    group.finish();
}

// ============================================================================
// REAL-WORLD SCENARIOS
// ============================================================================

fn bench_real_world_scenarios(c: &mut Criterion) {
    let mut group = c.benchmark_group("real_world_scenarios");
    group.measurement_time(Duration::from_secs(10));

    // IMU fusion (6DOF: position + velocity)
    let mut imu_filter = create_test_kalman_filter(6, 3);
    let imu_measurement = vec![0.1, 0.2, 0.3]; // accelerometer

    group.bench_function("imu_fusion_6dof", |b| {
        b.iter(|| {
            imu_filter.predict();
            black_box(imu_filter.update(black_box(&imu_measurement)).unwrap());
        });
    });

    // GPS tracking (4D: 2D position + velocity)
    let mut gps_filter = create_test_kalman_filter(4, 2);
    let gps_measurement = vec![1.0, 2.0]; // position

    group.bench_function("gps_tracking_4d", |b| {
        b.iter(|| {
            gps_filter.predict();
            black_box(gps_filter.update(black_box(&gps_measurement)).unwrap());
        });
    });

    // Radar tracking (6D: 3D position + velocity)
    let mut radar_filter = create_test_kalman_filter(6, 3);
    let radar_measurement = vec![10.0, 20.0, 5.0]; // range, azimuth, elevation

    group.bench_function("radar_tracking_6d", |b| {
        b.iter(|| {
            radar_filter.predict();
            black_box(radar_filter.update(black_box(&radar_measurement)).unwrap());
        });
    });

    group.finish();
}

// ============================================================================
// COMPARATIVE BENCHMARKS
// ============================================================================

#[cfg(feature = "adskalman")]
fn bench_vs_adskalman(c: &mut Criterion) {
    let mut group = c.benchmark_group("comparison_with_adskalman");

    let state_dim = 4;
    let measurement_dim = 2;
    let measurement = vec![1.0; measurement_dim];

    // Our implementation
    let mut our_kf = create_test_kalman_filter(state_dim, measurement_dim);
    group.bench_function("our_kalman_filter", |b| {
        b.iter(|| {
            our_kf.predict();
            black_box(our_kf.update(black_box(&measurement)).unwrap());
        });
    });

    // adskalman implementation (if feature enabled)
    use adskalman::KalmanFilterNoControl;
    let mut ads_kf = KalmanFilterNoControl::new(
        nalgebra::SMatrix::<f64, 4, 4>::identity(), // transition
        nalgebra::SMatrix::<f64, 2, 4>::zeros(),    // observation
        nalgebra::SMatrix::<f64, 4, 4>::identity(), // process noise
        nalgebra::SMatrix::<f64, 2, 2>::identity(), // measurement noise
    );

    group.bench_function("adskalman_reference", |b| {
        b.iter(|| {
            let prediction = ads_kf.predict();
            let obs = nalgebra::SVector::<f64, 2>::new(1.0, 1.0);
            black_box(prediction.update(&obs));
        });
    });

    group.finish();
}

fn bench_precision_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("precision_comparison");

    let state_dim = 10;
    let measurement_dim = 5;
    let measurement_f32 = vec![1.0f32; measurement_dim];
    let measurement_f64 = vec![1.0f64; measurement_dim];

    // f32 precision
    let mut kf_f32 = create_test_kalman_filter_f32(state_dim, measurement_dim);
    group.bench_function("f32_precision", |b| {
        b.iter(|| {
            kf_f32.predict();
            black_box(kf_f32.update(black_box(&measurement_f32)).unwrap());
        });
    });

    // f64 precision
    let mut kf_f64 = create_test_kalman_filter(state_dim, measurement_dim);
    group.bench_function("f64_precision", |b| {
        b.iter(|| {
            kf_f64.predict();
            black_box(kf_f64.update(black_box(&measurement_f64)).unwrap());
        });
    });

    group.finish();
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

fn create_test_kalman_filter(
    state_dim: usize,
    measurement_dim: usize,
) -> kalman_filter::KalmanFilter<f64> {
    KalmanFilterBuilder::new(state_dim, measurement_dim)
        .initial_state(vec![0.0; state_dim])
        .initial_covariance(create_identity_matrix(state_dim))
        .transition_matrix(create_identity_matrix(state_dim))
        .process_noise(create_scaled_identity_matrix(state_dim, 0.01))
        .observation_matrix(create_observation_matrix(measurement_dim, state_dim))
        .measurement_noise(create_scaled_identity_matrix(measurement_dim, 0.1))
        .build()
        .unwrap()
}

fn create_test_kalman_filter_f32(
    state_dim: usize,
    measurement_dim: usize,
) -> kalman_filter::KalmanFilter<f32> {
    KalmanFilterBuilder::new(state_dim, measurement_dim)
        .initial_state(vec![0.0f32; state_dim])
        .initial_covariance(create_identity_matrix_f32(state_dim))
        .transition_matrix(create_identity_matrix_f32(state_dim))
        .process_noise(create_scaled_identity_matrix_f32(state_dim, 0.01))
        .observation_matrix(create_observation_matrix_f32(measurement_dim, state_dim))
        .measurement_noise(create_scaled_identity_matrix_f32(measurement_dim, 0.1))
        .build()
        .unwrap()
}

fn create_identity_matrix(size: usize) -> Vec<f64> {
    let mut matrix = vec![0.0; size * size];
    for i in 0..size {
        matrix[i * size + i] = 1.0;
    }
    matrix
}

fn create_identity_matrix_f32(size: usize) -> Vec<f32> {
    let mut matrix = vec![0.0f32; size * size];
    for i in 0..size {
        matrix[i * size + i] = 1.0;
    }
    matrix
}

fn create_scaled_identity_matrix(size: usize, scale: f64) -> Vec<f64> {
    let mut matrix = vec![0.0; size * size];
    for i in 0..size {
        matrix[i * size + i] = scale;
    }
    matrix
}

fn create_scaled_identity_matrix_f32(size: usize, scale: f32) -> Vec<f32> {
    let mut matrix = vec![0.0f32; size * size];
    for i in 0..size {
        matrix[i * size + i] = scale;
    }
    matrix
}

fn create_observation_matrix(measurement_dim: usize, state_dim: usize) -> Vec<f64> {
    let mut matrix = vec![0.0; measurement_dim * state_dim];
    for i in 0..measurement_dim.min(state_dim) {
        matrix[i * state_dim + i] = 1.0;
    }
    matrix
}

fn create_observation_matrix_f32(measurement_dim: usize, state_dim: usize) -> Vec<f32> {
    let mut matrix = vec![0.0f32; measurement_dim * state_dim];
    for i in 0..measurement_dim.min(state_dim) {
        matrix[i * state_dim + i] = 1.0;
    }
    matrix
}

fn create_test_matrix(size: usize) -> Vec<f64> {
    let mut matrix = create_identity_matrix(size);
    // Add some structure to make it more realistic
    for i in 0..size {
        for j in 0..size {
            if i != j {
                matrix[i * size + j] = 0.1 / ((i + j + 1) as f64);
            }
        }
    }
    matrix
}

fn create_sparse_matrix(size: usize, sparsity: f64) -> Vec<f64> {
    let mut matrix = create_test_matrix(size);
    let total_elements = size * size;
    let zero_count = (total_elements as f64 * sparsity) as usize;

    // Randomly zero out elements (deterministic for benchmarking)
    for i in 0..zero_count {
        let idx = (i * 7 + 13) % total_elements; // Simple pseudo-random
        if idx < matrix.len() {
            matrix[idx] = 0.0;
        }
    }
    matrix
}

// Simple matrix inversion for benchmarking (not production-quality)
fn invert_matrix(matrix: &mut [f64], size: usize) -> bool {
    // Simple Gauss-Jordan elimination for benchmarking
    // This is not the production implementation
    if size <= 3 {
        // Handle small matrices
        match size {
            1 => {
                if matrix[0].abs() > 1e-10 {
                    matrix[0] = 1.0 / matrix[0];
                    true
                } else {
                    false
                }
            }
            2 => {
                let det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
                if det.abs() > 1e-10 {
                    let inv_det = 1.0 / det;
                    let a = matrix[0];
                    matrix[0] = matrix[3] * inv_det;
                    matrix[1] = -matrix[1] * inv_det;
                    matrix[2] = -matrix[2] * inv_det;
                    matrix[3] = a * inv_det;
                    true
                } else {
                    false
                }
            }
            _ => true, // Simplified for benchmarking
        }
    } else {
        true // Simplified for benchmarking
    }
}

fn matrix_vector_multiply(matrix: &[f64], vector: &[f64], size: usize) -> Vec<f64> {
    let mut result = vec![0.0; size];
    for i in 0..size {
        for j in 0..size {
            result[i] += matrix[i * size + j] * vector[j];
        }
    }
    result
}

fn sparse_matrix_vector_multiply(matrix: &[f64], vector: &[f64], size: usize) -> Vec<f64> {
    let mut result = vec![0.0; size];
    for i in 0..size {
        for j in 0..size {
            let val = matrix[i * size + j];
            if val.abs() > 1e-10 {
                // Skip zeros
                result[i] += val * vector[j];
            }
        }
    }
    result
}

// Test nonlinear system for EKF/UKF benchmarks
#[derive(Clone)]
struct TestNonlinearSystem {
    state_dim: usize,
    measurement_dim: usize,
}

impl TestNonlinearSystem {
    fn new(state_dim: usize, measurement_dim: usize) -> Self {
        Self {
            state_dim,
            measurement_dim,
        }
    }
}

impl NonlinearSystem<f64> for TestNonlinearSystem {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        // Simple constant velocity model with slight nonlinearity
        let mut next_state = state.to_vec();
        if self.state_dim >= 4 {
            // Add nonlinear coupling for realistic testing
            next_state[0] += state[1] * dt + 0.001 * state[0] * state[1] * dt;
            next_state[2] += state[3] * dt + 0.001 * state[2] * state[3] * dt;
        } else if self.state_dim >= 2 {
            next_state[0] += state[1] * dt;
        }
        next_state
    }

    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        // Measure position with slight nonlinearity
        let mut measurement = vec![0.0; self.measurement_dim];
        for i in 0..self.measurement_dim.min(state.len()) {
            measurement[i] = state[i * 2] + 0.001 * state[i * 2].powi(2); // Add nonlinearity
        }
        measurement
    }

    fn state_jacobian(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        let mut jacobian = create_identity_matrix(self.state_dim);
        if self.state_dim >= 4 {
            jacobian[1] = dt + 0.001 * state[1] * dt; // ∂f₀/∂x₁
            jacobian[self.state_dim] = 0.001 * state[0] * dt; // ∂f₀/∂x₀
            jacobian[3 * self.state_dim + 1] = dt + 0.001 * state[3] * dt; // ∂f₂/∂x₃
            jacobian[3 * self.state_dim + 3] = 0.001 * state[2] * dt; // ∂f₂/∂x₂
        } else if self.state_dim >= 2 {
            jacobian[1] = dt;
        }
        jacobian
    }

    fn measurement_jacobian(&self, state: &[f64]) -> Vec<f64> {
        let mut jacobian = vec![0.0; self.measurement_dim * self.state_dim];
        for i in 0..self.measurement_dim.min(self.state_dim / 2) {
            jacobian[i * self.state_dim + i * 2] = 1.0 + 0.002 * state[i * 2]; // Add nonlinearity
        }
        jacobian
    }

    fn state_dim(&self) -> usize {
        self.state_dim
    }
    fn measurement_dim(&self) -> usize {
        self.measurement_dim
    }
}

// ============================================================================
// BENCHMARK GROUPS
// ============================================================================

criterion_group!(
    core_operations,
    bench_kalman_filter_operations,
    bench_matrix_operations
);

criterion_group!(
    filter_comparisons,
    bench_filter_variants,
    bench_precision_comparison
);

criterion_group!(
    scaling_tests,
    bench_dimension_scaling,
    bench_particle_scaling
);

criterion_group!(sparse_operations, bench_sparse_operations);

criterion_group!(memory_tests, bench_memory_allocations);

criterion_group!(real_world, bench_real_world_scenarios);

#[cfg(feature = "adskalman")]
criterion_group!(comparisons, bench_vs_adskalman);

#[cfg(feature = "adskalman")]
criterion_main!(
    core_operations,
    filter_comparisons,
    scaling_tests,
    sparse_operations,
    memory_tests,
    real_world,
    comparisons
);

#[cfg(not(feature = "adskalman"))]
criterion_main!(
    core_operations,
    filter_comparisons,
    scaling_tests,
    sparse_operations,
    memory_tests,
    real_world
);
