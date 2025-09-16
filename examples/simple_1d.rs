//! Simple 1D Kalman filter example
//!
//! This example demonstrates tracking a 1D position with noisy measurements.
//! The system models constant velocity motion where:
//! - State: [position, velocity]
//! - Measurement: position only
//!
//! This example works with both Vec-based and nalgebra implementations.
//! Use `cargo run --example simple_1d` for Vec-based
//! Use `cargo run --example simple_1d --features nalgebra` for nalgebra-based
#![allow(unused, non_snake_case)]

use rand::Rng;

#[cfg(not(feature = "nalgebra"))]
use kalman_filters::{KalmanFilter, KalmanFilterBuilder};

#[cfg(feature = "nalgebra")]
use kalman_filters::StaticKalmanFilter;
#[cfg(feature = "nalgebra")]
use nalgebra::{SMatrix, SVector};

#[cfg(not(feature = "nalgebra"))]
fn main() {
    println!("Simple 1D Kalman Filter Example (Vec-based)");
    println!("============================================");

    // Time step
    let dt = 0.1;

    // Create state transition matrix (constant velocity model)
    // [position_new] = [1  dt] * [position_old]
    // [velocity_new]   [0  1 ]   [velocity_old]
    let f = vec![1.0, dt, 0.0, 1.0];

    // Initial state: position = 0, velocity = 1 m/s
    let x0 = vec![0.0, 1.0];

    // Initial covariance (uncertainty)
    let p0 = vec![1.0, 0.0, 0.0, 1.0];

    // Process noise (how much the system can change randomly)
    let q = vec![0.001, 0.0, 0.0, 0.001];

    // Observation matrix (we only measure position)
    let h = vec![1.0, 0.0];

    // Measurement noise
    let r = vec![0.1];

    // Build the Kalman filter
    let mut kf = KalmanFilterBuilder::<f64>::new(2, 1)
        .initial_state(x0)
        .initial_covariance(p0)
        .transition_matrix(f)
        .process_noise(q)
        .observation_matrix(h)
        .measurement_noise(r)
        .build()
        .expect("Failed to build Kalman filter");

    // Simulate true system and noisy measurements
    let mut rng = rand::thread_rng();
    let true_velocity = 1.0;
    let measurement_noise_std = 0.3;

    println!("\nTime | True Pos | Measured | Estimated | Est. Vel");
    println!("-----|----------|----------|-----------|----------");

    for step in 0..50 {
        let t = step as f64 * dt;
        let true_position = true_velocity * t;

        // Generate noisy measurement
        let noise = rng.gen_range(-measurement_noise_std..measurement_noise_std);
        let measured_position = true_position + noise;

        // Kalman filter predict step
        kf.predict();

        // Kalman filter update step
        kf.update(&[measured_position]).expect("Update failed");

        // Get estimated state
        let state = kf.state();
        let estimated_position = state[0];
        let estimated_velocity = state[1];

        // Print results every 10 steps
        if step % 10 == 0 {
            println!(
                "{:4.1} | {:8.3} | {:8.3} | {:9.3} | {:8.3}",
                t, true_position, measured_position, estimated_position, estimated_velocity
            );
        }
    }

    println!("\nFinal state covariance:");
    let p = kf.covariance();
    println!("Position variance: {:.6}", p[0]);
    println!("Velocity variance: {:.6}", p[3]);

    println!("\nThe Kalman filter successfully tracked the position and");
    println!("estimated the velocity from noisy position measurements.");
}

#[cfg(feature = "nalgebra")]
fn main() {
    println!("Simple 1D Kalman Filter Example (nalgebra-based)");
    println!("=================================================");

    // Time step
    let dt = 0.1;

    // Create state transition matrix (constant velocity model)
    let F = SMatrix::<f64, 2, 2>::from_row_slice(&[1.0, dt, 0.0, 1.0]);

    // Initial state: position = 0, velocity = 1 m/s
    let x0 = SVector::<f64, 2>::from([0.0, 1.0]);

    // Initial covariance (uncertainty)
    let P0 = SMatrix::<f64, 2, 2>::from_row_slice(&[1.0, 0.0, 0.0, 1.0]);

    // Process noise (how much the system can change randomly)
    let Q = SMatrix::<f64, 2, 2>::from_row_slice(&[0.001, 0.0, 0.0, 0.001]);

    // Observation matrix (we only measure position)
    let H = SMatrix::<f64, 1, 2>::from_row_slice(&[1.0, 0.0]);

    // Measurement noise
    let R = SMatrix::<f64, 1, 1>::from_row_slice(&[0.1]);

    // Create the Kalman filter
    let mut kf = StaticKalmanFilter::new(x0, P0, F, Q, H, R);

    // Simulate true system and noisy measurements
    let mut rng = rand::thread_rng();
    let true_velocity = 1.0;
    let measurement_noise_std = 0.3;

    println!("\nTime | True Pos | Measured | Estimated | Est. Vel");
    println!("-----|----------|----------|-----------|----------");

    for step in 0..50 {
        let t = step as f64 * dt;
        let true_position = true_velocity * t;

        // Generate noisy measurement
        let noise = rng.gen_range(-measurement_noise_std..measurement_noise_std);
        let measured_position = true_position + noise;

        // Kalman filter predict step
        kf.predict();

        // Kalman filter update step
        let measurement = SVector::<f64, 1>::from([measured_position]);
        kf.update(&measurement).expect("Update failed");

        // Get estimated state
        let state = kf.state();
        let estimated_position = state[0];
        let estimated_velocity = state[1];

        // Print results every 10 steps
        if step % 10 == 0 {
            println!(
                "{:4.1} | {:8.3} | {:8.3} | {:9.3} | {:8.3}",
                t, true_position, measured_position, estimated_position, estimated_velocity
            );
        }
    }

    println!("\nFinal state covariance:");
    let P = kf.covariance();
    println!("Position variance: {:.6}", P[(0, 0)]);
    println!("Velocity variance: {:.6}", P[(1, 1)]);

    println!("\nThe Kalman filter successfully tracked the position and");
    println!("estimated the velocity from noisy position measurements.");
}
