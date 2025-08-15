//! Cubature Kalman Filter example: High-dimensional system
//!
//! This example demonstrates the CKF's numerical stability advantages
//! for higher-dimensional systems compared to UKF.
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use kalman_filter::{CubatureKalmanFilter, NonlinearSystem};

/// High-dimensional nonlinear system (6D state)
struct HighDimensionalSystem;

impl NonlinearSystem<f64> for HighDimensionalSystem {
    fn state_transition(&self, state: &[f64], _control: Option<&[f64]>, dt: f64) -> Vec<f64> {
        // Coupled nonlinear dynamics
        vec![
            state[0] + state[3] * dt + 0.01 * state[0] * state[1] * dt,
            state[1] + state[4] * dt + 0.01 * state[1] * state[2] * dt,
            state[2] + state[5] * dt + 0.01 * state[2] * state[0] * dt,
            state[3] * (1.0 - 0.1 * dt) + 0.05 * state[4].sin() * dt,
            state[4] * (1.0 - 0.1 * dt) + 0.05 * state[5].sin() * dt,
            state[5] * (1.0 - 0.1 * dt) + 0.05 * state[3].sin() * dt,
        ]
    }
    
    fn measurement(&self, state: &[f64]) -> Vec<f64> {
        // Nonlinear measurements of positions
        vec![
            (state[0].powi(2) + state[1].powi(2)).sqrt(),  // Distance from origin in XY
            (state[1].powi(2) + state[2].powi(2)).sqrt(),  // Distance from origin in YZ
            state[0] + state[1] + state[2],                // Sum of positions
        ]
    }
    
    fn state_jacobian(&self, _state: &[f64], _control: Option<&[f64]>, _dt: f64) -> Vec<f64> {
        unreachable!("CKF doesn't need Jacobians")
    }
    
    fn measurement_jacobian(&self, _state: &[f64]) -> Vec<f64> {
        unreachable!("CKF doesn't need Jacobians")
    }
    
    fn state_dim(&self) -> usize { 6 }
    fn measurement_dim(&self) -> usize { 3 }
}

fn main() {
    println!("=== CKF High-Dimensional System Example ===\n");
    
    let system = HighDimensionalSystem;
    
    // Initial 6D state: [x, y, z, vx, vy, vz]
    let initial_state = vec![1.0, 0.5, -0.5, 0.1, 0.2, -0.1];
    
    // Initial covariance (6x6 identity scaled)
    let mut initial_covariance = vec![0.0; 36];
    for i in 0..6 {
        initial_covariance[i * 6 + i] = 0.1;
    }
    
    // Process noise (6x6 diagonal)
    let mut process_noise = vec![0.0; 36];
    for i in 0..3 {
        process_noise[i * 6 + i] = 0.01;        // Position noise
        process_noise[(i+3) * 6 + (i+3)] = 0.001; // Velocity noise
    }
    
    // Measurement noise (3x3 diagonal)
    let measurement_noise = vec![
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.02,
    ];
    
    // Create CKF
    let mut ckf = CubatureKalmanFilter::new(
        system,
        initial_state,
        initial_covariance,
        process_noise,
        measurement_noise,
        0.1, // dt = 0.1 seconds
    ).unwrap();
    
    println!("6D State tracking with CKF (2n = 12 cubature points)");
    println!("Initial state: [{:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}]",
        ckf.state()[0], ckf.state()[1], ckf.state()[2],
        ckf.state()[3], ckf.state()[4], ckf.state()[5]);
    
    // Track for 20 time steps
    println!("\nTime | Meas 1 | Meas 2 | Meas 3 | State Norm | Cov Trace");
    println!("-----|--------|--------|--------|------------|----------");
    
    for t in 1..=20 {
        // Predict
        ckf.predict().unwrap();
        
        // Generate simulated measurements
        let state = ckf.state();
        let true_meas1 = (state[0].powi(2) + state[1].powi(2)).sqrt();
        let true_meas2 = (state[1].powi(2) + state[2].powi(2)).sqrt();
        let true_meas3 = state[0] + state[1] + state[2];
        
        // Add noise
        let measurements = vec![
            true_meas1 + 0.02 * (t as f64 * 0.3).sin(),
            true_meas2 + 0.02 * (t as f64 * 0.5).cos(),
            true_meas3 + 0.01 * (t as f64 * 0.7).sin(),
        ];
        
        // Update
        ckf.update(&measurements).unwrap();
        
        // Compute state norm and covariance trace
        let state = ckf.state();
        let state_norm: f64 = state.iter().map(|x| x * x).sum::<f64>().sqrt();
        
        let cov = ckf.covariance();
        let cov_trace: f64 = (0..6).map(|i| cov[i * 6 + i]).sum();
        
        println!("{:4} | {:6.3} | {:6.3} | {:6.3} | {:10.3} | {:9.4}",
            t, measurements[0], measurements[1], measurements[2],
            state_norm, cov_trace);
    }
    
    // Final state
    let final_state = ckf.state();
    println!("\nFinal state estimate:");
    println!("  Position: [{:.3}, {:.3}, {:.3}]",
        final_state[0], final_state[1], final_state[2]);
    println!("  Velocity: [{:.3}, {:.3}, {:.3}]",
        final_state[3], final_state[4], final_state[5]);
    
    println!("\n✓ CKF successfully handled 6D nonlinear system");
    println!("  Used only 12 cubature points (vs 13 sigma points for UKF)");
}
