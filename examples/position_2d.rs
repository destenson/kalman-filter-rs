//! 2D Position tracking example
//! 
//! This example demonstrates tracking a 2D position (x, y) with velocity.
//! The state vector contains [x, y, vx, vy] and we measure [x, y].
//!
//! This example works with both Vec-based and nalgebra implementations.
//! Use `cargo run --example position_2d` for Vec-based
//! Use `cargo run --example position_2d --features nalgebra` for nalgebra-based
#![allow(unused, non_snake_case)] // DO NOT CHANGE

use rand::Rng;

#[cfg(not(feature = "nalgebra"))]
use kalman_filter::{KalmanFilter, KalmanFilterBuilder};

#[cfg(feature = "nalgebra")]
use kalman_filter::StaticKalmanFilter;
#[cfg(feature = "nalgebra")]
use nalgebra::{SMatrix, SVector};

#[cfg(not(feature = "nalgebra"))]
fn main() {
    println!("2D Position Tracking with Kalman Filter (Vec-based)");
    println!("====================================================");
    
    // Time step
    let dt = 0.1;
    
    // State transition matrix for 2D constant velocity model
    // State: [x, y, vx, vy]
    let f = vec![
        1.0, 0.0, dt,  0.0,  // x = x + vx * dt
        0.0, 1.0, 0.0, dt,   // y = y + vy * dt
        0.0, 0.0, 1.0, 0.0,  // vx = vx
        0.0, 0.0, 0.0, 1.0,  // vy = vy
    ];
    
    // Initial state: origin with velocity (1, 0.5) m/s
    let x0 = vec![0.0, 0.0, 1.0, 0.5];
    
    // Initial covariance
    let p0 = vec![
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ];
    
    // Process noise (acceleration uncertainty)
    let q_val = 0.01;  // Process noise strength
    let q = vec![
        q_val*dt*dt*dt/3.0, 0.0, q_val*dt*dt/2.0, 0.0,
        0.0, q_val*dt*dt*dt/3.0, 0.0, q_val*dt*dt/2.0,
        q_val*dt*dt/2.0, 0.0, q_val*dt, 0.0,
        0.0, q_val*dt*dt/2.0, 0.0, q_val*dt,
    ];
    
    // Observation matrix (we measure x and y positions)
    let h = vec![
        1.0, 0.0, 0.0, 0.0,  // Measure x
        0.0, 1.0, 0.0, 0.0,  // Measure y
    ];
    
    // Measurement noise
    let r_val = 0.1;  // Measurement noise strength
    let r = vec![
        r_val, 0.0,
        0.0, r_val,
    ];
    
    // Build the Kalman filter
    let mut kf = KalmanFilterBuilder::<f64>::new(4, 2)
        .initial_state(x0)
        .initial_covariance(p0)
        .transition_matrix(f)
        .process_noise(q)
        .observation_matrix(h)
        .measurement_noise(r)
        .build()
        .expect("Failed to build Kalman filter");
    
    // Simulate a moving object with circular motion
    let mut rng = rand::thread_rng();
    let omega = 0.1;  // Angular velocity for circular motion
    let radius = 5.0;
    
    println!("\nTime |  True X  |  True Y  | Meas. X  | Meas. Y  | Est. X   | Est. Y   | Est. Vx  | Est. Vy");
    println!("-----|----------|----------|----------|----------|----------|----------|----------|--------");
    
    for step in 0..100 {
        let t = step as f64 * dt;
        
        // True position (circular motion)
        let true_x = radius * (omega * t).cos();
        let true_y = radius * (omega * t).sin();
        
        // Generate noisy measurements
        let noise_x = rng.gen_range(-0.3..0.3);
        let noise_y = rng.gen_range(-0.3..0.3);
        let measured_x = true_x + noise_x;
        let measured_y = true_y + noise_y;
        
        // Kalman filter predict step
        kf.predict();
        
        // Kalman filter update step
        kf.update(&[measured_x, measured_y]).expect("Update failed");
        
        // Get estimated state
        let state = kf.state();
        
        // Print results every 20 steps
        if step % 20 == 0 {
            println!(
                "{:4.1} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:7.3}",
                t, true_x, true_y, measured_x, measured_y,
                state[0], state[1], state[2], state[3]
            );
        }
    }
    
    println!("\nFinal state covariance (diagonal elements):");
    let p = kf.covariance();
    println!("X position variance:  {:.6}", p[0 * 4 + 0]);
    println!("Y position variance:  {:.6}", p[1 * 4 + 1]);
    println!("X velocity variance:  {:.6}", p[2 * 4 + 2]);
    println!("Y velocity variance:  {:.6}", p[3 * 4 + 3]);
    
    println!("\nThe filter tracked the 2D position and estimated velocities");
    println!("from noisy position measurements.");
}

#[cfg(feature = "nalgebra")]
fn main() {
    println!("2D Position Tracking with Kalman Filter (nalgebra-based)");
    println!("=========================================================");
    
    // Time step
    let dt = 0.1;
    
    // State transition matrix for 2D constant velocity model
    // State: [x, y, vx, vy]
    let F = SMatrix::<f64, 4, 4>::from([
        [1.0, 0.0, dt,  0.0],  // x = x + vx * dt
        [0.0, 1.0, 0.0, dt ],  // y = y + vy * dt
        [0.0, 0.0, 1.0, 0.0],  // vx = vx
        [0.0, 0.0, 0.0, 1.0],  // vy = vy
    ]);
    
    // Initial state: origin with velocity (1, 0.5) m/s
    let x0 = SVector::<f64, 4>::from([0.0, 0.0, 1.0, 0.5]);
    
    // Initial covariance
    let P0 = SMatrix::<f64, 4, 4>::from([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    
    // Process noise (acceleration uncertainty)
    let q_val = 0.01;  // Process noise strength
    let Q = SMatrix::<f64, 4, 4>::from([
        [q_val*dt*dt*dt/3.0, 0.0, q_val*dt*dt/2.0, 0.0],
        [0.0, q_val*dt*dt*dt/3.0, 0.0, q_val*dt*dt/2.0],
        [q_val*dt*dt/2.0, 0.0, q_val*dt, 0.0],
        [0.0, q_val*dt*dt/2.0, 0.0, q_val*dt],
    ]);
    
    // Observation matrix (we measure x and y positions)
    let H = SMatrix::<f64, 2, 4>::from_row_slice(&[
        1.0, 0.0, 0.0, 0.0,  // Measure x
        0.0, 1.0, 0.0, 0.0,  // Measure y
    ]);
    
    // Measurement noise
    let r_val = 0.1;  // Measurement noise strength
    let R = SMatrix::<f64, 2, 2>::from([
        [r_val, 0.0],
        [0.0, r_val],
    ]);
    
    // Create the Kalman filter
    let mut kf = StaticKalmanFilter::new(x0, P0, F, Q, H, R);
    
    // Simulate a moving object with circular motion
    let mut rng = rand::thread_rng();
    let omega = 0.1;  // Angular velocity for circular motion
    let radius = 5.0;
    
    println!("\nTime |  True X  |  True Y  | Meas. X  | Meas. Y  | Est. X   | Est. Y   | Est. Vx  | Est. Vy");
    println!("-----|----------|----------|----------|----------|----------|----------|----------|--------");
    
    for step in 0..100 {
        let t = step as f64 * dt;
        
        // True position (circular motion)
        let true_x = radius * (omega * t).cos();
        let true_y = radius * (omega * t).sin();
        
        // Generate noisy measurements
        let noise_x = rng.gen_range(-0.3..0.3);
        let noise_y = rng.gen_range(-0.3..0.3);
        let measured_x = true_x + noise_x;
        let measured_y = true_y + noise_y;
        
        // Kalman filter predict step
        kf.predict();
        
        // Kalman filter update step
        let measurement = SVector::<f64, 2>::from([measured_x, measured_y]);
        kf.update(&measurement).expect("Update failed");
        
        // Get estimated state
        let state = kf.state();
        
        // Print results every 20 steps
        if step % 20 == 0 {
            println!(
                "{:4.1} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:8.3} | {:7.3}",
                t, true_x, true_y, measured_x, measured_y,
                state[0], state[1], state[2], state[3]
            );
        }
    }
    
    println!("\nFinal state covariance (diagonal elements):");
    let P = kf.covariance();
    println!("X position variance:  {:.6}", P[(0, 0)]);
    println!("Y position variance:  {:.6}", P[(1, 1)]);
    println!("X velocity variance:  {:.6}", P[(2, 2)]);
    println!("Y velocity variance:  {:.6}", P[(3, 3)]);
    
    println!("\nThe filter tracked the 2D position and estimated velocities");
    println!("from noisy position measurements.");
}
