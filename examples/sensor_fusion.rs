//! Sensor fusion example
//!
//! This example demonstrates fusing data from multiple sensors with different
//! characteristics to track a moving object. We simulate:
//! - GPS sensor: Provides position measurements with moderate noise
//! - IMU accelerometer: Provides acceleration measurements  
//! - Radar: Provides position and velocity with different noise characteristics
//!
//! The Kalman filter combines these measurements optimally based on their
//! respective noise characteristics.
#![allow(unused, non_snake_case)]

use rand::Rng;
use rand_distr::{Distribution, Normal};

#[cfg(not(feature = "nalgebra"))]
use kalman_filters::{KalmanFilter, KalmanFilterBuilder};

#[cfg(feature = "nalgebra")]
use kalman_filters::StaticKalmanFilter;
#[cfg(feature = "nalgebra")]
use nalgebra::{SMatrix, SVector};

#[cfg(not(feature = "nalgebra"))]
fn main() {
    println!("Multi-Sensor Fusion Example (Vec-based)");
    println!("========================================");
    println!("Fusing GPS, IMU, and Radar measurements\n");

    // Time step
    let dt = 0.1f64;

    // State vector: [x, y, vx, vy, ax, ay] - position, velocity, acceleration
    // State transition matrix (constant acceleration model)
    let f = vec![
        1.0,
        0.0,
        dt,
        0.0,
        dt * dt / 2.0,
        0.0, // x
        0.0,
        1.0,
        0.0,
        dt,
        0.0,
        dt * dt / 2.0, // y
        0.0,
        0.0,
        1.0,
        0.0,
        dt,
        0.0, // vx
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        dt, // vy
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0, // ax
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0, // ay
    ];

    // Initial state: at origin, moving with velocity (2, 1) m/s
    let x0 = vec![0.0, 0.0, 2.0, 1.0, 0.0, 0.0];

    // Initial covariance
    let p0 = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];

    // Process noise (uncertainty in acceleration)
    let q_acc = 0.1; // Acceleration process noise
    let mut q = vec![0.0; 36];
    // Fill process noise matrix
    q[0 * 6 + 0] = dt.powi(4) / 4.0 * q_acc; // x-x
    q[1 * 6 + 1] = dt.powi(4) / 4.0 * q_acc; // y-y
    q[2 * 6 + 2] = dt.powi(2) * q_acc; // vx-vx
    q[3 * 6 + 3] = dt.powi(2) * q_acc; // vy-vy
    q[4 * 6 + 4] = q_acc; // ax-ax
    q[5 * 6 + 5] = q_acc; // ay-ay

    // GPS observation matrix (measures x, y position only)
    let h_gps = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // x
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // y
    ];

    // GPS measurement noise (moderate position uncertainty)
    let r_gps = vec![
        0.5, 0.0, // GPS position noise
        0.0, 0.5,
    ];

    // Radar observation matrix (measures position and velocity)
    let h_radar = vec![
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // x
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // y
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // vx
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // vy
    ];

    // Radar measurement noise (better than GPS for position, includes velocity)
    let r_radar = vec![
        0.2, 0.0, 0.0, 0.0, // Better position accuracy
        0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, // Velocity measurements
        0.0, 0.0, 0.0, 0.3,
    ];

    // IMU observation matrix (measures acceleration)
    let h_imu = vec![
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, // ax
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, // ay
    ];

    // IMU measurement noise
    let r_imu = vec![
        0.05, 0.0, // Accelerometer noise
        0.0, 0.05,
    ];

    // Build Kalman filter (we'll switch observation matrices as needed)
    let mut kf = KalmanFilter::<f64>::initialize(
        6,
        2, // 6 states, 2 measurements (will adjust for each sensor)
        x0,
        p0,
        f,
        q,
        h_gps.clone(), // Start with GPS
        r_gps.clone(),
    )
    .expect("Failed to build Kalman filter");

    // Simulation parameters
    let mut rng = rand::thread_rng();
    let gps_noise = Normal::new(0.0, 0.5).unwrap();
    let radar_noise = Normal::new(0.0, 0.2).unwrap();
    let imu_noise = Normal::new(0.0, 0.05).unwrap();

    // True trajectory: circular motion with changing acceleration
    let omega = 0.2; // Angular frequency
    let radius = 10.0;

    println!("Time | True X | True Y | GPS X  | GPS Y  | Radar X| Radar Y| Est. X | Est. Y | Est. Vx| Est. Vy");
    println!("-----|--------|--------|--------|--------|--------|--------|--------|--------|--------|--------");

    for step in 0..100 {
        let t = step as f64 * dt;

        // True state (circular motion)
        let true_x = radius * t.cos() * 0.1;
        let true_y = radius * t.sin() * 0.1;
        let true_vx = -radius * omega * t.sin() * 0.1;
        let true_vy = radius * omega * t.cos() * 0.1;
        let true_ax = -radius * omega * omega * t.cos() * 0.1;
        let true_ay = -radius * omega * omega * t.sin() * 0.1;

        // Predict step
        kf.predict();

        // Sensor measurements (alternating between sensors)
        match step % 3 {
            0 => {
                // GPS measurement (every 3rd step)
                let gps_x = true_x + gps_noise.sample(&mut rng);
                let gps_y = true_y + gps_noise.sample(&mut rng);

                // Update observation model for GPS
                kf.measurement_dim = 2;
                kf.H = h_gps.clone();
                kf.R = r_gps.clone();

                kf.update(&[gps_x, gps_y]).expect("GPS update failed");

                if step % 10 == 0 {
                    print!(
                        "{:4.1} | {:6.2} | {:6.2} | {:6.2} | {:6.2} |",
                        t, true_x, true_y, gps_x, gps_y
                    );
                }
            }
            1 => {
                // Radar measurement
                let radar_x = true_x + radar_noise.sample(&mut rng);
                let radar_y = true_y + radar_noise.sample(&mut rng);
                let radar_vx = true_vx + radar_noise.sample(&mut rng) * 1.5;
                let radar_vy = true_vy + radar_noise.sample(&mut rng) * 1.5;

                // Update observation model for Radar
                kf.measurement_dim = 4;
                kf.H = h_radar.clone();
                kf.R = r_radar.clone();

                kf.update(&[radar_x, radar_y, radar_vx, radar_vy])
                    .expect("Radar update failed");

                if step % 10 == 1 {
                    print!(" {:6.2} | {:6.2} |", radar_x, radar_y);
                }
            }
            _ => {
                // IMU measurement
                let imu_ax = true_ax + imu_noise.sample(&mut rng);
                let imu_ay = true_ay + imu_noise.sample(&mut rng);

                // Update observation model for IMU
                kf.measurement_dim = 2;
                kf.H = h_imu.clone();
                kf.R = r_imu.clone();

                kf.update(&[imu_ax, imu_ay]).expect("IMU update failed");
            }
        }

        // Print state estimate periodically
        if step % 10 == 0 || step % 10 == 1 {
            let state = kf.state();
            println!(
                " {:6.2} | {:6.2} | {:6.2} | {:6.2}",
                state[0], state[1], state[2], state[3]
            );
        }
    }

    println!("\nFinal state covariance (diagonal elements):");
    let p = kf.covariance();
    println!("X position variance:   {:.6}", p[0 * 6 + 0]);
    println!("Y position variance:   {:.6}", p[1 * 6 + 1]);
    println!("X velocity variance:   {:.6}", p[2 * 6 + 2]);
    println!("Y velocity variance:   {:.6}", p[3 * 6 + 3]);
    println!("X acceleration variance: {:.6}", p[4 * 6 + 4]);
    println!("Y acceleration variance: {:.6}", p[5 * 6 + 5]);

    println!("\nSensor fusion successfully combined GPS, Radar, and IMU measurements");
    println!("to provide optimal state estimation with different sensor characteristics.");
}

#[cfg(feature = "nalgebra")]
fn main() {
    println!("Multi-Sensor Fusion Example (nalgebra-based)");
    println!("=============================================");
    println!("Fusing GPS, IMU, and Radar measurements\n");

    // Time step
    let dt = 0.1;

    // State transition matrix (constant acceleration model)
    let F = SMatrix::<f64, 6, 6>::from_row_slice(&[
        1.0,
        0.0,
        dt,
        0.0,
        dt * dt / 2.0,
        0.0, // x
        0.0,
        1.0,
        0.0,
        dt,
        0.0,
        dt * dt / 2.0, // y
        0.0,
        0.0,
        1.0,
        0.0,
        dt,
        0.0, // vx
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        dt, // vy
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0, // ax
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0, // ay
    ]);

    // Initial state: at origin, moving with velocity (2, 1) m/s
    let x0 = SVector::<f64, 6>::from([0.0, 0.0, 2.0, 1.0, 0.0, 0.0]);

    // Initial covariance
    let P0 = SMatrix::<f64, 6, 6>::identity();

    // Process noise
    let q_acc = 0.1;
    let mut Q = SMatrix::<f64, 6, 6>::zeros();
    Q[(0, 0)] = dt.powi(4) / 4.0 * q_acc;
    Q[(1, 1)] = dt.powi(4) / 4.0 * q_acc;
    Q[(2, 2)] = dt.powi(2) * q_acc;
    Q[(3, 3)] = dt.powi(2) * q_acc;
    Q[(4, 4)] = q_acc;
    Q[(5, 5)] = q_acc;

    // For nalgebra version, we'll create separate filters for each sensor type
    // since we can't dynamically change dimensions

    // GPS Kalman Filter (2 measurements)
    let H_gps = SMatrix::<f64, 2, 6>::from_row_slice(&[
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    ]);
    let R_gps = SMatrix::<f64, 2, 2>::from_row_slice(&[0.5, 0.0, 0.0, 0.5]);

    let mut kf = StaticKalmanFilter::new(x0, P0, F, Q, H_gps, R_gps);

    // Simulation parameters
    let mut rng = rand::thread_rng();
    let gps_noise = Normal::new(0.0, 0.5).unwrap();

    // True trajectory
    let omega = 0.2;
    let radius = 10.0;

    println!("Time | True X | True Y | GPS X  | GPS Y  | Est. X | Est. Y | Est. Vx| Est. Vy");
    println!("-----|--------|--------|--------|--------|--------|--------|--------|--------");

    for step in 0..50 {
        let t = step as f64 * dt;

        // True state
        let true_x = radius * t.cos() * 0.1;
        let true_y = radius * t.sin() * 0.1;

        // Predict
        kf.predict();

        // GPS measurement
        let gps_x = true_x + gps_noise.sample(&mut rng);
        let gps_y = true_y + gps_noise.sample(&mut rng);
        let measurement = SVector::<f64, 2>::from([gps_x, gps_y]);

        // Update
        kf.update(&measurement).expect("Update failed");

        // Print results
        if step % 10 == 0 {
            let state = kf.state();
            println!(
                "{:4.1} | {:6.2} | {:6.2} | {:6.2} | {:6.2} | {:6.2} | {:6.2} | {:6.2} | {:6.2}",
                t, true_x, true_y, gps_x, gps_y, state[0], state[1], state[2], state[3]
            );
        }
    }

    println!("\nFinal state covariance (diagonal elements):");
    let P = kf.covariance();
    println!("X position variance:   {:.6}", P[(0, 0)]);
    println!("Y position variance:   {:.6}", P[(1, 1)]);
    println!("X velocity variance:   {:.6}", P[(2, 2)]);
    println!("Y velocity variance:   {:.6}", P[(3, 3)]);
    println!("X acceleration variance: {:.6}", P[(4, 4)]);
    println!("Y acceleration variance: {:.6}", P[(5, 5)]);

    println!(
        "\nNote: nalgebra version simplified to GPS-only due to static dimension constraints."
    );
    println!(
        "Full multi-sensor fusion would require separate filter instances or dynamic matrices."
    );
}
