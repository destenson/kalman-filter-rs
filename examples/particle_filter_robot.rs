//! Particle Filter example: Robot localization with multimodal distributions
//!
//! This example demonstrates particle filter for robot localization where
//! the posterior can be multimodal (multiple possible locations).

use kalman_filter::{ParticleFilter, ResamplingStrategy};
use std::f64::consts::PI;

fn main() {
    println!("=== Particle Filter Robot Localization Example ===\n");
    
    // Robot state: [x, y, theta] (position and heading)
    let state_dim = 3;
    let num_particles = 500;
    
    // Initial uncertainty (robot could be anywhere in a 10x10 area)
    let initial_mean = vec![5.0, 5.0, 0.0];
    let initial_std = vec![3.0, 3.0, PI]; // Large uncertainty
    
    // Process noise (motion uncertainty)
    let process_noise_std = vec![0.1, 0.1, 0.05];
    
    // Measurement noise
    let measurement_noise_std = vec![0.5, 0.5];
    
    // Create particle filter
    let mut pf = ParticleFilter::new(
        state_dim,
        num_particles,
        initial_mean,
        initial_std,
        process_noise_std,
        measurement_noise_std,
        1.0, // dt
    ).unwrap();
    
    // Use systematic resampling for better performance
    pf.set_resampling_strategy(ResamplingStrategy::Systematic);
    
    println!("Robot localization with {} particles", num_particles);
    println!("Initial uncertainty: ±3m in position, ±180° in heading\n");
    
    // Known landmarks in the environment
    let landmarks = vec![
        (2.0, 8.0),   // Landmark 1
        (8.0, 8.0),   // Landmark 2
        (8.0, 2.0),   // Landmark 3
        (2.0, 2.0),   // Landmark 4
    ];
    
    println!("Landmarks at: {:?}", landmarks);
    
    // Motion model: simple differential drive robot
    let motion_model = |state: &[f64], dt: f64| -> Vec<f64> {
        let x = state[0];
        let y = state[1];
        let theta = state[2];
        
        // Control inputs (forward velocity and angular velocity)
        let v = 1.0;  // 1 m/s forward
        let omega = 0.1; // 0.1 rad/s turning
        
        vec![
            x + v * theta.cos() * dt,
            y + v * theta.sin() * dt,
            theta + omega * dt,
        ]
    };
    
    // Likelihood function based on landmark observations
    let likelihood_fn = |state: &[f64], measurement: &[f64]| -> f64 {
        let x = state[0];
        let y = state[1];
        
        // Find closest landmark
        let observed_landmark_idx = measurement[0] as usize;
        if observed_landmark_idx >= landmarks.len() {
            return 1e-10;
        }
        
        let (lx, ly) = landmarks[observed_landmark_idx];
        let measured_dist = measurement[1];
        
        // Calculate expected distance
        let expected_dist = ((x - lx).powi(2) + (y - ly).powi(2)).sqrt();
        
        // Gaussian likelihood
        let diff = measured_dist - expected_dist;
        let sigma = 0.5;
        (-0.5 * diff * diff / (sigma * sigma)).exp()
    };
    
    println!("\nTime | Observed | Distance | Est. X | Est. Y | Est. θ | ESS");
    println!("-----|----------|----------|--------|--------|--------|------");
    
    // Simulate robot motion and observations
    for t in 1..=10 {
        // Predict step (motion update)
        pf.predict(motion_model);
        
        // Simulate landmark observation
        let landmark_idx = t % landmarks.len();
        let (lx, ly) = landmarks[landmark_idx];
        
        // True robot position (for simulation)
        let best = pf.best_particle();
        let true_dist = ((best.state[0] - lx).powi(2) + (best.state[1] - ly).powi(2)).sqrt();
        
        // Observed distance with noise
        let observed_dist = true_dist + 0.3 * ((t as f64) * 0.5).sin();
        
        // Update step with measurement
        let measurement = vec![landmark_idx as f64, observed_dist];
        pf.update(&measurement, likelihood_fn).unwrap();
        
        // Get state estimate (weighted mean)
        let state = pf.mean();
        let ess = pf.effective_sample_size();
        
        println!("{:4} | L{:7} | {:8.2} | {:6.2} | {:6.2} | {:6.2} | {:4.0}",
            t, landmark_idx + 1, observed_dist,
            state[0], state[1], state[2], ess);
    }
    
    // Final estimate
    let final_state = pf.mean();
    let best_particle = pf.best_particle();
    
    println!("\nFinal estimates:");
    println!("  Weighted mean: x={:.2}, y={:.2}, θ={:.2}",
        final_state[0], final_state[1], final_state[2]);
    println!("  Best particle: x={:.2}, y={:.2}, θ={:.2} (weight={:.3})",
        best_particle.state[0], best_particle.state[1], best_particle.state[2],
        best_particle.weight);
    
    // Check for multimodality
    let covariance = pf.covariance();
    let position_variance = covariance[0] + covariance[4]; // σ²_x + σ²_y
    
    if position_variance > 2.0 {
        println!("\n⚠ High variance detected - possible multimodal distribution");
        println!("  The robot might be uncertain between multiple locations");
    } else {
        println!("\n✓ Robot successfully localized with particle filter");
    }
    
    println!("  Handled non-Gaussian, potentially multimodal distributions");
}