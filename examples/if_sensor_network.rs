//! Distributed sensor network example using Information Filter
//! 
//! This example demonstrates a 20-node sensor network tracking a moving target.
//! Each sensor has limited range and can only communicate with neighbors.
//! The Information Filter enables efficient distributed fusion.
#![allow(unused, non_snake_case)] // DO NOT CHANGE!

use kalman_filter::information::{
    DistributedInformationFilter, 
    AverageConsensus, 
    ConsensusAlgorithm,
    InformationForm,
};
use rand::Rng;
use std::collections::HashMap;

/// Simulate a sensor network tracking a moving target
fn main() {
    println!("Distributed Sensor Network with Information Filter");
    println!("==================================================");
    println!("20 sensors tracking a moving target\n");
    
    // Network parameters
    const NUM_SENSORS: usize = 20;
    const GRID_SIZE: f64 = 100.0;
    const SENSOR_RANGE: f64 = 30.0;
    const COMM_RANGE: f64 = 40.0;
    
    // Target parameters
    let target_speed = 5.0;  // m/s
    let dt = 0.5;  // Time step
    
    // State: [x, y, vx, vy] - position and velocity
    let state_dim = 4;
    
    // Create distributed filter
    let mut dif = DistributedInformationFilter::<f64>::new(state_dim);
    
    // Place sensors in a grid
    let mut sensor_positions = Vec::new();
    let grid_size = ((NUM_SENSORS as f64).sqrt().ceil()) as usize;
    let grid_spacing = GRID_SIZE / grid_size as f64;
    
    for i in 0..grid_size {
        for j in 0..grid_size {
            if sensor_positions.len() < NUM_SENSORS {
                let x = (i as f64 + 0.5) * grid_spacing;
                let y = (j as f64 + 0.5) * grid_spacing;
                sensor_positions.push((x, y));
            }
        }
    }
    
    println!("Sensor Network Configuration:");
    println!("- {} sensors in {}x{} grid", NUM_SENSORS, grid_size, grid_size);
    println!("- Sensing range: {} m", SENSOR_RANGE);
    println!("- Communication range: {} m", COMM_RANGE);
    
    // Initialize nodes with weak prior
    let weak_info = 0.001;  // Weak prior information
    for i in 0..NUM_SENSORS {
        let Y_init = vec![
            weak_info, 0.0, 0.0, 0.0,
            0.0, weak_info, 0.0, 0.0,
            0.0, 0.0, weak_info, 0.0,
            0.0, 0.0, 0.0, weak_info,
        ];
        let y_init = vec![0.0, 0.0, 0.0, 0.0];
        
        dif.add_node(i, Y_init, y_init).unwrap();
    }
    
    // Connect nodes within communication range
    let mut num_connections = 0;
    for i in 0..sensor_positions.len() {
        for j in i+1..sensor_positions.len() {
            let (x1, y1) = sensor_positions[i];
            let (x2, y2) = sensor_positions[j];
            let dist = ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt();
            
            if dist <= COMM_RANGE {
                // Add small random delay (0-2 time steps)
                let delay = rand::thread_rng().gen_range(0..3);
                dif.connect_nodes(i, j, delay).unwrap();
                num_connections += 1;
            }
        }
    }
    
    println!("- {} communication links established", num_connections);
    
    // Target trajectory (circular motion)
    let mut rng = rand::thread_rng();
    let center_x = GRID_SIZE / 2.0;
    let center_y = GRID_SIZE / 2.0;
    let radius = GRID_SIZE / 3.0;
    let omega = 0.1;  // Angular velocity
    
    println!("\nSimulation Results:");
    println!("Time | True X | True Y | Sensors | Consensus X | Consensus Y | Error");
    println!("-----|--------|--------|---------|-------------|-------------|-------");
    
    // Simulation loop
    for step in 0..20 {
        let t = step as f64 * dt;
        
        // True target position
        let true_x = center_x + radius * (omega * t).cos();
        let true_y = center_y + radius * (omega * t).sin();
        let true_vx = -radius * omega * (omega * t).sin();
        let true_vy = radius * omega * (omega * t).cos();
        
        // Sensors that can see the target
        let mut detecting_sensors = Vec::new();
        
        for (sensor_id, &(sx, sy)) in sensor_positions.iter().enumerate() {
            let dist_to_target = ((true_x - sx).powi(2) + (true_y - sy).powi(2)).sqrt();
            
            if dist_to_target <= SENSOR_RANGE {
                detecting_sensors.push(sensor_id);
                
                // Generate noisy measurement
                let noise_std = 1.0 + dist_to_target * 0.1;  // Distance-dependent noise
                let meas_x = true_x + rng.gen_range(-noise_std..noise_std);
                let meas_y = true_y + rng.gen_range(-noise_std..noise_std);
                
                // Observation matrix (position only)
                let H = vec![
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                ];
                
                // Measurement noise (distance-dependent)
                let r_val = noise_std.powi(2);
                let R = vec![
                    r_val, 0.0,
                    0.0, r_val,
                ];
                
                // Local update
                dif.local_update(sensor_id, &[meas_x, meas_y], &H, &R).unwrap();
            }
        }
        
        // Process messages (information propagation)
        for _ in 0..3 {  // Multiple rounds for better convergence
            dif.process_messages().unwrap();
        }
        
        // Run consensus algorithm
        let topology = dif.topology.clone();
        let mut local_states = HashMap::new();
        for (node_id, node) in &dif.nodes {
            local_states.insert(*node_id, node.local_state.clone());
        }
        
        let mut consensus = AverageConsensus::<f64>::new_metropolis(&topology, state_dim);
        for _ in 0..5 {  // Consensus iterations
            consensus.iterate(&mut local_states, &topology).unwrap();
        }
        
        // Get consensus estimate
        if let Some(consensus_state) = consensus.get_consensus() {
            if let Ok(state) = consensus_state.recover_state() {
                let error = ((state[0] - true_x).powi(2) + (state[1] - true_y).powi(2)).sqrt();
                
                if step % 2 == 0 {  // Print every other step
                    println!("{:4.1} | {:6.1} | {:6.1} | {:7} | {:11.1} | {:11.1} | {:6.2}",
                            t, true_x, true_y, detecting_sensors.len(),
                            state[0], state[1], error);
                }
            }
        }
    }
    
    // Network statistics
    let stats = dif.get_network_stats();
    println!("\nNetwork Statistics:");
    println!("- Nodes: {}", stats.num_nodes);
    println!("- Edges: {}", stats.num_edges);
    println!("- Average degree: {:.1}", stats.avg_degree);
    println!("- Maximum degree: {}", stats.max_degree);
    
    println!("\nDistributed Information Filter successfully tracked target");
    println!("across sparse sensor network with limited communication.");

    
    multi_rate_example();
}

/// Simulate node failures
fn simulate_node_failure(dif: &mut DistributedInformationFilter<f64>, failed_nodes: Vec<usize>) {
    println!("\nSimulating node failures: {:?}", failed_nodes);
    
    for node_id in failed_nodes {
        // Remove node from network
        if dif.nodes.remove(&node_id).is_some() {
            // Remove from topology
            dif.topology.remove(&node_id);
            
            // Remove connections to failed node
            for neighbors in dif.topology.values_mut() {
                neighbors.retain(|&n| n != node_id);
            }
            
            // Update remaining nodes' neighbor lists
            for node in dif.nodes.values_mut() {
                node.neighbors.retain(|&n| n != node_id);
            }
            
            println!("Node {} failed and removed from network", node_id);
        }
    }
}

/// Demonstrate multi-rate sensing
fn multi_rate_example() {
    println!("\n\nMulti-Rate Sensor Fusion Example");
    println!("=================================");
    
    // Different sensor types with different update rates
    // - GPS: 1 Hz (slow, accurate position)
    // - IMU: 100 Hz (fast, acceleration)
    // - Camera: 10 Hz (medium, position)
    
    let mut dif = DistributedInformationFilter::<f64>::new(6);  // [x, y, z, vx, vy, vz]
    
    // Initialize 3 sensor nodes
    let info_init = 0.001;
    let Y_init = vec![info_init; 36];
    let y_init = vec![0.0; 6];
    
    // Node 0: GPS
    dif.add_node(0, Y_init.clone(), y_init.clone()).unwrap();
    // Node 1: IMU
    dif.add_node(1, Y_init.clone(), y_init.clone()).unwrap();
    // Node 2: Camera
    dif.add_node(2, Y_init.clone(), y_init.clone()).unwrap();
    
    // Fully connected network
    dif.connect_nodes(0, 1, 0).unwrap();
    dif.connect_nodes(0, 2, 0).unwrap();
    dif.connect_nodes(1, 2, 0).unwrap();
    
    println!("3-node network with different sensor types:");
    println!("- Node 0: GPS (1 Hz)");
    println!("- Node 1: IMU (100 Hz)");
    println!("- Node 2: Camera (10 Hz)");
    
    // Simulate 1 second of data
    let mut time = 0.0;
    let dt = 0.01;  // 100 Hz base rate
    
    while time < 1.0 {
        let time_ms = (time * 1000.0) as u32;
        
        // GPS updates every 1000ms
        if time_ms % 1000 == 0 {
            let H = vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // x
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,  // y
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  // z
            ];
            let R = vec![
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 2.0,  // Worse vertical accuracy
            ];
            let measurement = vec![10.0, 20.0, 5.0];
            dif.local_update(0, &measurement, &H, &R).unwrap();
        }
        
        // IMU updates every 10ms (100 Hz)
        if time_ms % 10 == 0 {
            // IMU measures acceleration (affects velocity)
            let H = vec![
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  // vx
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,  // vy
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  // vz
            ];
            let R = vec![
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ];
            let measurement = vec![0.5, 0.2, -0.1];
            dif.local_update(1, &measurement, &H, &R).unwrap();
        }
        
        // Camera updates every 100ms (10 Hz)
        if time_ms % 100 == 0 {
            let H = vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // x
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,  // y
            ];
            let R = vec![
                0.5, 0.0,
                0.0, 0.5,
            ];
            let measurement = vec![10.5, 20.2];
            dif.local_update(2, &measurement, &H, &R).unwrap();
        }
        
        // Process messages
        dif.process_messages().unwrap();
        
        time += dt;
    }
    
    println!("\nProcessed 1 second of multi-rate sensor data");
    println!("- GPS: 1 update");
    println!("- IMU: 100 updates");
    println!("- Camera: 10 updates");
    
    // Get final consensus
    if let Ok(consensus) = dif.get_consensus_state() {
        println!("\nFinal consensus state:");
        println!("Position: [{:.2}, {:.2}, {:.2}]", consensus[0], consensus[1], consensus[2]);
        println!("Velocity: [{:.2}, {:.2}, {:.2}]", consensus[3], consensus[4], consensus[5]);
    }
}

