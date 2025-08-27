//! Example of exposing Kalman filter metrics via HTTP endpoint
//!
//! This example demonstrates how to:
//! - Initialize the metrics system
//! - Run Kalman filters with metrics collection
//! - Expose metrics via an HTTP endpoint for Prometheus scraping
//!
//! Run with: cargo run --example metrics_server --features prometheus-metrics
//!
//! Then access metrics at: http://localhost:9090/metrics

#[cfg(not(feature = "prometheus-metrics"))]
fn main() {
    eprintln!("This example requires the prometheus-metrics feature.");
    eprintln!("Run with: cargo run --example metrics_server --features prometheus-metrics");
}

#[cfg(feature = "prometheus-metrics")]
fn main() {
    use kalman_filters::KalmanFilterBuilder;
    use std::thread;
    use std::time::Duration;
    use std::net::TcpListener;

    // Initialize metrics system
    kalman_filters::metrics::init();
    println!("Metrics system initialized");

    // Create a thread to run filters and generate metrics
    thread::spawn(|| {
        // Create a simple 2D position-velocity Kalman filter
        let mut kf = KalmanFilterBuilder::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
            .transition_matrix(vec![1.0, 1.0, 0.0, 1.0]) // dt = 1.0
            .process_noise(vec![0.001, 0.0, 0.0, 0.001])
            .observation_matrix(vec![1.0, 0.0]) // observe position only
            .measurement_noise(vec![0.1])
            .build()
            .expect("Failed to build Kalman filter");

        println!("Running Kalman filter simulation...");
        
        // Simulate continuous operation
        let mut time: f64 = 0.0;
        loop {
            // Predict step
            kf.predict();
            
            // Generate synthetic measurement (true position with noise)
            let true_position = time;
            // Simple noise generation without rand dependency
            let noise = ((time.sin() * 100.0) % 1.0 - 0.5) * 0.2;
            let measurement = vec![true_position + noise];
            
            // Update step
            if let Err(e) = kf.update(&measurement) {
                eprintln!("Update failed: {:?}", e);
            }
            
            time += 1.0;
            
            // Sleep to simulate real-time operation
            thread::sleep(Duration::from_millis(100));
            
            if time as i32 % 10 == 0 {
                println!("Processed {} steps, state: {:?}", time as i32, kf.state());
            }
        }
    });

    // Start HTTP server for metrics endpoint
    let listener = TcpListener::bind("127.0.0.1:9090")
        .expect("Failed to bind to port 9090");
    
    println!("Metrics server listening on http://localhost:9090/metrics");
    println!("Use Ctrl+C to stop the server");
    
    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                handle_request(stream);
            }
            Err(e) => {
                eprintln!("Connection failed: {}", e);
            }
        }
    }
}

#[cfg(feature = "prometheus-metrics")]
fn handle_request(mut stream: std::net::TcpStream) {
    use prometheus_client::encoding::text::encode;
    use std::io::{Read, Write};
    
    // Read the request (we only care about the path)
    let mut buffer = [0; 1024];
    if let Err(e) = stream.read(&mut buffer) {
        eprintln!("Failed to read request: {}", e);
        return;
    }
    
    let request = String::from_utf8_lossy(&buffer);
    
    // Check if this is a request for /metrics
    if request.starts_with("GET /metrics") {
        // Get the metrics registry
        let registry = kalman_filters::metrics::registry();
        
        // Encode metrics in Prometheus text format
        let mut buffer = String::new();
        if let Err(e) = encode(&mut buffer, registry) {
            eprintln!("Failed to encode metrics: {}", e);
            let response = format!(
                "HTTP/1.1 500 Internal Server Error\r\n\
                Content-Type: text/plain\r\n\
                Content-Length: {}\r\n\
                \r\n\
                Failed to encode metrics",
                "Failed to encode metrics".len()
            );
            let _ = stream.write_all(response.as_bytes());
            return;
        }
        
        // Send HTTP response with metrics
        let response = format!(
            "HTTP/1.1 200 OK\r\n\
            Content-Type: text/plain; version=0.0.4\r\n\
            Content-Length: {}\r\n\
            \r\n\
            {}",
            buffer.len(),
            buffer
        );
        
        if let Err(e) = stream.write_all(response.as_bytes()) {
            eprintln!("Failed to send response: {}", e);
        }
    } else if request.starts_with("GET /") {
        // Serve a simple HTML page for the root path
        let html = r#"<!DOCTYPE html>
<html>
<head>
    <title>Kalman Filter Metrics Server</title>
</head>
<body>
    <h1>Kalman Filter Metrics Server</h1>
    <p>Metrics are available at <a href="/metrics">/metrics</a></p>
    <p>This endpoint is designed to be scraped by Prometheus.</p>
    <h2>Example Prometheus Configuration</h2>
    <pre>
scrape_configs:
  - job_name: 'kalman_filters'
    static_configs:
      - targets: ['localhost:9090']
    </pre>
</body>
</html>"#;
        
        let response = format!(
            "HTTP/1.1 200 OK\r\n\
            Content-Type: text/html\r\n\
            Content-Length: {}\r\n\
            \r\n\
            {}",
            html.len(),
            html
        );
        
        let _ = stream.write_all(response.as_bytes());
    } else {
        // 404 for other paths
        let response = "HTTP/1.1 404 Not Found\r\n\
                       Content-Type: text/plain\r\n\
                       Content-Length: 9\r\n\
                       \r\n\
                       Not Found";
        let _ = stream.write_all(response.as_bytes());
    }
}
