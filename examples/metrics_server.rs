//! Example of exposing Kalman filter metrics via HTTP endpoint
//!
//! This example demonstrates how to:
//! - Initialize the metrics system
//! - Run Kalman filters with metrics collection
//! - Expose metrics via an HTTP endpoint for Prometheus scraping
//! - Configure filter and simulation parameters via command-line arguments
//!
//! ## Usage
//! ```bash
//! # Run with random noise (different output each time)
//! cargo run --example metrics_server --features prometheus-metrics
//!
//! # Run with a specific seed for reproducible results
//! cargo run --example metrics_server --features prometheus-metrics -- --seed 42
//!
//! # Customize parameters
//! cargo run --example metrics_server --features prometheus-metrics -- \
//!     --seed 42 \
//!     --port 8080 \
//!     --dt 0.5 \
//!     --process-noise-pos 0.01 \
//!     --process-noise-vel 0.01 \
//!     --meas-noise 0.2 \
//!     --meas-noise-std 0.3 \
//!     --sleep-ms 200
//! ```
//!
//! Then access metrics at: http://localhost:9090/metrics (or your configured port)

#[cfg(not(feature = "prometheus-metrics"))]
fn main() {
    eprintln!("This example requires the prometheus-metrics feature.");
    eprintln!("Run with: cargo run --example metrics_server --features prometheus-metrics");
}

#[cfg(feature = "prometheus-metrics")]
fn main() {
    use clap::Parser;
    use kalman_filters::KalmanFilterBuilder;
    use rand::SeedableRng;
    use rand_distr::{Distribution, Normal};
    use std::net::TcpListener;
    use std::thread;
    use std::time::Duration;

    /// Kalman filter metrics server with Prometheus endpoint
    #[derive(Parser, Debug)]
    #[command(author, version, about, long_about = None)]
    struct Args {
        /// Random seed for reproducible results (if not specified, uses random seed)
        #[arg(long)]
        seed: Option<u64>,

        /// Port for metrics HTTP server
        #[arg(long, default_value_t = 9090)]
        port: u16,

        /// Time step (dt) for filter transition matrix
        #[arg(long, default_value_t = 1.0)]
        dt: f64,

        /// Process noise for position component
        #[arg(long, default_value_t = 0.001)]
        process_noise_pos: f64,

        /// Process noise for velocity component
        #[arg(long, default_value_t = 0.001)]
        process_noise_vel: f64,

        /// Measurement noise variance
        #[arg(long, default_value_t = 0.1)]
        meas_noise: f64,

        /// Standard deviation of measurement noise for simulation
        #[arg(long, default_value_t = 0.2)]
        meas_noise_std: f64,

        /// Milliseconds to sleep between filter steps
        #[arg(long, default_value_t = 100)]
        sleep_ms: u64,
    }

    let args = Args::parse();

    // Use provided seed or generate random one
    let seed = args.seed.unwrap_or_else(|| {
        let seed_value = rand::random::<u64>();
        println!("Using random seed: {} (use --seed {} to reproduce)\n", seed_value, seed_value);
        seed_value
    });

    if args.seed.is_some() {
        println!("Using seed: {}\n", seed);
    }

    // Initialize metrics system
    kalman_filters::metrics::init();
    println!("Metrics system initialized");
    println!("Configuration:");
    println!("  Port: {}", args.port);
    println!("  dt: {}", args.dt);
    println!("  Process noise (pos): {}", args.process_noise_pos);
    println!("  Process noise (vel): {}", args.process_noise_vel);
    println!("  Measurement noise: {}", args.meas_noise);
    println!("  Measurement noise std: {}", args.meas_noise_std);
    println!("  Sleep interval: {} ms\n", args.sleep_ms);

    // Clone values needed for the thread
    let dt = args.dt;
    let process_noise_pos = args.process_noise_pos;
    let process_noise_vel = args.process_noise_vel;
    let meas_noise = args.meas_noise;
    let meas_noise_std = args.meas_noise_std;
    let sleep_ms = args.sleep_ms;

    // Create a thread to run filters and generate metrics
    thread::spawn(move || {
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let noise_dist = Normal::new(0.0, meas_noise_std).unwrap();
        // Create a simple 2D position-velocity Kalman filter
        let mut kf = KalmanFilterBuilder::new(2, 1)
            .initial_state(vec![0.0, 0.0])
            .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
            .transition_matrix(vec![1.0, dt, 0.0, 1.0])
            .process_noise(vec![process_noise_pos, 0.0, 0.0, process_noise_vel])
            .observation_matrix(vec![1.0, 0.0]) // observe position only
            .measurement_noise(vec![meas_noise])
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
            let noise = noise_dist.sample(&mut rng);
            let measurement = vec![true_position + noise];

            // Update step
            if let Err(e) = kf.update(&measurement) {
                eprintln!("Update failed: {:?}", e);
            }

            time += dt;

            // Sleep to simulate real-time operation
            thread::sleep(Duration::from_millis(sleep_ms));

            if (time / dt) as i32 % 10 == 0 {
                println!("Processed {} steps, state: {:?}", (time / dt) as i32, kf.state());
            }
        }
    });

    // Start HTTP server for metrics endpoint
    let bind_addr = format!("127.0.0.1:{}", args.port);
    let listener = TcpListener::bind(&bind_addr)
        .unwrap_or_else(|_| panic!("Failed to bind to port {}", args.port));

    println!("Metrics server listening on http://localhost:{}/metrics", args.port);
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
