//! Prometheus metrics instrumentation for Kalman filters
//!
//! This module provides optional Prometheus metrics for monitoring filter performance,
//! numerical stability, and computational characteristics in production environments.
//!
//! All metrics are feature-gated behind `prometheus-metrics` and have zero overhead
//! when disabled.
//!
//! # Usage
//!
//! Enable the `prometheus-metrics` feature in your `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! kalman_filter = { version = "1.0", features = ["prometheus-metrics"] }
//! ```
//!
//! Then initialize metrics and access the registry:
//!
//! ```rust
//! use kalman_filter::metrics;
//!
//! // Initialize metrics (call once at startup)
//! metrics::init();
//!
//! // Get registry for /metrics endpoint
//! let registry = metrics::registry();
//! ```
//!
//! # Available Metrics
//!
//! ## Counter Metrics
//! - `kalman_filter_predictions_total{filter_type}` - Total predict operations
//! - `kalman_filter_updates_total{filter_type}` - Total update operations  
//! - `kalman_filter_errors_total{filter_type, error_type}` - Error counts
//! - `kalman_filter_resampling_total{filter_type, strategy}` - Particle filter resampling
//!
//! ## Gauge Metrics  
//! - `kalman_filter_state_dimension{filter_type}` - Current state dimension
//! - `kalman_filter_covariance_trace{filter_type}` - Trace of covariance matrix
//! - `kalman_filter_innovation_norm{filter_type}` - Latest innovation norm
//! - `kalman_filter_particles_effective{filter_type}` - Effective sample size
//!
//! ## Histogram Metrics
//! - `kalman_filter_predict_duration_seconds{filter_type}` - Predict step duration
//! - `kalman_filter_update_duration_seconds{filter_type}` - Update step duration
//! - `kalman_filter_matrix_inversion_duration_seconds` - Matrix inversion time

#![cfg(feature = "prometheus-metrics")]

use prometheus_client::encoding::EncodeLabelSet;
use prometheus_client::metrics::counter::Counter;
use prometheus_client::metrics::family::Family;
use prometheus_client::metrics::gauge::Gauge;
use prometheus_client::metrics::histogram::{exponential_buckets, Histogram};
use prometheus_client::registry::Registry;
use std::sync::atomic::AtomicU64;
use std::sync::OnceLock;
use std::time::{Duration, Instant};

/// Filter type labels for metrics
#[derive(Clone, Debug, Hash, PartialEq, Eq, EncodeLabelSet)]
pub struct FilterTypeLabels {
    /// Type of filter (kf, ekf, ukf, if, pf, enkf, ckf)
    pub filter_type: String,
}

/// Error type labels for error metrics
#[derive(Clone, Debug, Hash, PartialEq, Eq, EncodeLabelSet)]
pub struct ErrorLabels {
    /// Type of filter
    pub filter_type: String,
    /// Type of error (singular_matrix, dimension_mismatch, numerical_instability)
    pub error_type: String,
}

/// Resampling strategy labels for particle filter metrics
#[derive(Clone, Debug, Hash, PartialEq, Eq, EncodeLabelSet)]
pub struct ResamplingLabels {
    /// Type of filter
    pub filter_type: String,
    /// Resampling strategy (systematic, multinomial, stratified, residual)
    pub strategy: String,
}

/// Global metrics registry
static REGISTRY: OnceLock<Registry> = OnceLock::new();

/// Counter metrics
static PREDICTIONS_TOTAL: OnceLock<Family<FilterTypeLabels, Counter>> = OnceLock::new();
static UPDATES_TOTAL: OnceLock<Family<FilterTypeLabels, Counter>> = OnceLock::new();
static ERRORS_TOTAL: OnceLock<Family<ErrorLabels, Counter>> = OnceLock::new();
static RESAMPLING_TOTAL: OnceLock<Family<ResamplingLabels, Counter>> = OnceLock::new();

/// Gauge metrics
static STATE_DIMENSION: OnceLock<Family<FilterTypeLabels, Gauge>> = OnceLock::new();
static COVARIANCE_TRACE: OnceLock<Family<FilterTypeLabels, Gauge<f64, AtomicU64>>> =
    OnceLock::new();
static INNOVATION_NORM: OnceLock<Family<FilterTypeLabels, Gauge<f64, AtomicU64>>> = OnceLock::new();
static PARTICLES_EFFECTIVE: OnceLock<Family<FilterTypeLabels, Gauge<f64, AtomicU64>>> =
    OnceLock::new();
static ENSEMBLE_SIZE: OnceLock<Family<FilterTypeLabels, Gauge>> = OnceLock::new();

/// Histogram metrics
static PREDICT_DURATION: OnceLock<Family<FilterTypeLabels, Histogram>> = OnceLock::new();
static UPDATE_DURATION: OnceLock<Family<FilterTypeLabels, Histogram>> = OnceLock::new();
static MATRIX_INVERSION_DURATION: OnceLock<Histogram> = OnceLock::new();
static JACOBIAN_COMPUTATION_DURATION: OnceLock<Family<FilterTypeLabels, Histogram>> =
    OnceLock::new();

/// Initialize the metrics system
///
/// This should be called once at application startup before using any filters.
/// Subsequent calls are safe but have no effect.
pub fn init() {
    // Initialize registry
    let registry = REGISTRY.get_or_init(|| {
        let mut registry = Registry::default();

        // Initialize counter metrics
        let predictions = PREDICTIONS_TOTAL.get_or_init(|| Family::default());
        registry.register(
            "kalman_filter_predictions_total",
            "Total number of predict operations performed",
            predictions.clone(),
        );

        let updates = UPDATES_TOTAL.get_or_init(|| Family::default());
        registry.register(
            "kalman_filter_updates_total",
            "Total number of update operations performed",
            updates.clone(),
        );

        let errors = ERRORS_TOTAL.get_or_init(|| Family::default());
        registry.register(
            "kalman_filter_errors_total",
            "Total number of errors encountered by type",
            errors.clone(),
        );

        let resampling = RESAMPLING_TOTAL.get_or_init(|| Family::default());
        registry.register(
            "kalman_filter_resampling_total",
            "Total number of particle filter resampling operations",
            resampling.clone(),
        );

        // Initialize gauge metrics
        let state_dim = STATE_DIMENSION.get_or_init(|| Family::default());
        registry.register(
            "kalman_filter_state_dimension",
            "Current state dimension of the filter",
            state_dim.clone(),
        );

        let cov_trace = COVARIANCE_TRACE
            .get_or_init(|| Family::new_with_constructor(|| Gauge::<f64, AtomicU64>::default()));
        registry.register(
            "kalman_filter_covariance_trace",
            "Trace of the covariance matrix (sum of diagonal elements)",
            cov_trace.clone(),
        );

        let innovation = INNOVATION_NORM
            .get_or_init(|| Family::new_with_constructor(|| Gauge::<f64, AtomicU64>::default()));
        registry.register(
            "kalman_filter_innovation_norm",
            "Norm of the latest innovation vector",
            innovation.clone(),
        );

        let particles = PARTICLES_EFFECTIVE
            .get_or_init(|| Family::new_with_constructor(|| Gauge::<f64, AtomicU64>::default()));
        registry.register(
            "kalman_filter_particles_effective",
            "Effective sample size for particle filters",
            particles.clone(),
        );

        let ensemble = ENSEMBLE_SIZE.get_or_init(|| Family::default());
        registry.register(
            "kalman_filter_ensemble_size",
            "Current ensemble size for ensemble Kalman filters",
            ensemble.clone(),
        );

        // Initialize histogram metrics with exponential buckets
        let predict_hist = PREDICT_DURATION.get_or_init(|| {
            Family::new_with_constructor(|| Histogram::new(exponential_buckets(1e-6, 2.0, 20)))
        });
        registry.register(
            "kalman_filter_predict_duration_seconds",
            "Duration of predict step operations in seconds",
            predict_hist.clone(),
        );

        let update_hist = UPDATE_DURATION.get_or_init(|| {
            Family::new_with_constructor(|| Histogram::new(exponential_buckets(1e-6, 2.0, 20)))
        });
        registry.register(
            "kalman_filter_update_duration_seconds",
            "Duration of update step operations in seconds",
            update_hist.clone(),
        );

        let matrix_inv = MATRIX_INVERSION_DURATION
            .get_or_init(|| Histogram::new(exponential_buckets(1e-6, 2.0, 20)));
        registry.register(
            "kalman_filter_matrix_inversion_duration_seconds",
            "Duration of matrix inversion operations in seconds",
            matrix_inv.clone(),
        );

        let jacobian = JACOBIAN_COMPUTATION_DURATION.get_or_init(|| {
            Family::new_with_constructor(|| Histogram::new(exponential_buckets(1e-6, 2.0, 20)))
        });
        registry.register(
            "kalman_filter_jacobian_computation_duration_seconds",
            "Duration of Jacobian computation in seconds",
            jacobian.clone(),
        );

        registry
    });

    // Registry is now initialized
    log::debug!("Kalman filter metrics initialized");
}

/// Get the global metrics registry
///
/// Returns the Prometheus registry that can be used to expose metrics
/// via an HTTP endpoint. Call `init()` first to ensure metrics are registered.
pub fn registry() -> &'static Registry {
    REGISTRY.get().unwrap_or_else(|| {
        // Auto-initialize if not done explicitly
        init();
        REGISTRY.get().unwrap()
    })
}

/// Record a prediction operation
pub fn record_prediction(filter_type: &str) {
    if let Some(counter) = PREDICTIONS_TOTAL.get() {
        counter
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .inc();
    }
}

/// Record an update operation
pub fn record_update(filter_type: &str) {
    if let Some(counter) = UPDATES_TOTAL.get() {
        counter
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .inc();
    }
}

/// Record an error occurrence
pub fn record_error(filter_type: &str, error_type: &str) {
    if let Some(counter) = ERRORS_TOTAL.get() {
        counter
            .get_or_create(&ErrorLabels {
                filter_type: filter_type.to_string(),
                error_type: error_type.to_string(),
            })
            .inc();
    }
}

/// Record a particle filter resampling operation
pub fn record_resampling(filter_type: &str, strategy: &str) {
    if let Some(counter) = RESAMPLING_TOTAL.get() {
        counter
            .get_or_create(&ResamplingLabels {
                filter_type: filter_type.to_string(),
                strategy: strategy.to_string(),
            })
            .inc();
    }
}

/// Set the current state dimension
pub fn set_state_dimension(filter_type: &str, dimension: usize) {
    if let Some(gauge) = STATE_DIMENSION.get() {
        gauge
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .set(dimension as i64);
    }
}

/// Set the covariance matrix trace
pub fn set_covariance_trace(filter_type: &str, trace: f64) {
    if let Some(gauge) = COVARIANCE_TRACE.get() {
        gauge
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .set(trace);
    }
}

/// Set the innovation norm
pub fn set_innovation_norm(filter_type: &str, norm: f64) {
    if let Some(gauge) = INNOVATION_NORM.get() {
        gauge
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .set(norm);
    }
}

/// Set the effective particle count
pub fn set_effective_particles(filter_type: &str, count: f64) {
    if let Some(gauge) = PARTICLES_EFFECTIVE.get() {
        gauge
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .set(count);
    }
}

/// Set the ensemble size
pub fn set_ensemble_size(filter_type: &str, size: usize) {
    if let Some(gauge) = ENSEMBLE_SIZE.get() {
        gauge
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .set(size as i64);
    }
}

/// Record predict operation duration
pub fn record_predict_duration(filter_type: &str, duration: Duration) {
    if let Some(histogram) = PREDICT_DURATION.get() {
        histogram
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .observe(duration.as_secs_f64());
    }
}

/// Record update operation duration
pub fn record_update_duration(filter_type: &str, duration: Duration) {
    if let Some(histogram) = UPDATE_DURATION.get() {
        histogram
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .observe(duration.as_secs_f64());
    }
}

/// Record matrix inversion duration
pub fn record_matrix_inversion_duration(duration: Duration) {
    if let Some(histogram) = MATRIX_INVERSION_DURATION.get() {
        histogram.observe(duration.as_secs_f64());
    }
}

/// Record Jacobian computation duration
pub fn record_jacobian_duration(filter_type: &str, duration: Duration) {
    if let Some(histogram) = JACOBIAN_COMPUTATION_DURATION.get() {
        histogram
            .get_or_create(&FilterTypeLabels {
                filter_type: filter_type.to_string(),
            })
            .observe(duration.as_secs_f64());
    }
}

/// Utility for timing operations
pub struct MetricsTimer {
    start: Instant,
}

impl MetricsTimer {
    /// Start a new timer
    pub fn start() -> Self {
        Self {
            start: Instant::now(),
        }
    }

    /// Finish timing and record to a histogram
    pub fn finish_predict(self, filter_type: &str) {
        let duration = self.start.elapsed();
        record_predict_duration(filter_type, duration);
    }

    /// Finish timing and record to update histogram
    pub fn finish_update(self, filter_type: &str) {
        let duration = self.start.elapsed();
        record_update_duration(filter_type, duration);
    }

    /// Finish timing and record to matrix inversion histogram
    pub fn finish_matrix_inversion(self) {
        let duration = self.start.elapsed();
        record_matrix_inversion_duration(duration);
    }

    /// Finish timing and record to Jacobian histogram
    pub fn finish_jacobian(self, filter_type: &str) {
        let duration = self.start.elapsed();
        record_jacobian_duration(filter_type, duration);
    }

    /// Get elapsed duration without recording
    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }
}

/// Helper macro for recording metrics with minimal overhead
#[macro_export]
macro_rules! metrics_record {
    (prediction, $filter_type:expr) => {
        #[cfg(feature = "prometheus-metrics")]
        $crate::metrics::record_prediction($filter_type);
    };
    (update, $filter_type:expr) => {
        #[cfg(feature = "prometheus-metrics")]
        $crate::metrics::record_update($filter_type);
    };
    (error, $filter_type:expr, $error_type:expr) => {
        #[cfg(feature = "prometheus-metrics")]
        $crate::metrics::record_error($filter_type, $error_type);
    };
    (resampling, $filter_type:expr, $strategy:expr) => {
        #[cfg(feature = "prometheus-metrics")]
        $crate::metrics::record_resampling($filter_type, $strategy);
    };
}

/// Helper macro for timing operations
#[macro_export]
macro_rules! metrics_time {
    ($operation:ident, $filter_type:expr, $block:block) => {{
        #[cfg(feature = "prometheus-metrics")]
        let _timer = $crate::metrics::MetricsTimer::start();

        let result = $block;

        #[cfg(feature = "prometheus-metrics")]
        match stringify!($operation) {
            "predict" => _timer.finish_predict($filter_type),
            "update" => _timer.finish_update($filter_type),
            "matrix_inversion" => _timer.finish_matrix_inversion(),
            "jacobian" => _timer.finish_jacobian($filter_type),
            _ => {}
        }

        result
    }};
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_metrics_initialization() {
        init();
        let registry = registry();

        // Registry should be initialized
        assert!(!registry.encode().is_empty());
    }

    #[test]
    fn test_counter_metrics() {
        init();

        record_prediction("kf");
        record_update("kf");
        record_error("kf", "singular_matrix");
        record_resampling("pf", "systematic");

        // Metrics should be recorded (exact values not testable without encoding)
    }

    #[test]
    fn test_gauge_metrics() {
        init();

        set_state_dimension("kf", 4);
        set_covariance_trace("kf", 2.5);
        set_innovation_norm("kf", 0.1);
        set_effective_particles("pf", 850.0);
        set_ensemble_size("enkf", 100);

        // Metrics should be set
    }

    #[test]
    fn test_histogram_metrics() {
        init();

        let duration = Duration::from_millis(1);
        record_predict_duration("kf", duration);
        record_update_duration("kf", duration);
        record_matrix_inversion_duration(duration);
        record_jacobian_duration("ekf", duration);

        // Metrics should be recorded
    }

    #[test]
    fn test_timer() {
        let timer = MetricsTimer::start();
        thread::sleep(Duration::from_millis(1));

        assert!(timer.elapsed() >= Duration::from_millis(1));
    }

    #[test]
    fn test_macros() {
        init();

        metrics_record!(prediction, "kf");
        metrics_record!(error, "kf", "test_error");

        let result = metrics_time!(predict, "kf", {
            thread::sleep(Duration::from_millis(1));
            42
        });

        assert_eq!(result, 42);
    }
}
