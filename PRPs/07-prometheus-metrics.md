# PRP: Add Prometheus-Compatible Metrics Capabilities

## Goal
Add optional Prometheus metrics instrumentation to the Kalman filter library, enabling users to monitor filter performance, numerical stability, and computational characteristics in production environments without impacting performance when disabled.

## Why
- **Production Observability**: Monitor filter behavior in real-world deployments
- **Performance Tracking**: Measure computation times for predict/update cycles
- **Numerical Health**: Track condition numbers, innovation statistics, convergence rates
- **Resource Usage**: Monitor particle counts, ensemble sizes, matrix operations
- **Debugging Aid**: Correlate filter divergence with system metrics
- **Zero-Cost When Disabled**: Optional feature with no overhead when not used

## What
Implement Prometheus metrics that:
1. Use `prometheus-client` crate as optional dependency
2. Provide metrics via feature flag `prometheus-metrics`
3. Track key filter operations (predict, update, resampling)
4. Monitor numerical health (covariance trace, innovation norms)
5. Count error conditions and recoveries
6. Measure computation times for expensive operations

### Success Criteria
- [ ] Metrics available when `prometheus-metrics` feature enabled
- [ ] Zero performance impact when feature disabled
- [ ] All 7 filter variants have appropriate metrics
- [ ] Metrics follow Prometheus naming conventions
- [ ] Documentation explains metric meanings and usage
- [ ] Example showing metrics endpoint integration

## All Needed Context

### Prometheus Libraries Research
```yaml
- url: https://docs.rs/prometheus-client
  why: Official OpenMetrics implementation, type-safe, no unsafe code
  
- url: https://github.com/prometheus/client_rust
  why: Reference implementation with examples
  
- url: https://prometheus.io/docs/practices/naming/
  why: Metric naming conventions to follow
  
- file: C:\Users\deste\repos\kalman_filter_rs\src\logging.rs
  why: Existing diagnostic infrastructure to augment with metrics
  
- file: C:\Users\deste\repos\kalman_filter_rs\Cargo.toml
  why: Add optional dependency and feature flag
  
- file: C:\Users\deste\repos\kalman_filter_rs\src\lib.rs
  why: Feature-gated module export pattern
```

### Metrics Design Patterns
Based on Prometheus best practices and Rust ecosystem patterns:

1. **Registry Pattern**: Global or injectable registry for metric collection
2. **Lazy Static**: Use `once_cell` or `std::sync::OnceLock` for metric definitions
3. **Feature Gating**: All metrics code behind `#[cfg(feature = "prometheus-metrics")]`
4. **Label Strategy**: Keep cardinality low, use static labels where possible

### Key Metrics to Implement

**Counter Metrics** (monotonically increasing):
- `kalman_filter_predictions_total{filter_type="kf"}` - Total predict operations
- `kalman_filter_updates_total{filter_type="kf"}` - Total update operations  
- `kalman_filter_errors_total{filter_type="kf", error_type="singular"}` - Error counts
- `kalman_filter_resampling_total{filter_type="pf", strategy="systematic"}` - Particle filter resampling

**Gauge Metrics** (can go up or down):
- `kalman_filter_state_dimension{filter_type="kf"}` - Current state dimension
- `kalman_filter_covariance_trace{filter_type="kf"}` - Trace of covariance matrix
- `kalman_filter_innovation_norm{filter_type="kf"}` - Latest innovation norm
- `kalman_filter_particles_effective{filter_type="pf"}` - Effective sample size
- `kalman_filter_ensemble_size{filter_type="enkf"}` - Current ensemble size

**Histogram Metrics** (distributions):
- `kalman_filter_predict_duration_seconds{filter_type="kf"}` - Predict step duration
- `kalman_filter_update_duration_seconds{filter_type="kf"}` - Update step duration
- `kalman_filter_matrix_inversion_duration_seconds` - Matrix inversion time
- `kalman_filter_jacobian_computation_duration_seconds{filter_type="ekf"}` - Jacobian computation

### Integration Points

1. **Core Operations** (src/filter.rs, extended.rs, etc.):
   - Wrap predict/update with timing measurements
   - Track innovation statistics after update
   - Monitor covariance health metrics

2. **Error Paths** (all modules):
   - Increment error counters before returning Err()
   - Label errors by type (singular, dimension_mismatch, etc.)

3. **Expensive Computations**:
   - Matrix inversions (src/filter.rs::invert_matrix)
   - Jacobian computations (src/extended.rs::compute_numerical_jacobian_*)
   - Particle resampling (src/particle/filter.rs)
   - Consensus iterations (src/information/consensus.rs)

### Known Gotchas
- **Label Cardinality**: Avoid dynamic label values that could explode metric count
- **Thread Safety**: Metrics must be Send + Sync for concurrent access
- **Performance**: Use `log::log_enabled!` pattern to skip expensive metric computation
- **Feature Interactions**: Ensure metrics work with all feature combinations
- **Registry Access**: Provide way for users to access registry for /metrics endpoint

## Implementation Blueprint

### Task List (in order)

1. **Setup metrics infrastructure**
   - Add `prometheus-client = { version = "0.22", optional = true }` to Cargo.toml
   - Add `once_cell = { version = "1.19", optional = true }` for lazy statics
   - Create feature flag `prometheus-metrics = ["prometheus-client", "once_cell"]`
   - Create src/metrics.rs module with registry and metric definitions

2. **Define core metrics**
   - Create metric families for each filter type
   - Use const generics or macros to reduce boilerplate
   - Setup lazy initialization with OnceLock
   - Export registry access function for users

3. **Instrument KalmanFilter (src/filter.rs)**
   - Add timing around predict() and update()
   - Track covariance trace after each operation
   - Count successful vs failed updates
   - Monitor innovation norm in update step

4. **Instrument non-linear filters**
   - ExtendedKalmanFilter: Jacobian computation time
   - UnscentedKalmanFilter: Sigma point generation time
   - CubatureKalmanFilter: Cubature point computation
   - Track linearization quality metrics

5. **Instrument advanced filters**
   - ParticleFilter: Effective sample size, resampling frequency
   - EnsembleKalmanFilter: Ensemble spread, member divergence
   - InformationFilter: Information matrix condition number
   - DistributedFilter: Consensus iterations, message counts

6. **Add computation metrics**
   - Matrix inversion duration and success rate
   - Numerical stability checks (determinant, condition number)
   - Memory allocations for large operations (if possible)

7. **Create example and documentation**
   - Add examples/metrics_server.rs showing HTTP endpoint
   - Document each metric's meaning and interpretation
   - Provide Grafana dashboard JSON template
   - Update README with metrics feature usage

### Module Structure
```
src/
├── metrics.rs           # Core metrics infrastructure
│   ├── registry()       # Access to prometheus Registry
│   ├── init_metrics()   # Initialize all metric families
│   └── filter_metrics() # Per-filter metric helpers
├── metrics/
│   ├── counters.rs      # Counter metric definitions
│   ├── gauges.rs        # Gauge metric definitions
│   └── histograms.rs    # Histogram metric definitions
└── [each filter module gets metrics instrumentation inline]
```

### Instrumentation Pattern
The implementation should follow this pattern for zero-cost abstraction:
- Use `#[cfg(feature = "prometheus-metrics")]` on all metrics code
- Create inline functions that compile to no-ops when disabled
- Use macros for repetitive instrumentation patterns
- Leverage existing logging infrastructure where possible

## Validation Gates

```bash
# Build without metrics (should have zero overhead)
cargo build --release
cargo test --all-targets

# Build with metrics enabled
cargo build --release --features prometheus-metrics
cargo test --all-targets --features prometheus-metrics

# Check that metrics don't affect benchmarks significantly
cargo bench --features prometheus-metrics
# Compare with baseline: cargo bench

# Verify no metrics symbols when feature disabled
cargo build --release
nm -C target/release/libkalman_filter.rlib | grep -i prometheus
# Should return nothing

# Run metrics example
cargo run --example metrics_server --features prometheus-metrics
# In another terminal:
curl http://localhost:9090/metrics | grep kalman_filter
# Should see metrics output

# Check feature combinations work
cargo check --features "prometheus-metrics,nalgebra"
cargo check --features "prometheus-metrics,parallel"
cargo check --features "prometheus-metrics,legacy"

# Lint and format
cargo fmt --check
cargo clippy --all-features -- -D warnings

# Documentation builds
cargo doc --features prometheus-metrics --open
```

## Error Handling Strategy
- Metrics should never panic or affect filter operation
- Use `.unwrap_or_else(|_| {})` for metric operations
- Log metric registration failures at debug level
- Provide fallback behavior if registry initialization fails

## References for Implementation
- prometheus-client examples: https://github.com/prometheus/client_rust/tree/main/examples
- Metric naming: https://prometheus.io/docs/practices/naming/
- Rust metrics patterns: https://github.com/metrics-rs/metrics
- Performance considerations: https://prometheus.io/docs/practices/instrumentation/#things-to-watch-out-for

## Notes for AI Agent
- Start with basic counters in KalmanFilter before complex metrics
- Use existing filter dimensions from constructors for gauge initialization
- Reuse timing infrastructure from logging module if applicable
- Test with small example first before instrumenting all filters
- Keep metrics names consistent across filter variants
- Remember to document each metric in module docs
- Consider providing const for metric names to avoid typos

## Quality Score: 8/10
Comprehensive metrics plan with clear implementation path and validation gates. Deducted points for complexity of instrumenting 7 filter variants and ensuring zero overhead when disabled. All necessary context provided including library choices, patterns, and specific metric definitions. Integration points clearly identified.