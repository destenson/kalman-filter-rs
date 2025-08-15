# PRP: Performance Benchmarks for Kalman Filter Library

## Goal
Implement comprehensive performance benchmarks to measure, compare, and optimize all Kalman filter variants, establishing performance baselines and enabling regression detection.

## Why
- **Current State**: Empty benchmark file (benches/kalman_benchmarks.rs)
- **Performance Validation**: No current way to measure filter performance
- **Optimization Targets**: Can't optimize without measurement baselines
- **Competitive Analysis**: Need comparison with other Kalman filter crates
- **User Guidance**: Help users choose the right filter variant for their performance needs

## What
Create a comprehensive benchmark suite that:
1. Measures core operations (predict, update, inversion)
2. Compares filter variants (KF vs EKF vs UKF vs IF)
3. Tests scaling with dimensions (2D to 1000D states)
4. Benchmarks sparse vs dense operations
5. Compares with other Rust Kalman crates (adskalman)

### Success Criteria
- [ ] All 7 filter variants have benchmarks
- [ ] Dimension scaling from 2 to 1000 states tested
- [ ] Memory usage profiled alongside speed
- [ ] Results comparable with adskalman benchmarks
- [ ] CI integration for regression detection
- [ ] Benchmark results in README

## All Needed Context

### Benchmark References
```yaml
- url: https://bheisler.github.io/criterion.rs/book/
  why: Criterion.rs documentation - the benchmark framework we're using
  
- url: https://github.com/strawlab/adskalman-rs/blob/main/benches/
  why: Reference implementation of Kalman filter benchmarks
  
- url: https://nnethercote.github.io/perf-book/
  why: Rust performance book for optimization strategies
  
- url: https://github.com/nalgebra-org/nalgebra/tree/dev/benches
  why: Matrix operation benchmarks to reference
  
- file: C:\Users\deste\repos\kalman_filter-0.1.2\Cargo.toml
  why: Already has criterion dev-dependency configured
  
- file: C:\Users\deste\repos\kalman_filter-0.1.2\benches\kalman_benchmarks.rs
  why: Empty file to populate with benchmarks
  
- file: C:\Users\deste\repos\kalman_filter-0.1.2\src\filter.rs
  why: Core operations to benchmark
```

### Benchmark Patterns from Ecosystem
Based on high-performance Rust libraries:
- Use criterion's `group` for related benchmarks
- Parameterize over dimensions using `BenchmarkId`
- Include both black_box inputs and outputs
- Measure both throughput and iteration time
- Use realistic data (not zeros/identity matrices)

### Performance Considerations
- Matrix operations dominate runtime (especially inversion)
- Sparse operations should show dramatic speedup at high dimensions
- Memory allocation can be significant for large states
- Cache effects matter for dimension > 100
- Branch prediction affects filter selection

### Known Benchmark Gotchas
- Criterion needs `cargo bench --bench kalman_benchmarks`
- Default optimization can eliminate dead code - use black_box
- Warm-up iterations needed for stable measurements
- Small matrices may fit in cache, skewing results
- Need to disable CPU frequency scaling for consistent results

## Implementation Blueprint

### Task List (in order)

1. **Setup benchmark infrastructure**
   - Create benchmark groups for each filter type
   - Add dimension parameterization (2, 4, 10, 50, 100, 500, 1000)
   - Setup data generation utilities
   - Configure Criterion settings

2. **Benchmark core Kalman filter**
   - Predict step performance
   - Update step performance
   - Full cycle (predict + update)
   - Matrix inversion timing
   - Builder pattern overhead

3. **Benchmark filter variants**
   - Extended KF with Jacobian computation
   - Unscented KF with sigma points
   - Information filter with sparse matrices
   - Ensemble KF with member count scaling
   - Particle filter with particle count scaling
   - Cubature KF with high dimensions

4. **Benchmark sparse operations**
   - Sparse vs dense information filter
   - Sparsity levels (50%, 70%, 90%, 95%)
   - Distributed information filter scaling
   - Consensus algorithm convergence

5. **Memory and allocation benchmarks**
   - Allocation count per operation
   - Peak memory usage by dimension
   - Stack vs heap usage
   - Clone/copy overhead

6. **Comparative benchmarks**
   - Against adskalman (if feature enabled)
   - Against naive matrix operations
   - Float32 vs Float64 performance
   - nalgebra vs Vec backend

7. **Create benchmark report**
   - Generate plots with gnuplot/plotters
   - Add results table to README
   - Create BENCHMARKS.md with detailed analysis
   - Setup GitHub Action for regression detection

### Benchmark Structure
```
benches/kalman_benchmarks.rs
├── Core Operations
│   ├── predict_2d, predict_10d, predict_100d
│   ├── update_2d, update_10d, update_100d
│   └── inversion_2d, inversion_10d, inversion_100d
├── Filter Variants
│   ├── kf_vs_ekf_vs_ukf_10d
│   ├── information_vs_kalman_100d
│   └── particle_scaling_1000_to_10000
├── Sparse Operations
│   ├── sparse_90_percent_100d
│   └── sparse_scaling_50d_to_500d
└── Real World Scenarios
    ├── imu_fusion_6dof
    ├── gps_tracking_4d
    └── radar_tracking_6d
```

## Validation Gates

```bash
# Run all benchmarks
cargo bench --bench kalman_benchmarks

# Run with save baseline
cargo bench --bench kalman_benchmarks -- --save-baseline main

# Compare with baseline
cargo bench --bench kalman_benchmarks -- --baseline main

# Generate HTML report (if configured)
cargo bench --bench kalman_benchmarks -- --output-format bencher

# Profile for bottlenecks (Linux/Mac)
cargo bench --bench kalman_benchmarks --no-run
perf record --call-graph=dwarf target/release/deps/kalman_benchmarks-*
perf report

# Check benchmark compilation
cargo check --benches --all-features

# Verify no performance regression (> 5%)
# Run in CI with previous version as baseline
```

## Performance Targets
Based on adskalman and theoretical limits:
- 2D predict: < 1 microsecond
- 10D predict: < 10 microseconds  
- 100D predict: < 1 millisecond
- Sparse information (90% sparse, 100D): 10x faster than dense
- Particle filter: Linear scaling with particle count

## Error Handling Strategy
- Benchmarks should use `.unwrap()` for simplicity
- Focus on happy path performance
- Add separate benchmarks for error cases if needed

## References for Implementation
- Criterion examples: https://github.com/bheisler/criterion.rs/tree/master/benches
- Benchmark game: https://benchmarksgame-team.pages.debian.net/benchmarksgame/
- nalgebra benchmarks: https://github.com/dimforge/nalgebra/tree/dev/benches
- Profiling guide: https://nnethercote.github.io/perf-book/profiling.html

## Notes for AI Agent
- Start with simple dimensions before scaling up
- Use realistic data patterns (not all zeros)
- Group related benchmarks for easy comparison
- Include both cold and warm cache scenarios
- Add comments explaining what each benchmark measures
- Consider numerical stability edge cases
- Use same random seed for reproducibility

## Quality Score: 9/10
Comprehensive benchmark plan with specific measurements, clear structure, and comparison targets. Includes both micro and macro benchmarks with real-world scenarios. Point deducted for platform-specific profiling tools that may need adjustment.