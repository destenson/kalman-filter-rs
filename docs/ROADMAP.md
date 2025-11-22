# Kalman Filters Crate - Roadmap to World-Class

This document outlines the strategic roadmap for evolving the `kalman-filters` crate from a solid implementation into a world-class library that competes with the best offerings in any language.

## Current State Assessment (v1.1.0)

**Strengths:**

- ✅ Comprehensive filter variants (7 types: KF, EKF, UKF, CKF, EnKF, IF, PF)
- ✅ Well-structured builder pattern for all filters
- ✅ Extensive test coverage (numerical stability, property tests, fault tolerance)
- ✅ Good documentation with examples
- ✅ Prometheus metrics integration
- ✅ Generic scalar support (f32/f64)
- ✅ Optional nalgebra integration for static matrices
- ✅ Minimal technical debt (only 3 TODOs, clean codebase)

**Critical Gaps:**

- ❌ Hand-rolled matrix operations (performance and correctness concern)
- ❌ Public struct fields break encapsulation
- ❌ ~300-400 lines of duplicated matrix code
- ❌ Long methods (>150 lines) hurt readability
- ❌ Silent error handling in CKF predict (CRITICAL - now fixed!)

---

## Priority 1: Infrastructure & Performance (HIGH IMPACT)

### 1. Add Comprehensive Benchmark Suite

**Status**: Partial (basic benchmarks exist)
**Impact**: 🔥 HIGH - Performance regression detection
**Effort**: 1 week

**Current State**: `benches/kalman_benchmarks.rs` exists but incomplete

**Requirements**:

- Benchmark each filter type (KF, EKF, UKF, CKF, EnKF, IF, PF)
- Test across dimensions: 2D, 10D, 50D, 100D
- Measure:
  - Prediction step time
  - Update step time
  - Memory allocation overhead
  - Cache efficiency
- Use `criterion` crate for statistical rigor
- Enable comparison against baseline (pre-BLAS implementation)
- Track in CI to prevent regressions

**Deliverables**:

- `cargo bench` suite with criterion
- Performance dashboard (criterion HTML reports)
- CI integration with historical tracking

---

### 2. Replace Hand-Rolled Matrix Operations with Optimized BLAS

**Status**: Not Started
**Impact**: 🔥 CRITICAL - 5-10x performance improvement
**Effort**: 2-3 weeks
**Files**: `src/filter.rs`, `src/scented.rs`, `src/extended.rs`, `src/unscented.rs`

**Problem**: All matrix operations are currently triple-nested loops:

```rust
for i in 0..n {
    for j in 0..n {
        for k in 0..n {
            fp[i * n + j] = fp[i * n + j] + F[i * n + k] * P[k * n + j];
        }
    }
}
```

**Solution**:

- Use `ndarray` with BLAS backend AND extend `nalgebra` usage to all filters
- Extract to shared `matrix_ops` module
- Benchmark before/after to quantify improvements

**Benefits**:

- 5-10x performance improvement from BLAS
- Better numerical stability
- Reduced code duplication (~300-400 lines eliminated)
- Foundation for GPU acceleration

**Risks**:

- API changes if switching matrix representation
- Public interfaces may maintain backward compatibility

---

### 3. GPU Acceleration Support

**Status**: Not Started
**Impact**: 🔥 HIGH - Enable large-scale applications
**Effort**: 4-6 weeks

**Target Use Cases**:

- Real-time computer vision (100+ particles)
- Ensemble forecasting (1000+ ensemble members)
- Robotics sensor fusion (high-frequency updates)

**Implementation**:

- Add optional `cuda` and/or `wgpu` feature(s)
- Target ParticleFilter and EnsembleKalmanFilter first
- Provide CPU fallback for portability

**Challenges**:

- Complex build dependencies
- Platform compatibility (CUDA = NVIDIA only)
- Memory transfer overhead (CPU ↔ GPU). This should be documented to inform the user.

**Success Metrics**:

- 50-100x speedup for 1000+ particles
- < 1ms latency for 100D particle filter

---

## Priority 2: Code Quality & Maintainability (MEDIUM-HIGH IMPACT)

### 4. Eliminate Code Duplication

**Status**: Not Started
**Impact**: 🔥 HIGH - ~400 lines of duplicated code
**Effort**: 1-2 weeks

**Duplication Patterns**:

1. **Matrix Multiplication** (8+ instances)
   - `filter.rs:185-203`, `extended.rs:256-275`, `information/filter.rs:286-304`
   - Extract to `matrix_ops::multiply(A, B) -> C`

2. **Cholesky Decomposition** (2 instances)
   - `scented.rs:202-227`, `unscented.rs:299-324`
   - Extract to `matrix_ops::cholesky(P) -> L`

3. **Test Helper Functions** (multiple instances)
   - `extended.rs:860-874`, `unscented.rs:1065-1078` - both have `rand_simple()`
   - Move to `tests/helpers.rs` module

**Approach**:

- Create `src/matrix_ops.rs` module
- Provide generic implementations for `Vec<T>` matrices
- Optionally use `nalgebra` for better performance
- Update all filter implementations to use shared code

---

### 5. Encapsulation - Make Struct Fields Private

**Status**: Not Started
**Impact**: 🔥 HIGH - API safety and flexibility
**Effort**: 1 week

**Problem**: All filter structs expose public fields:

```rust
pub struct KalmanFilter<T> {
    pub x: Vec<T>,  // Users can break invariants!
    pub P: Vec<T>,  // Covariance could become non-symmetric
    // ...
}
```

**Solution**:

```rust
pub struct KalmanFilter<T> {
    state_dim: usize,  // Private
    x: Vec<T>,         // Private
    P: Vec<T>,         // Private
    // ...
}

impl<T: KalmanScalar> KalmanFilter<T> {
    pub fn state(&self) -> &[T] { &self.x }
    pub fn covariance(&self) -> &[T] { &self.P }
    pub fn set_process_noise(&mut self, Q: Vec<T>) -> Result<(), KalmanError> {
        validate_covariance(&Q, self.state_dim)?;
        self.Q = Q;
        Ok(())
    }
}
```

**Benefits**:

- Enforce invariants (e.g., P must be symmetric positive-definite)
- Enable internal representation changes without breaking API
- Better error messages for invalid operations

**Migration Path**:

- Add deprecation warnings in v1.2
- Remove public access in v2.0
- Provide migration guide

---

### 6. Refactor Long Methods

**Status**: Not Started
**Impact**: 🟡 MEDIUM - Readability and testability
**Effort**: 1-2 weeks

**Problem**: Update methods exceed 150 lines:

- `filter.rs::update()`: 174 lines
- `extended.rs::update()`: 151 lines
- `unscented.rs::update()`: 180 lines

**Solution**: Extract sub-operations into focused methods:

```rust
impl<T> KalmanFilter<T> {
    fn update(&mut self, z: &[T]) -> Result<()> {
        let y = self.compute_innovation(z)?;
        let S = self.compute_innovation_covariance()?;
        let K = self.compute_kalman_gain(&S)?;
        self.update_state(&K, &y);
        self.update_covariance(&K);
        Ok(())
    }

    fn compute_innovation(&self, z: &[T]) -> Result<Vec<T>> { /* ... */ }
    fn compute_innovation_covariance(&self) -> Result<Vec<T>> { /* ... */ }
    fn compute_kalman_gain(&self, S: &[T]) -> Result<Vec<T>> { /* ... */ }
    fn update_state(&mut self, K: &[T], y: &[T]) { /* ... */ }
    fn update_covariance(&mut self, K: &[T]) { /* ... */ }
}
```

**Benefits**:

- Each method < 30 lines, single responsibility
- Easier to unit test individual steps
- Better error handling at each stage
- Clearer documentation

---

### 7. Replace Magic Numbers with Named Constants

**Status**: Not Started
**Impact**: 🟡 MEDIUM - Code clarity
**Effort**: 1 day

**Examples**:

```rust
// Current
let avg = (new_p[i * n + j] + new_p[j * n + i]) * T::from_f64(0.5);

// Improved
const SYMMETRY_WEIGHT: f64 = 0.5;
let avg = (new_p[i * n + j] + new_p[j * n + i]) * T::from_f64(SYMMETRY_WEIGHT);
```

**Locations**:

- `filter.rs:395`: 0.5 (symmetrization)
- `unscented.rs:72-79`: UKF defaults (alpha=1e-3, beta=2.0, kappa=0.0)
- `ensemble/filter.rs:132`: 1.05 (5% inflation)
- `validation.rs:6-9`: Numerical thresholds

**Create**:

- `src/constants.rs` module
- Document rationale for each constant value

---

### 8. Fix Inconsistent Error Handling

**Status**: ✅ COMPLETED (CKF fixed)
**Impact**: 🔥 HIGH - Correctness
**Effort**: Ongoing

**Fixed Issues**:

- ✅ `scented.rs:243-245` - Silent error now properly propagated

**Remaining Work**:

- Audit all filters for `.inspect_err()` without propagation
- Ensure consistent use of `?` operator
- Add error context with `context()` or custom error types

---

### 9. Clean Up Compiler Warnings and Allow Directives

**Status**: Not Started
**Impact**: 🟡 MEDIUM - Code hygiene
**Effort**: 1 day

**Problem**: Blanket suppressions hide real issues:

```rust
#![allow(unused, non_snake_case)] // DO NOT CHANGE
```

**Approach**:

- Remove `#![allow(...)]` directives
- Fix underlying warnings:
  - Unused variables → rename to `_var` or remove
  - `non_snake_case` → follow Rust conventions or justify exceptions
- Only allow specific instances with inline `#[allow(...)]` + comment

**Benefits**:

- Compiler warnings catch real bugs
- Better code quality signal
- Easier for contributors to follow conventions

---

### 10. Split NonlinearSystem Trait

**Status**: Not Started
**Impact**: 🟡 MEDIUM - API design
**Effort**: 1 week

**Problem**: CKF and UKF don't need Jacobians but trait requires them:

```rust
fn state_jacobian(...) -> Vec<f64> {
    unreachable!("UKF doesn't need Jacobians")
}
```

**Solution**: Trait hierarchy

```rust
pub trait NonlinearDynamics<T> {
    fn state_transition(&self, x: &[T], u: Option<&[T]>, dt: T) -> Vec<T>;
    fn measurement(&self, x: &[T]) -> Vec<T>;
    fn state_dim(&self) -> usize;
    fn measurement_dim(&self) -> usize;
}

pub trait LinearizableSystem<T>: NonlinearDynamics<T> {
    fn state_jacobian(&self, x: &[T], u: Option<&[T]>, dt: T) -> Vec<T>;
    fn measurement_jacobian(&self, x: &[T]) -> Vec<T>;
}
```

**Usage**:

- `UKF<T, S: NonlinearDynamics<T>>`
- `EKF<T, S: LinearizableSystem<T>>`

**Migration**: Provide blanket impl for backward compatibility

---

## Priority 3: Advanced Features (DIFFERENTIATORS)

### 11. Smoothing Algorithms

**Status**: Not Started
**Impact**: 🔥 HIGH - Competitive feature
**Effort**: 2-3 weeks

**Algorithms to Implement**:

1. **Rauch-Tung-Striebel (RTS) Smoother**
   - Backward pass after forward KF
   - Improves state estimates using future measurements
   - Essential for offline applications

2. **Fixed-Lag Smoother**
   - Real-time smoothing with N-step delay
   - Useful for video processing, trajectory estimation

3. **Extended/Unscented Smoother**
   - Nonlinear versions of RTS
   - For EKF/UKF workflows

**API Design**:

```rust
impl KalmanFilter<T> {
    fn smooth(&self, history: &FilterHistory<T>) -> Result<Vec<SmoothedState<T>>>;
}

pub struct FilterHistory<T> {
    states: Vec<Vec<T>>,
    covariances: Vec<Vec<T>>,
    transition_matrices: Vec<Vec<T>>,
}
```

**Use Cases**:

- Post-processing sensor data
- Trajectory reconstruction
- Improving training data quality

---

### 12. Adaptive Filter Tuning

**Status**: Not Started
**Impact**: 🔥 HIGH - Usability for practitioners
**Effort**: 3-4 weeks

**Problem**: Users struggle to tune Q and R matrices

**Solution**: Implement adaptive algorithms

1. **Innovation-Based Adaptive KF**
   - Auto-tune R based on innovation statistics
   - Detect model mismatches

2. **Multiple Model Adaptive Estimation (MMAE)**
   - Run multiple filters with different Q/R
   - Weight by likelihood

3. **Covariance Matching**
   - Adjust Q to match empirical innovation covariance

**API**:

```rust
pub struct AdaptiveKalmanFilter<T> {
    filter: KalmanFilter<T>,
    adaptation_strategy: AdaptationStrategy,
    innovation_history: RingBuffer<Vec<T>>,
}

pub enum AdaptationStrategy {
    InnovationBased { window_size: usize },
    CovarianceMatching { alpha: f64 },
    MultipleModel { models: Vec<KalmanFilter<T>> },
}
```

---

### 13. Multi-Model Filtering

**Status**: Not Started
**Impact**: 🔥 HIGH - Enable mode-switching systems
**Effort**: 3-4 weeks

**Implement**: Interacting Multiple Model (IMM) Filter

**Use Cases**:

- Aircraft with different flight modes (cruise, turn, climb)
- Target tracking with maneuvering
- Economic forecasting with regime switches

**Algorithm**:

1. Run N filters in parallel (one per mode)
2. Compute mode probabilities
3. Mix estimates weighted by probabilities

**API**:

```rust
pub struct InteractingMultipleModel<T> {
    filters: Vec<KalmanFilter<T>>,
    mode_probabilities: Vec<T>,
    transition_matrix: Vec<Vec<T>>, // Mode transitions
}
```

---

### 14. Square Root Filters

**Status**: Not Started
**Impact**: 🟡 MEDIUM - Numerical stability
**Effort**: 2-3 weeks

**Problem**: Long-running filters lose positive-definiteness of P

**Solution**: Work with `L` where `P = L * L^T`

**Variants**:

- Square Root Kalman Filter (SRKF)
- Square Root Unscented Kalman Filter (SR-UKF)

**Benefits**:

- Guaranteed positive-definiteness
- Better numerical stability for ill-conditioned systems
- Halve storage (only need lower triangle)

**Trade-offs**:

- Slightly more complex implementation
- May be slower without optimized Cholesky

---

### 15. Constrained Filtering

**Status**: Not Started
**Impact**: 🟡 MEDIUM - Real-world applicability
**Effort**: 2-3 weeks

**Types of Constraints**:

1. **Box Constraints**: `x_min ≤ x ≤ x_max`
   - Example: Probability must be [0, 1]

2. **Linear Equality**: `A * x = b`
   - Example: Sum of components = 1 (simplex)

3. **Linear Inequality**: `A * x ≤ b`
   - Example: Budget constraints

**Approaches**:

- **Projection**: Filter update → project onto constraint set
- **Penalty**: Add constraint violation to cost function
- **Barrier**: Use barrier functions in optimization

**API**:

```rust
pub struct ConstrainedKalmanFilter<T> {
    filter: KalmanFilter<T>,
    constraints: Vec<Constraint<T>>,
}

pub enum Constraint<T> {
    Box { min: Vec<T>, max: Vec<T> },
    LinearEquality { A: Vec<T>, b: Vec<T> },
    LinearInequality { A: Vec<T>, b: Vec<T> },
}
```

---

## Priority 4: Documentation & Community (MEDIUM IMPACT)

### 16. Interactive Tutorial Book

**Status**: Not Started
**Impact**: 🔥 HIGH - User adoption
**Effort**: 4-6 weeks

**Create**: mdBook with theory + practice

**Structure**:

1. **Introduction**
   - What are Kalman filters?
   - When to use them
   - Decision tree: Which filter variant?

2. **Theory** (gentle introduction)
   - Bayesian estimation
   - Gaussian distributions
   - Prediction-update cycle

3. **Practical Tutorials**
   - Tutorial 1: 1D position tracking
   - Tutorial 2: 2D position + velocity
   - Tutorial 3: Robot localization
   - Tutorial 4: Sensor fusion (GPS + IMU)
   - Tutorial 5: Nonlinear systems (EKF/UKF)

4. **Advanced Topics**
   - Parameter tuning
   - Numerical stability
   - Performance optimization
   - Common pitfalls

5. **API Reference** (auto-generated from docs.rs)

**Deliverables**:

- `docs/` directory with mdBook source
- Deploy to GitHub Pages
- Link from README.md

---

### 17. WASM Interactive Examples

**Status**: Not Started
**Impact**: 🟡 MEDIUM - Marketing and education
**Effort**: 2-3 weeks

**Goal**: Live filter demos in browser

**Examples**:

1. **Mouse Tracker**
   - User moves mouse, filter tracks with latency
   - Visualize innovation, uncertainty ellipse

2. **Noisy Pendulum**
   - Simulate pendulum with sensor noise
   - Compare KF vs EKF vs UKF

3. **Robot Localization**
   - 2D grid, robot with noisy odometry
   - Particle filter vs EKF comparison

**Tech Stack**:

- `wasm-bindgen` for Rust → WASM
- `web-sys` for canvas drawing
- Simple HTML/CSS/JS frontend

**Deployment**: GitHub Pages alongside tutorial book

---

### 18. Benchmark Comparisons

**Status**: Not Started
**Impact**: 🟡 MEDIUM - Marketing credibility
**Effort**: 1-2 weeks

**Compare Against**:

- Python: `pykalman`, `filterpy`, `simdkalman`
- Rust: Other Kalman filter crates (`adskalman`, etc.)
- C++: Eigen-based implementations

**Metrics**:

- Execution time (prediction + update)
- Memory usage
- Numerical accuracy (vs ground truth)

**Publish**:

- Add to README.md
- Write blog post for Rust community
- Be transparent about methodology

**Goal**: Demonstrate Rust competitive advantage

---

## Priority 5: Production Readiness (MEDIUM IMPACT)

### 19. Enhanced Error Types

**Status**: Partial
**Impact**: 🟡 MEDIUM - Developer experience
**Effort**: 1 week

**Current**: Basic `KalmanError` enum exists

**Improve**:

```rust
#[derive(Debug, thiserror::Error)]
pub enum KalmanError {
    #[error("Dimension mismatch: expected {expected:?}, got {actual:?}")]
    DimensionMismatch { expected: (usize, usize), actual: (usize, usize) },

    #[error("Matrix is singular (not invertible) at {location}")]
    SingularMatrix { location: &'static str },

    #[error("Numerical instability detected: {metric} = {value:.2e} at {location}")]
    NumericalInstability { metric: &'static str, value: f64, location: &'static str },

    #[error("Filter divergence: innovation norm {norm:.2e} exceeds threshold {threshold:.2e}")]
    FilterDivergence { norm: f64, threshold: f64 },

    #[error("Covariance matrix is not positive definite")]
    InvalidCovariance,
}
```

**Add**: Error recovery strategies

- Automatic covariance regularization
- Innovation gating for outlier rejection
- Filter reset on divergence

---

### 20. Observability and Health Metrics

**Status**: Partial (Prometheus metrics exist)
**Impact**: 🟡 MEDIUM - Production monitoring
**Effort**: 1-2 weeks

**Expand Prometheus Metrics**:

```rust
// Current metrics (✅ exist)
- prediction_count
- update_count
- covariance_trace
- innovation_norm

// Add (🔜 future)
- filter_divergence_events (counter)
- numerical_instability_warnings (counter)
- matrix_condition_number (gauge)
- innovation_chi_squared (histogram)
- prediction_latency (histogram)
- update_latency (histogram)
```

**Health Checks**:

```rust
pub struct FilterHealth {
    pub is_healthy: bool,
    pub divergence_risk: f64,  // 0.0 = none, 1.0 = critical
    pub numerical_stability: f64,
    pub warnings: Vec<String>,
}

impl KalmanFilter<T> {
    pub fn health_check(&self) -> FilterHealth { /* ... */ }
}
```

---

### 21. no_std Support for Embedded

**Status**: Partial (`alloc` feature exists)
**Impact**: 🟡 MEDIUM - Embedded robotics market
**Effort**: 1-2 weeks

**Current**: Has `alloc` feature, needs testing

**Requirements**:

- Verify `no_std` + `alloc` works
- Test on embedded targets:
  - ARM Cortex-M (STM32, nRF)
  - RISC-V (ESP32-C3)
  - Embedded Linux (Raspberry Pi)

**Challenges**:

- No `Vec` in pure `no_std` → need fixed-size arrays
- No `f64` on some targets → ensure `f32` works
- Limited memory → benchmark memory footprint

**Solution**:

```rust
#[cfg(not(feature = "alloc"))]
pub struct StaticKalmanFilter<T, const N: usize, const M: usize> {
    x: [T; N],
    P: [[T; N]; N],
    // Fixed-size arrays, no heap allocation
}
```

---

### 22. Fuzzing for Robustness

**Status**: Not Started
**Impact**: 🟡 MEDIUM - Bug discovery
**Effort**: 1 week

**Setup**: cargo-fuzz

**Fuzz Targets**:

1. **Matrix Operations**
   - Random matrices → inversion, multiplication
   - Find numerical edge cases

2. **Filter Updates**
   - Random measurements → update
   - Detect divergence, panics, crashes

3. **Builder API**
   - Random parameter combinations
   - Ensure proper validation

**Continuous Fuzzing**: OSS-Fuzz integration

---

## Summary Timeline

### Immediate (Next 2 Weeks)

- ✅ Fix critical CKF error handling (DONE)
- 🔜 Extract matrix operations to shared module
- 🔜 Add comprehensive benchmarks

### Short Term (1-3 Months)

- Replace hand-rolled matrix ops with BLAS
- Make struct fields private
- Refactor long methods
- Implement smoothing algorithms

### Medium Term (3-6 Months)

- Adaptive filter tuning
- Multi-model filtering (IMM)
- Interactive tutorial book
- GPU acceleration (initial version)

### Long Term (6-12 Months)

- Square root filters
- Constrained filtering
- WASM interactive demos
- Full production observability

---

## Success Metrics

**Performance**:

- ⚡ 5-10x speedup from BLAS optimization
- ⚡ 50-100x speedup for GPU particle filters
- 📊 Match or beat Python `filterpy` performance

**Code Quality**:

- 🧹 Zero `#![allow(unused)]` directives
- 🧹 < 50 lines per method (average)
- 🧹 < 1000 lines per file
- 🧹 Zero code duplication in core algorithms

**Community**:

- 📚 Tutorial book with 10+ complete examples
- 🌐 WASM demos on GitHub Pages
- ⭐ 500+ GitHub stars
- 📦 10,000+ downloads per month on crates.io

**Production Readiness**:

- ✅ Comprehensive error handling with recovery
- ✅ Health monitoring and metrics
- ✅ Proven on embedded targets (no_std)
- ✅ 90%+ test coverage maintained

---

## Contributing

This roadmap is a living document. Priorities may shift based on:

- Community feedback
- User pain points
- Emerging research
- Rust ecosystem evolution

**Want to help?** Pick an item, open an issue to discuss approach, submit a PR!

**Questions?** Open a discussion on GitHub.

---

*Last Updated: 2025-01-21*
*Next Review: 2025-02-21*
