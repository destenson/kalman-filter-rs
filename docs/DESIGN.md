# Unified Kalman Filter API Design

## Problem

Currently, users must write different code depending on which backend is enabled:

```rust
// Without nalgebra feature
#[cfg(not(feature = "nalgebra"))]
let mut kf = KalmanFilter::new(...);
kf.update(&[1.0]);

// With nalgebra feature
#[cfg(feature = "nalgebra")]
let mut kf = StaticKalmanFilter::<f64, 1, 2>::new(...);
kf.update(&SVector::from([1.0]));
```

This violates the principle of abstraction - users shouldn't need to know about implementation details.

## Solution

The `KalmanFilterBuilder` will automatically construct the appropriate backend implementation and return a unified interface.

### Design Principles

1. **Builder decides backend**: Based on enabled features, builder constructs optimal implementation
2. **Unified interface**: User code works identically regardless of backend
3. **Slice-based API**: Use `&[T]` for input/output to avoid exposing backend types
4. **Type erasure when needed**: Hide compile-time dimensions from user-facing API
5. **Zero-cost when possible**: No runtime overhead for feature detection

### Implementation Strategy

#### Option 1: Enum-based wrapper (Simple, some overhead)

```rust
pub enum KalmanFilterImpl<T> {
    Vec(KalmanFilter<T>),
    #[cfg(feature = "nalgebra")]
    Static2x1(StaticKalmanFilter<T, 1, 2>),
    #[cfg(feature = "nalgebra")]
    Static3x1(StaticKalmanFilter<T, 1, 3>),
    // ... more variants for common sizes
}
```

Pros: Simple, works for dynamic dimensions
Cons: Match overhead, limited nalgebra coverage

#### Option 2: Trait-based abstraction (Clean, flexible)

```rust
pub trait KalmanFilterTrait<T> {
    fn predict(&mut self);
    fn update(&mut self, measurement: &[T]) -> KalmanResult<()>;
    fn state(&self) -> Vec<T>;
    fn covariance(&self) -> Vec<T>;
}

impl<T> KalmanFilterTrait<T> for KalmanFilter<T> { ... }
#[cfg(feature = "nalgebra")]
impl<T, const M: usize, const N: usize> KalmanFilterTrait<T> for StaticKalmanFilter<T, M, N> { ... }
```

Pros: Clean abstraction, no size limits
Cons: Trait object overhead if using `Box<dyn KalmanFilterTrait>`

#### Option 3: Conditional type alias (Zero-cost, limited flexibility)

```rust
#[cfg(not(feature = "nalgebra"))]
pub type KF<T> = KalmanFilter<T>;

#[cfg(feature = "nalgebra")]
pub type KF<T> = /* wrapper that uses nalgebra internally */;
```

Pros: Zero overhead
Cons: Can't switch at runtime, complex to get right

### Recommended Approach: **Option 2 with builder optimization**

1. Create `KalmanFilterTrait` for unified interface
2. Builder returns `Box<dyn KalmanFilterTrait<T>>` for maximum flexibility
3. For known dimensions at compile time, builder can return concrete type (optional optimization)
4. All user-facing APIs use `&[T]` slices

### Migration Path

1. **Phase 1** (this PR): Add new unified API
   - Create `KalmanFilterTrait`
   - Update `KalmanFilterBuilder` to return trait object
   - Examples use new API

2. **Phase 2** (next release): Deprecate old APIs
   - Mark `KalmanFilter::new()` and `StaticKalmanFilter::new()` as deprecated
   - Add deprecation notices pointing to builder pattern

3. **Phase 3** (major version): Clean up
   - Remove direct constructors
   - Keep structs for advanced users who want specific backend

### Example Usage

After unification, user code looks like this:

```rust
use kalman_filters::KalmanFilterBuilder;

// Works with ANY backend - no feature flags needed!
let mut kf = KalmanFilterBuilder::new(2, 1)  // state_dim=2, measurement_dim=1
    .initial_state(vec![0.0, 0.0])
    .initial_covariance(vec![1.0, 0.0, 0.0, 1.0])
    .transition_matrix(vec![1.0, 0.1, 0.0, 1.0])
    .process_noise(vec![0.001, 0.0, 0.0, 0.001])
    .observation_matrix(vec![1.0, 0.0])
    .measurement_noise(vec![0.1])
    .build()?;

// Unified API - works identically regardless of backend
kf.predict();
kf.update(&[1.0])?;

let state = kf.state();  // Returns Vec<f64>
let cov = kf.covariance();  // Returns Vec<f64>
```

### Performance Considerations

- Trait object has small vtable overhead (~1-2 ns per call)
- Slice conversions may require copying for nalgebra backend
- For performance-critical code, users can still access concrete types directly
- Builder could offer `.build_concrete()` for zero-overhead access

### Open Questions

1. Should `state()` and `covariance()` return `Vec<T>` or `&[T]`?
   - `Vec<T>`: Easier, but allocates
   - `&[T]`: Zero-copy, but lifetime management is complex

2. Should we support runtime dimension detection in builder?
   - Yes: Maximum flexibility
   - No: Simpler, user must know dimensions

3. How to handle advanced features like Jacobians for EKF?
   - Keep trait minimal, add extension traits for advanced features

## Implementation Plan

See TODO list in main implementation.
