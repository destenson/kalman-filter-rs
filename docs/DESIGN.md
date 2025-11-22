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

### Recommended Approach: **Option 1 (Enum-based) - IMPLEMENTED**

We chose Option 1 with an enum-based wrapper for the following reasons:

1. **No heap allocation**: Returns concrete `UnifiedKalmanFilter<T>` enum instead of `Box<dyn Trait>`
2. **Works with dynamic dimensions**: Can switch measurement matrices at runtime for sensor fusion
3. **Predictable performance**: Match overhead is minimal and predictable
4. **Common dimensions optimized**: Pre-defined variants for common sizes (1x1, 2x1, 3x1, 4x2, 6x3)

Implementation:
- `UnifiedKalmanFilter<T>` enum wraps backend-specific implementations
- `KalmanFilterTrait<T>` provides unified interface
- Builder automatically selects optimal backend based on dimensions
- All user-facing APIs use `&[T]` slices

### Current Status: **Phase 1 COMPLETE**

**Implemented:**
- ✅ `UnifiedKalmanFilter<T>` enum with backend-specific variants
- ✅ `KalmanFilterTrait<T>` providing unified interface
- ✅ `KalmanFilterBuilder` returns `UnifiedKalmanFilter<T>`
- ✅ Automatic backend selection based on dimensions and enabled features
- ✅ Unified examples demonstrating backend-agnostic code

**Available Examples:**
- `simple_1d_unified.rs` - 1D position tracking (2 states, 1 measurement)
- `position_2d_unified.rs` - 2D position tracking (4 states, 2 measurements)
- `sensor_fusion_unified.rs` - Multi-sensor fusion with dynamic observation matrices

**Original Examples (showing direct backend usage):**
- `simple_1d.rs` - Shows Vec vs nalgebra differences
- `position_2d.rs` - Shows Vec vs nalgebra differences
- `sensor_fusion.rs` - Shows Vec vs nalgebra differences

### Migration Path

**Phase 1 (COMPLETE)**: Add new unified API
- ✅ Create `KalmanFilterTrait` and `UnifiedKalmanFilter`
- ✅ Update `KalmanFilterBuilder` to return unified type
- ✅ Unified examples demonstrate backend-agnostic usage

**Phase 2 (Future)**: Encourage unified API adoption
- Mark direct constructors as "advanced usage"
- Documentation emphasizes unified API as recommended approach
- Keep both APIs available for flexibility

**Phase 3 (No planned timeline)**: Consider API simplification
- Based on user feedback, may simplify API surface
- Direct backend access remains available for advanced users

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

### Benefits of the Unified API

**For Users:**
1. **Write Once, Run Anywhere**: Same code works with all backend features
2. **No Feature Flag Pollution**: No need for `#[cfg(feature = "...")]` in user code
3. **Automatic Optimization**: Builder selects best backend based on dimensions
4. **Flexible Runtime Behavior**: Can switch observation matrices for sensor fusion
5. **Simpler API**: Use Vec and slices instead of nalgebra types

**For Library Maintainers:**
1. **Single Example Codebase**: Unified examples work with all backends
2. **Easier Testing**: Test unified API instead of multiple backend paths
3. **Better Documentation**: Focus on what users need, not implementation details

### Design Decisions (Resolved)

**1. Return type for `state()` and `covariance()`**
- **Decision**: Return `Vec<T>` (allocates)
- **Rationale**: Simpler API, lifetime management would be complex across backends
- **Performance**: Allocation overhead is minimal compared to filter computation

**2. Runtime vs compile-time dimension handling**
- **Decision**: Support both
- **Rationale**: Builder accepts runtime dimensions, selects compile-time optimized backend when available
- **Implementation**: Enum with variants for common dimensions, falls back to Vec backend

**3. Dynamic measurement dimensions for sensor fusion**
- **Decision**: Support via `set_observation_matrix()` and `set_measurement_noise()`
- **Rationale**: Essential for real-world multi-sensor applications
- **Limitation**: Falls back to Vec backend when measurement dimension needs to change

**4. Advanced features like EKF Jacobians**
- **Decision**: Keep `KalmanFilterTrait` minimal, provide direct backend access for advanced use
- **Rationale**: 80/20 rule - unified API serves most users, advanced users can access backends directly
- **Future**: May add extension traits as needed

## Usage Recommendations

### When to use the Unified API

**Recommended for:**
- ✅ Learning Kalman filters
- ✅ Prototyping and experimentation
- ✅ Most production applications
- ✅ Multi-sensor fusion with varying observation dimensions
- ✅ Code that needs to work with different backends

**Use direct backends for:**
- ⚡ Ultra-performance-critical inner loops (avoid match overhead)
- 🔧 Advanced features not exposed by unified API
- 🎯 Specific optimizations requiring compile-time dimensions
- 📚 Understanding implementation details

### Performance Notes

**Unified API overhead:**
- Enum match: ~1-3 CPU cycles per operation (negligible vs filter math)
- Vec allocation on `state()`/`covariance()`: ~10-100 ns (usually insignificant)
- No heap allocation for filter itself (enum is stack-allocated)

**Backend selection:**
- With `nalgebra` feature: Uses `StaticKalmanFilter` for dimensions (1,1), (2,1), (3,1), (4,2), (6,3)
- Other dimensions: Falls back to `KalmanFilter` (Vec-based)
- Without `nalgebra` feature: Always uses `KalmanFilter`

## Implementation Notes

The unified API is implemented in `src/unified.rs` and used by `src/builder.rs`.
