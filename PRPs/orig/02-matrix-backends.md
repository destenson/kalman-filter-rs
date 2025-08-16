# PRP: Multi-Backend Matrix Support for Kalman Filter

## Goal
Implement a flexible backend abstraction layer for the Kalman filter that supports multiple matrix libraries (arrays, vectors, gai-rs-math-matrices Matrix, nalgebra, ndarray, and potentially others) with compile-time backend selection through feature flags.

## Why
- **Flexibility**: Different use cases require different backends (embedded systems need arrays, scientific computing prefers nalgebra/ndarray)
- **Performance**: Allow users to choose the optimal backend for their specific hardware and use case
- **Integration**: Enable seamless integration with existing codebases using different matrix libraries
- **Future-proofing**: Easy addition of new backends like faer-rs, candle, or sprs without rewriting filter logic

## What
Create a trait-based abstraction that allows the Kalman filter to work with multiple matrix backends, selected at compile time through feature flags.

### Success Criteria
- [ ] Kalman filter works with at least 4 different backends (arrays, Vec, nalgebra, gai-rs-math-matrices)
- [ ] Backend selection via feature flags with sensible default
- [ ] No performance regression vs current implementation
- [ ] Examples demonstrating each backend
- [ ] All existing tests pass with each backend

## All Needed Context

### Documentation & References
```yaml
- url: https://docs.rs/nalgebra/latest/nalgebra/base/struct.Matrix.html
  why: nalgebra Matrix API for backend implementation
  
- url: https://docs.rs/ndarray/latest/ndarray/
  why: ndarray ArrayBase API and dimension handling
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\numerical-backends\src\backends\traits.rs
  why: Existing backend abstraction pattern in codebase to follow
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-math-matrices\src\traits.rs
  why: Matrix trait definitions from gai-rs-math
  
- file: C:\Users\deste\repos\gai\gai-rs\crates\gai-rs-kalman\src\filter.rs
  why: Current implementation to refactor - uses Vec<T> for storage

- url: https://github.com/strawlab/adskalman-rs/blob/main/src/lib.rs
  why: Example of nalgebra-based Kalman implementation
  
- url: https://github.com/spdes/kalman-rust
  why: Example of ndarray-based Kalman implementation
```

### Suggested Codebase Structure
```bash
gai-rs-kalman/
├── Cargo.toml          # Updated with backend features
├── src/
│   ├── lib.rs          # Module exports with feature gates
│   ├── backend/
│   │   ├── mod.rs      # Backend trait definition
│   │   ├── array.rs    # Fixed-size array backend
│   │   ├── vec.rs      # Dynamic Vec backend
│   │   ├── nalgebra.rs # nalgebra backend
│   │   ├── ndarray.rs  # ndarray backend  
│   │   └── gai_math.rs # gai-rs-math-matrices backend
│   ├── filter.rs       # Generic over backend trait
│   ├── types.rs        # Backend-agnostic types
│   ├── builder.rs      # Generic builder
│   └── ...
└── examples/
    ├── simple_1d_vec.rs
    ├── simple_1d_nalgebra.rs
    ├── position_2d_ndarray.rs
    └── ...
```

### Known Gotchas & Library Quirks
```rust
// CRITICAL: nalgebra uses column-major storage by default
// CRITICAL: ndarray can be either row-major or column-major
// CRITICAL: gai-rs-math-matrices uses row-major [[T; COLS]; ROWS]
// CRITICAL: Must handle both static and dynamic dimensions
// CRITICAL: Some backends (arrays) only work with compile-time known dimensions
// CRITICAL: Matrix inversion can fail - need Result types
// CRITICAL: Performance characteristics vary significantly between backends
```

## Implementation Blueprint

### Backend Trait Design
```rust
// Core trait that all backends must implement
pub trait KalmanBackend: Clone + Send + Sync {
    type Scalar: KalmanScalar;
    type Vector: Clone;
    type Matrix: Clone;
    
    // Constructors
    fn zeros_vector(size: usize) -> Self::Vector;
    fn zeros_matrix(rows: usize, cols: usize) -> Self::Matrix;
    fn identity_matrix(size: usize) -> Self::Matrix;
    
    // Element access
    fn get_vector_element(&self, v: &Self::Vector, index: usize) -> Self::Scalar;
    fn set_vector_element(&self, v: &mut Self::Vector, index: usize, value: Self::Scalar);
    fn get_matrix_element(&self, m: &Self::Matrix, row: usize, col: usize) -> Self::Scalar;
    fn set_matrix_element(&self, m: &mut Self::Matrix, row: usize, col: usize, value: Self::Scalar);
    
    // Core operations for Kalman filter
    fn matrix_multiply(&self, a: &Self::Matrix, b: &Self::Matrix) -> Self::Matrix;
    fn matrix_vector_multiply(&self, m: &Self::Matrix, v: &Self::Vector) -> Self::Vector;
    fn matrix_transpose(&self, m: &Self::Matrix) -> Self::Matrix;
    fn matrix_add(&self, a: &Self::Matrix, b: &Self::Matrix) -> Self::Matrix;
    fn matrix_subtract(&self, a: &Self::Matrix, b: &Self::Matrix) -> Self::Matrix;
    fn vector_subtract(&self, a: &Self::Vector, b: &Self::Vector) -> Self::Vector;
    
    // Matrix inversion for update step
    fn matrix_inverse(&self, m: &Self::Matrix) -> Result<Self::Matrix, KalmanError>;
    
    // Helper for symmetry enforcement
    fn make_symmetric(&self, m: &mut Self::Matrix);
}

// Optional trait for backends supporting static dimensions
pub trait StaticBackend<const N: usize, const M: usize>: KalmanBackend {
    fn from_arrays_vector(data: [Self::Scalar; N]) -> Self::Vector;
    fn from_arrays_matrix(data: [[Self::Scalar; N]; N]) -> Self::Matrix;
}
```

### Task List

```yaml
Task 1: Create backend trait module
MODIFY src/lib.rs:
  - Add pub mod backend
  - Add backend re-exports behind feature gates
  
CREATE src/backend/mod.rs:
  - Define KalmanBackend trait
  - Define StaticBackend trait
  - Create BackendType enum for runtime selection

Task 2: Implement Vec backend
CREATE src/backend/vec.rs:
  - MIRROR pattern from: existing src/filter.rs implementation
  - Implement KalmanBackend for VecBackend struct
  - Move matrix operations from filter.rs

Task 3: Implement array backend
CREATE src/backend/array.rs:
  - Use const generics for compile-time dimensions
  - Implement both KalmanBackend and StaticBackend
  - Only available for fixed-size filters

Task 4: Implement nalgebra backend
CREATE src/backend/nalgebra.rs:
  - Use nalgebra::DMatrix for dynamic dimensions
  - Use nalgebra::SMatrix for static dimensions
  - Leverage nalgebra's built-in operations

Task 5: Implement gai-rs-math backend
CREATE src/backend/gai_math.rs:
  - Use gai_rs_math_matrices::Matrix
  - Follow patterns from gai-rs-math-matrices/src/lib.rs

Task 6: Implement ndarray backend
CREATE src/backend/ndarray.rs:
  - Use ndarray::Array2 for matrices
  - Use ndarray::Array1 for vectors
  - Handle dimension checking

Task 7: Refactor KalmanFilter to use backend
MODIFY src/filter.rs:
  - Make KalmanFilter generic over B: KalmanBackend
  - Replace Vec operations with backend calls
  - Update predict() and update() methods

Task 8: Update builder for backend support
MODIFY src/builder.rs:
  - Make builder generic over backend
  - Add backend selection method
  - Validate dimensions for static backends

Task 9: Create backend-specific examples
CREATE examples/backend_comparison.rs:
  - Same filter with different backends
  - Performance comparison
  - Show API differences

Task 10: Update and create tests
MODIFY src/filter.rs tests:
  - Parameterize over backends
  - Test each backend implementation
  - Add backend-specific edge cases
```

### Integration Points
```yaml
CARGO:
  - add to: Cargo.toml
  - features:
    default = ["backend-vec"]
    backend-vec = []
    backend-array = []
    backend-nalgebra = ["nalgebra"]
    backend-ndarray = ["ndarray"]
    backend-gai-math = ["gai-rs-math-matrices"]
    all-backends = ["backend-vec", "backend-array", "backend-nalgebra", "backend-ndarray", "backend-gai-math"]

MODULE EXPORTS:
  - add to: src/lib.rs
  - pattern: 
    #[cfg(feature = "backend-nalgebra")]
    pub use backend::nalgebra::NalgebraBackend;

DOCUMENTATION:
  - update: README.md
  - add backend selection guide
  - performance characteristics table
```

## Validation Loop

### Level 1: Syntax & Style
```bash
# Check each backend compiles
cargo check --features backend-vec
cargo check --features backend-array
cargo check --features backend-nalgebra
cargo check --features backend-ndarray
cargo check --features backend-gai-math

# Lint all code
cargo clippy --all-features -- -D warnings
cargo fmt --check
```

### Level 2: Unit Tests
```rust
// Test framework for each backend
#[cfg(test)]
mod backend_tests {
    use super::*;
    
    fn test_backend_operations<B: KalmanBackend>() {
        // Test matrix multiplication
        let a = B::identity_matrix(3);
        let b = B::zeros_matrix(3, 3);
        let c = B::matrix_multiply(&a, &b);
        // Assert c is zeros
        
        // Test inversion
        let m = B::identity_matrix(3);
        let inv = B::matrix_inverse(&m).unwrap();
        // Assert inv equals identity
    }
    
    #[test]
    #[cfg(feature = "backend-vec")]
    fn test_vec_backend() {
        test_backend_operations::<VecBackend>();
    }
    
    // Repeat for each backend...
}
```

### Level 3: Integration Tests
```bash
# Run all examples with each backend
cargo run --example simple_1d --features backend-vec
cargo run --example simple_1d --features backend-nalgebra
cargo run --example simple_1d --features backend-ndarray

# Run comparative benchmark
cargo run --example backend_comparison --features all-backends

# Test with all features
cargo test --all-features
```

### Level 4: Performance Validation
```bash
# Benchmark each backend
cargo bench --features backend-vec
cargo bench --features backend-nalgebra
cargo bench --features backend-ndarray

# Compare results in benches/report.txt
```

## Final Validation Checklist
- [ ] All backends compile: `cargo check --all-features`
- [ ] No clippy warnings: `cargo clippy --all-features -- -D warnings`
- [ ] All tests pass: `cargo test --all-features`
- [ ] Examples run with each backend
- [ ] Performance within 10% for Vec backend (no regression)
- [ ] Documentation includes backend selection guide
- [ ] Feature flags properly isolate dependencies

## Anti-Patterns to Avoid
- ❌ Don't expose backend-specific types in public API
- ❌ Don't require users to understand backend internals
- ❌ Don't sacrifice safety for performance 
- ❌ Don't create tight coupling between filter and backend
- ❌ Don't ignore dimension mismatches - fail fast
- ❌ Don't assume all backends support all operations

## Error Handling Strategy
- Use Result types for operations that can fail (inversion, dimension mismatch)
- Backend-specific errors wrapped in KalmanError enum
- Clear error messages indicating backend and operation
- Graceful fallback when optional operations unavailable

## References for Implementation
- Existing numerical-backends crate: `C:\Users\deste\repos\gai\gai-rs\crates\numerical-backends\`
- nalgebra Kalman: https://github.com/strawlab/adskalman-rs
- ndarray Kalman: https://github.com/spdes/kalman-rust  
- Trait-based abstraction: https://doc.rust-lang.org/book/ch10-02-traits.html
- Feature flags: https://doc.rust-lang.org/cargo/reference/features.html

## Notes for AI Agent
- Start with the backend trait definition - get the abstraction right first
- Implement Vec backend first by refactoring existing code
- Use conditional compilation (#[cfg]) extensively for backend selection
- Each backend should be in its own module for clarity
- Test each backend in isolation before integration
- Consider using macro_rules! to reduce boilerplate across backends
- The array backend will be most restrictive - design trait to accommodate it
- Performance benchmarks are critical - users will choose based on speed

## Quality Score: 9/10
Comprehensive context with clear abstraction design, specific implementation steps following existing codebase patterns, executable validation gates, and references to both internal and external examples. Deducted one point as some backend implementations (like gai-rs-math-matrices integration details) may require additional discovery during implementation.
