# PRP: Implement no_std Support and Feature-Gated Filter Variants

## Goal
Transform the Kalman filter library to support no_std environments and allow users to selectively include only the filter variants they need through feature gates, reducing binary size and compilation time while enabling embedded system deployment.

## Why
- **Embedded Support**: Enable usage on microcontrollers and resource-constrained systems without std library
- **Binary Size Optimization**: Users can include only needed filters (e.g., just KF without PF/EnKF)
- **Compilation Speed**: Faster builds when compiling only required components
- **Memory Control**: Predictable memory usage patterns for safety-critical applications
- **Broader Adoption**: Opens library to embedded robotics, IoT, aerospace markets

## What
Implement a comprehensive feature flag system that:
1. Makes the library work without std library (no_std compatible)
2. Feature-gates each of the 7 filter variants independently
3. Provides alloc-based alternatives for dynamic memory when std is disabled
4. Replaces std-only types (HashMap, etc.) with no_std alternatives
5. Maintains backward compatibility with existing users (std remains default)

### Success Criteria
- [ ] Library compiles with `#![no_std]` when std feature is disabled
- [ ] Each filter variant can be independently enabled/disabled
- [ ] All tests pass in both std and no_std configurations
- [ ] Examples work with selective feature combinations
- [ ] Binary size reduction of >50% when using single filter variant
- [ ] Documentation clearly explains feature combinations

## All Needed Context

### Current Architecture Analysis
```yaml
- file: C:\Users\deste\repos\kalman_filter_rs\Cargo.toml
  why: Current features already include std/alloc, need to extend
  
- file: C:\Users\deste\repos\kalman_filter_rs\src\lib.rs
  why: Main module exports, currently unconditionally includes all filters
  
- file: C:\Users\deste\repos\kalman_filter_rs\src\**\*.rs
  why: 198 Vec usages, HashMap in information filters, std::fmt widespread
```

### Std Dependencies Found
1. **Collections**: Vec (198 uses), HashMap (information module), VecDeque (distributed)
2. **Math**: std::f64::consts::PI (particle filter)
3. **Formatting**: std::fmt (error handling)
4. **Sync**: std::sync::OnceLock, AtomicU64 (metrics module)
5. **Time**: std::time::Duration, Instant (metrics module)

### No-std Rust Patterns Reference
```yaml
- url: https://docs.rust-embedded.org/book/intro/no-std.html
  why: Embedded Rust book's no_std guide - canonical reference
  
- url: https://github.com/rust-lang/hashbrown
  why: HashMap replacement that works with alloc, used by std internally
  
- url: https://docs.rs/libm/latest/libm/
  why: Math functions for no_std (sin, cos, sqrt, etc.)
  
- url: https://github.com/dimforge/nalgebra
  why: Example of well-designed no_std math library with feature gates
  
- url: https://docs.rs/heapless/latest/heapless/
  why: Fixed-size collections for true no-alloc embedded systems
```

### Feature Organization Strategy
Based on SOTA.md analysis, the 7 filter variants to feature-gate:
1. `kf` - Standard Kalman Filter (src/filter.rs)
2. `ekf` - Extended Kalman Filter (src/extended.rs)  
3. `ukf` - Unscented Kalman Filter (src/unscented.rs)
4. `pf` - Particle Filter (src/particle/)
5. `enkf` - Ensemble Kalman Filter (src/ensemble/)
6. `ckf` - Cubature Kalman Filter (src/scented.rs)
7. `if` - Information Filter (src/information/)

### Type Replacement Mappings
| std Type | no_std + alloc | no_std + heapless |
|----------|----------------|-------------------|
| std::vec::Vec | alloc::vec::Vec | heapless::Vec |
| std::collections::HashMap | hashbrown::HashMap | heapless::FnvIndexMap |
| std::collections::VecDeque | alloc::collections::VecDeque | heapless::Deque |
| std::fmt | core::fmt | core::fmt |
| std::f64::consts | libm constants | libm constants |

### Known Gotchas
- Random number generation (rand crate) needs careful feature configuration
- Floating point operations may need libm for transcendental functions
- Error type must work in no_std (use core::fmt instead of std::fmt)
- Metrics module heavily depends on std - should be feature-gated entirely
- Legacy feature must interact correctly with new feature gates
- Builder pattern uses Vec extensively - needs alloc at minimum

## Implementation Blueprint

### Task List (in order)

1. **Setup Core no_std Infrastructure**
   - Add `#![cfg_attr(not(feature = "std"), no_std)]` to lib.rs
   - Add `extern crate alloc` when alloc feature enabled
   - Update Cargo.toml with comprehensive feature structure
   - Add libm and hashbrown as optional dependencies
   - Create prelude module for conditional imports

2. **Refactor Type Imports**
   - Create `src/compat.rs` module for type aliases
   - Conditionally import Vec, HashMap, VecDeque based on features
   - Replace all `std::` imports with conditional imports
   - Update error types to use core::fmt
   - Fix floating point constants to use libm when needed

3. **Feature-gate Filter Variants**
   - Wrap each filter module in `#[cfg(feature = "filter_name")]`
   - Update lib.rs exports to be conditional
   - Make builder support conditional based on included filters
   - Ensure types module remains always available (core traits)
   - Update validation module to work with available filters

4. **Update Information Filter Module**
   - Replace HashMap with hashbrown or BTreeMap
   - Make sparse operations optional (requires HashMap)
   - Ensure consensus algorithms work with available collections
   - Feature-gate distributed filter (requires more collections)

5. **Fix Examples and Tests**
   - Update example required-features in Cargo.toml
   - Add no_std example showing embedded usage
   - Create feature combination tests
   - Update benchmarks to handle optional features
   - Add CI matrix for different feature combinations

6. **Documentation Updates**
   - Document feature flags in README.md
   - Add feature compatibility matrix
   - Create migration guide for embedded users
   - Update rustdoc with feature requirements
   - Add size comparison table for feature combinations

### Cargo.toml Structure
```toml
[features]
default = ["std", "kf"]  # Minimal breaking change
std = ["rand/std", "rand_distr/std"]
alloc = ["hashbrown"]

# Filter variants (individually selectable)
kf = []        # Kalman Filter  
ekf = ["kf"]   # Extended KF (depends on KF)
ukf = ["kf"]   # Unscented KF (depends on KF)
pf = []        # Particle Filter
enkf = ["kf"]  # Ensemble KF (depends on KF)
ckf = ["kf"]   # Cubature KF (depends on KF)
if = ["kf"]    # Information Filter (depends on KF)

# Feature groups for convenience
all-filters = ["kf", "ekf", "ukf", "pf", "enkf", "ckf", "if"]
embedded = ["kf", "libm"]  # Minimal embedded preset
```

### Import Pattern for src/compat.rs
```rust
// This module handles std vs no_std compatibility

#[cfg(feature = "std")]
pub use std::vec::Vec;
#[cfg(all(not(feature = "std"), feature = "alloc"))]
pub use alloc::vec::Vec;

#[cfg(feature = "std")]
pub use std::collections::HashMap;
#[cfg(all(not(feature = "std"), feature = "alloc"))]
pub use hashbrown::HashMap;
```

## Validation Gates

```bash
# Test std configuration (default)
cargo test --all-targets
cargo test --doc

# Test no_std with alloc
cargo test --no-default-features --features alloc,kf
cargo build --no-default-features --features alloc,kf --target thumbv7em-none-eabihf

# Test individual filters
cargo build --no-default-features --features std,kf
cargo build --no-default-features --features std,ekf  
cargo build --no-default-features --features std,pf

# Test filter combinations
cargo test --no-default-features --features std,kf,ekf,ukf
cargo test --no-default-features --features std,all-filters

# Check binary sizes
cargo bloat --release --no-default-features --features std,kf
cargo bloat --release --features std,all-filters

# Verify examples still work
cargo run --example simple_1d --no-default-features --features std,kf
cargo run --example particle_filter_robot --no-default-features --features std,pf

# Documentation builds
cargo doc --no-default-features --features alloc,kf
cargo doc --all-features

# Lint and format
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo clippy --no-default-features --features alloc,kf -- -D warnings
```

## Error Handling Strategy
- Use Result types that work in no_std (already using KalmanResult)
- Ensure error formatting works without std (use core::fmt::Display)
- Provide fallback for missing features (compile-time errors with helpful messages)
- Document feature requirements in error messages

## Migration Strategy
1. Phase 1: Add no_std support without breaking changes (std remains default)
2. Phase 2: Feature-gate filters while maintaining `all-filters` for compatibility
3. Phase 3: Update documentation and examples
4. Phase 4: Next major version could change defaults if needed

## References for Implementation
- heapless crate for embedded: https://docs.rs/heapless/
- hashbrown for HashMap: https://docs.rs/hashbrown/
- libm for math functions: https://docs.rs/libm/
- Example no_std library: https://github.com/dimforge/nalgebra/blob/dev/Cargo.toml
- Embedded book: https://docs.rust-embedded.org/book/

## Notes for AI Agent
- Start with minimal changes - add infrastructure first
- Test frequently - no_std errors can be cryptic  
- Use `cargo expand` to verify feature gates work correctly
- Keep std as default to avoid breaking existing users
- Test on actual embedded target if possible (e.g., thumbv7em-none-eabihf)
- Information filter module will need most work due to HashMap usage
- Metrics module should be entirely gated behind std feature

## Quality Score: 8/10
Comprehensive plan with clear implementation path and validation gates. Deducted points for complexity of updating 7 filter variants and extensive collection type replacements needed. All necessary context provided including type mappings, feature dependencies, and migration strategy. Risk areas identified (Information filter HashMap usage, metrics module).