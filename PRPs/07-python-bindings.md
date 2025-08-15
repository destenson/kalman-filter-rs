# PRP-07: Python Bindings for Kalman Filter Crate

## Status: NOT FEASIBLE AS AUTOMATIC SOLUTION

## Summary
Initial goal was to provide Python bindings for the kalman_filter Rust crate with automatic binding generation during the build process. Research has shown this is not feasible without significant manual wrapper code.

## Research Findings

### Approaches Investigated

1. **PyO3 with Manual Wrappers**
   - ❌ Requires manual wrapper for every struct, method, and trait
   - ❌ Cannot handle generics directly (need concrete types)
   - ❌ Extensive boilerplate code needed

2. **UniFFI**
   - ❌ Still requires wrapper implementations
   - ❌ Cannot handle generic types like `KalmanFilter<T>`
   - ❌ Needs Arc wrappers for objects
   - ❌ Requires either UDL files or proc-macro annotations

3. **PyO3-FFI with ctypes**
   - ❌ Requires C-compatible wrapper functions in Rust
   - ❌ Needs Python ctypes wrapper code
   - ❌ Manual memory management concerns

4. **Direct FFI Export**
   - ❌ Still requires `#[no_mangle]` extern "C" functions
   - ❌ Cannot expose Rust types directly
   - ❌ Needs wrapper layer for complex types

### Root Causes Why Automatic Bindings Aren't Feasible

1. **Type System Mismatch**
   - Rust generics (`KalmanFilter<T>`) have no Python equivalent
   - Traits like `NonlinearSystem` cannot map to Python directly
   - Rust's ownership model doesn't translate

2. **No True Automatic Solutions Exist**
   - Every tool requires some form of manual wrapper
   - Even "automatic" tools like UniFFI need annotations or interface definitions
   - The closest to automatic (cbindgen) only generates C headers, not Python code

3. **Library Design Not FFI-Friendly**
   - Heavy use of generics
   - Trait-based abstractions
   - Row-major matrix storage as flat Vec<T>
   - Would need redesign for FFI compatibility

## Lessons Learned

### What We Should Have Done First
1. **Research Phase Before PRP**
   - Create an RFC (Request for Comments) document
   - Spike minimal proofs of concept
   - Test each approach with simplest possible example
   - Get alignment before writing implementation PRP

2. **Define "Automatic" Clearly**
   - True automatic = zero wrapper code
   - Semi-automatic = minimal boilerplate
   - Manual = full wrapper implementation

3. **Evaluate Library FFI-Readiness**
   - Check for generic usage
   - Identify trait dependencies
   - Assess type complexity
   - Determine wrapper scope

## Recommendations

### Short Term
**Do Not Implement Python Bindings Now**
- Too much manual work required
- Would need to wrap all 7 filter types
- Maintenance burden too high
- Not aligned with "automatic" requirement

### Medium Term Options

1. **Create Simplified FFI-Friendly Layer**
   - Non-generic `KalmanFilterF64` type only
   - Expose only core functionality
   - ~500 lines of wrapper code
   - Estimated effort: 1 week

2. **Use Existing Python Libraries**
   - FilterPy - pure Python, well documented
   - pykalman - mature, widely used
   - Document Rust library as "inspiration" for Python implementations

3. **Generate Wrapper Code**
   - Build script to generate boilerplate
   - Template-based code generation
   - Still requires maintenance
   - Estimated effort: 2 weeks

### Long Term Options

1. **Custom Proc Macro Solution** ⭐ **MOST PROMISING**
   - Write a procedural macro that generates FFI wrappers automatically
   - Add attributes like `#[python_binding]` to structs/functions
   - Macro would generate:
     - C-compatible wrapper functions with `#[no_mangle]`
     - Handle type for opaque pointers
     - Getter/setter functions for fields
     - Python ctypes bindings as a separate generated file
   - Benefits:
     - Truly automatic - just annotate what to expose
     - Maintains single source of truth
     - Can handle generics by generating concrete versions
     - Could work like `wasm-bindgen` but for Python
   - Example usage:
     ```rust
     #[python_binding]
     impl KalmanFilter<f64> {
         #[python_binding]
         pub fn predict(&mut self) { ... }
     }
     ```
   - Estimated effort: 2-3 weeks to build the macro system
   - Could be released as a separate crate for reuse

2. **Redesign for FFI**
   - Remove generics from public API
   - Use C-compatible types
   - Implement cbindgen + automatic Python generation
   - Major breaking change

3. **WASM Compilation**
   - Compile to WebAssembly
   - Use wasm-bindgen
   - Works in browser and Node.js
   - Python can load WASM modules

4. **Wait for Better Tools**
   - Rust/Python FFI is evolving
   - New tools may emerge
   - Not actionable now

## Proc Macro Approach - Implementation Sketch

A custom proc macro could solve the automatic binding problem:

```rust
// In kalman_filter crate with new "python" feature
#[cfg_attr(feature = "python", python_binding::class)]
pub struct KalmanFilter<T> {
    // ...
}

#[cfg_attr(feature = "python", python_binding::methods)]
impl KalmanFilter<f64> {
    #[cfg_attr(feature = "python", python_binding::constructor)]
    pub fn new(state_dim: usize, measurement_dim: usize) -> Self { ... }
    
    #[cfg_attr(feature = "python", python_binding::method)]
    pub fn predict(&mut self) { ... }
}
```

The macro would generate:
1. C FFI functions automatically
2. Python ctypes bindings
3. Handle memory management
4. Type conversions

This is similar to how `wasm-bindgen` works but targeting Python instead of JavaScript.

## Conclusion

The goal of "automatic" Python bindings is not achievable with **existing** tools and the library's design. However, a custom proc macro solution could provide truly automatic bindings.

**Recommendation: Consider this PRP for future implementation with proc macro approach**

Short term: Close as "Won't Implement Now"
Long term: Revisit when ready to invest in proc macro development (2-3 weeks)

The proc macro could potentially become a separate, reusable crate that benefits the wider Rust ecosystem.

## Alternative: Documentation-Based Approach

Instead of bindings, provide:
1. Detailed algorithm documentation
2. Mathematical equations
3. Reference implementation notes
4. Python implementation guide
5. Comparison with FilterPy/pykalman

This would help Python developers implement their own versions while learning from the Rust implementation's design.

## Files to Clean Up
- Remove `py-kalman-filter/` directory
- Remove `src/uniffi_bindings.rs`
- Remove `src/ffi.rs`
- Remove `uniffi` and FFI-related features from Cargo.toml
- Remove `pyproject.toml`

## PRP Quality Score: N/A (Research Complete, Implementation Not Recommended)
