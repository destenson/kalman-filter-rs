# Design notes — linear Kalman filter core

## 1. Architecture

- `src/filter.rs` (exported as `kalman_filters::linear`) holds the whole filter:
  `LinearKalmanFilter<T, N = Dyn, M = Dyn>`, with `T: RealField + Copy` and `N`, `M` nalgebra dimensions.
  `Dyn` (run-time sizes, the default) and `Const<K>` (compile-time sizes, stack storage) run the same code.
- Fields are nalgebra `OMatrix`/`OVector`: `x, P, F, Q, H, R`, plus a private `Workspace` that holds every
  temporary (candidate `x`/`P`, `I−KH`, `Kᵀ`, `y`, `S`, and the work vectors nalgebra's `quadform` needs).
- `new` validates and parses the row-major slices. `predict` and `update` write into the workspace
  with nalgebra's in-place BLAS-style calls (`gemv`, `gemm`, `gemm_tr`, `quadform_*_with_workspace`) and
  `Cholesky::{new_unchecked, solve_mut}`. Then `commit` swaps the candidate in, but only if it is finite.
- `FilterAllocator<N, M>` is a single named trait for the five `DefaultAllocator: Allocator<…>` bounds.
- `src/matrix/mod.rs` is a ~100-line compatibility shim (row-major `Vec<T>` → nalgebra) that only
  serves the not-yet-rebuilt EKF/UKF/CKF/IF modules. The linear filter does not use it.

## 2. Key decisions

- **nalgebra is a required dependency; there is no own matrix layer.** Rejected: keeping the flat-`Vec`
  backend traits. nalgebra already provides in-place gemm/gemv, quadratic forms with caller-supplied
  workspaces, and an in-place Cholesky. With nalgebra doing all of that, the filter is about 150 lines of
  straight-line math. nalgebra is pure Rust, so it needs no system libraries, and it supports `no_std`.
- **One generic type rather than separate dynamic and static filters.** `new(n, m, …)` turns `n`/`m`
  into `N`/`M` via `Dim::try_to_usize`/`from_usize`. A `Const<K>` that disagrees with `n` gives
  `DimensionMismatch`, never nalgebra's assert. So the only typed front end is a type parameter.
  It adds no constructor or behaviour of its own (B16).
- **Joseph form, via two nalgebra quadratic forms:** `P⁺ = A P Aᵀ + Kᵀᵀ R Kᵀ` with `A = I − KH`
  (`quadform_tr_with_workspace`, then `quadform_with_workspace`), followed by
  `fill_upper_triangle_with_lower_triangle` for exact symmetry. Rejected: the simple form plus
  symmetrization, which loses PSD-ness on models A and B.
- **The filter solves with `S` and never inverts it.** `Kᵀ = S⁻¹(HP)` is computed by `Cholesky::solve_mut`
  on a workspace copy of `HP`. `x⁺ = x + Kᵀᵀy` uses `gemv_tr`, so `K` is never formed.
- **Singularity test relative to each pivot.** nalgebra's `Cholesky::new` only checks `pivot > 0`.
  The filter uses `new_unchecked` instead, then requires `L_kk² > 100ε·S_kk` for every k.
  A NaN pivot also fails this test. Scaling measurement component i by cᵢ scales `L_kk²` and `S_kk`
  by the same cᵢ², so the test is unit-invariant (B10, B21, B22). Rejected: a threshold on `max|S|`,
  which fails B22.
- **Allocation-free `S` buffer.** `Cholesky` takes its matrix by value. The buffer is stored as
  `Option<OMatrix>`, moved into the factorization and recovered with `unpack_dirty()`, on both the
  success and the error path. Steady state has zero allocations (B12). For static dimensions
  everything is on the stack anyway.
- **Atomic steps.** Every check (length, finiteness of `z`, singular `S`) happens before anything is
  committed. Results are built in the workspace and `mem::swap`ped in only if finite. Any error therefore
  leaves `state()`/`covariance()` bit-identical (B9, B20). Garbage left in the scratch buffers after a
  failed step is harmless, because every buffer is fully overwritten before it is read
  (`beta = 0` or `copy_from`). A unit test covers this.
- **PSD validation with nalgebra's Cholesky.** A matrix is accepted if it is symmetric within `100ε·max|a|`,
  and either is exactly zero or has a positive trace with `A + 100ε·tr(A)·I` Cholesky-factorable,
  i.e. `λ_min > −100ε·tr(A)`. Every tolerance is relative, so any scaling of a valid model stays
  valid (B18). Rejected: an eigen-decomposition (`SymmetricEigen`), which needs extra `DimSub<U1>`
  allocator bounds for no gain.
- **Storage is column-major, but `covariance()` is still row-major and borrowed.** `P` is kept exactly
  symmetric, so the two layouts coincide. `state()`/`covariance()` return `&[T]` straight from nalgebra.
  The same holds for symmetric input to `new`: `covariance()` returns `p0` bit-for-bit.
- **Errors.** Added `ZeroDimension` and `NonFiniteInput`. Non-finite results return
  `FilterDivergence`. Dimension products use `checked_mul` (B19).
- **No logging or metrics in the math.** Callers can wrap `predict`/`update` (Q4).

## 3. What fought back

- **Scalar trait mismatch.** Every out-of-scope module is generic over `KalmanScalar: num_traits::Float`.
  Making `KalmanScalar: RealField` would make every `x.sqrt()`/`abs()` in those modules ambiguous,
  because both traits define them. So the filter is bounded on `RealField` directly. Two consequences:
  (a) `information/conversion.rs` needed `T: KalmanScalar + RealField` (plus one `Float::abs`
  disambiguation), and (b) the shim converts values through `f64`, which is exact for f32/f64.
  When the nonlinear filters are rebuilt, they should take `T: RealField` too, and `KalmanScalar`
  can then go away.
- **Bounds sprawl,** as expected. `DefaultAllocator: FilterAllocator<N, M>` must be repeated on the
  struct, the impl and the workspace, and helpers still name `Allocator<D, D>`. Supertrait
  elaboration keeps it to one line per item.
- **`Cholesky` owns its matrix.** This forced the `Option` move-out/move-back dance above.
- **nalgebra's `Cholesky::new` uses an absolute `> 0` pivot test.** That is not usable for the
  singularity decision, hence `new_unchecked` plus the relative check on `l_dirty()`'s diagonal.
- **`quadform` results are not exactly symmetric**, so symmetrization remains after the Joseph form.
  It copies the lower triangle rather than averaging.

## 4. Known weaknesses

- The constant `100ε` is a judgement call. In f32, model B's first pivot is about `1.2e-4·S_kk`. That
  passes with `100ε` (1.2e-5), but would be borderline with the spec reference's `1e3ε`.
- `S` is singular → `SingularMatrix`, and an overflowing `S` (∞) is also reported as `SingularMatrix`,
  not as `FilterDivergence`. Both leave the filter unchanged.
- Validation of slightly asymmetric input (within `100ε`) keeps the lower triangle, so `covariance()`
  equals `p0` exactly only for exactly symmetric `p0`.
- `model()` returns a 4-tuple of nalgebra references. It exists only for the KF→IF conversion.
- The `matrix` shim converts every value through `f64` and allocates. It is not for hot paths, and
  its `invert_matrix` uses LU (`try_inverse`): it reports only exactly singular or non-finite
  results as `SingularMatrix`, where the old code applied an absolute pivot threshold.
- Public signatures expose nalgebra types (`Dim`, `FilterAllocator`, `OMatrix` in `model()`),
  so nalgebra's semver is now part of this crate's API. `nalgebra` is re-exported.

## 5. Next changes

- Easy: time-varying models (`set_transition(&[T])` etc. would reuse `symmetric_psd`); control input
  (one more `gemv` in predict, with `B` validated in `new`); exposing the innovation/`S` for NIS gating
  (both are already in the workspace); a square-root variant (nalgebra has `Cholesky::rank_one_update`).
- Easy: EKF on this core. The EKF update is this update with `H` the Jacobian and `y = z − h(x)`. Extracting
  the update body into a function taking `(h, y)` is a small refactor.
- Hard-ish: anything that wants `T` outside `RealField` (e.g. fixed-point). The bounds would have to change.
  Mixing static `N` with dynamic `M` already works, because the dimensions are independent parameters.

## 6. What was worth keeping from the original

- Kept: the `KalmanError` enum (extended), `KalmanScalar` for the other modules, and the row-major-slice
  construction convention. The old in-file tests' *intent* (closed form, Riccati steady state) is
  covered by the spec tests.
- Dropped: the flat-`Vec` `KalmanFilter`, `StaticKalmanFilter`, `KalmanFilterBuilder`, `UnifiedKalmanFilter`/
  `IntoUnified`/`KalmanFilterTrait`, `legacy.rs`, `validation.rs`, and the
  `MatrixOps`/`MatrixDecomposition`/`MatrixBackend` traits with naive/nalgebra/ndarray backends
  (≈ 4 000 lines). The explicit inverse, the absolute epsilons, and the logging inside the numeric steps
  are gone too.

## Migration

| Old | New |
|---|---|
| `KalmanFilter::initialize(n, m, x0, P0, F, Q, H, R)` (Vecs) | `LinearKalmanFilter::<T>::new(n, m, &x0, &P0, &F, &Q, &H, &R)` (slices) |
| `KalmanFilterBuilder::new(n,m).….build()` | same `LinearKalmanFilter::new` |
| `StaticKalmanFilter<T, N, M>::new(SVector…)` | `LinearKalmanFilter<T, Const<N>, Const<M>>::new(n, m, &slices…)` |
| `UnifiedKalmanFilter`, `IntoUnified`, `KalmanFilterTrait` | `LinearKalmanFilter` (no trait) |
| `KalmanFilter::new(n, m)` + `kf.x = …; kf.P = …` | not available; pass everything to `new` |
| `kf.predict()` (no result) | `kf.predict()?` (errors on non-finite result) |
| `kf.state()` / `kf.x`, `kf.covariance()` / `kf.P` | `kf.state()` / `kf.covariance()` → `&[T]` |
| `set_*` model setters, `legacy::KalmanFilter` control input | removed (not in the contract) |
| `KalmanFilter::<T>::invert_matrix`, `matrix::{MatrixOps, …}` | `nalgebra` directly; `matrix::invert_matrix` shim for the old modules |
| `KalmanScalar::Epsilon(&self)`, `validation::*` | removed |
| feature `nalgebra` | no-op (nalgebra is always on) |

Out-of-scope edits (mechanical): `Cargo.toml`, `src/lib.rs`, the `invert_matrix` call path in `extended.rs`,
`scented.rs`, `unscented.rs`, `information/{filter,extended,sparse,distributed}.rs` and
`builders/information.rs`, plus the port of `information/conversion.rs`.
Examples and benches were ported to `LinearKalmanFilter::new`. Static-size variants use `Const` dims.
The sensor-fusion examples now rebuild the filter per sensor from `state()`/`covariance()`, since
setters are gone. Old integration tests of the removed API were `#[cfg(any())]`-gated:
whole files `core`, `algorithm_validation`, `cross_validation` and `quickcheck_invariants`; individual
items in `fault_tolerance`, `functional`, `numerical_stability`, `validation` and `property`. The KF↔IF
tests in `information_tests`/`property_tests` were ported instead.
`cargo clippy --all-targets` still reports deny-level lints, all in code that was already there
(`erasing_op` in four examples; `src/ensemble`, `src/logging.rs`, `particle_filter_robot.rs`). None is in owned files.

## Summary

- Tests: 32/32 frozen spec tests pass; `cargo test --lib` passes (105, including 6 new filter unit tests).
- LOC: `src/filter.rs` 421 (337 without unit tests, docs included), `src/matrix/mod.rs` 94 (shim); ≈ 4 000 lines removed.
- Build: `cargo build --all-targets` green with default features; clippy is clean in the owned files.
- Most important idea: the filter *is* nalgebra. Generic `Dim` parameters cover static and dynamic
  sizes, a preallocated workspace and in-place BLAS calls make steps allocation-free, and pivots are
  tested relative to their own diagonal entries.
