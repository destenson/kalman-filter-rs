# Changelog

All notable user-visible changes to `kalman_filters` are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the crate follows
[Semantic Versioning](https://semver.org/).

## [1.1.0] - Unreleased

1.1.0 adds a pluggable matrix-backend layer and a backend-agnostic filter API. Code that
compiled against 1.0.1 with the default features keeps compiling.

### Added

- `matrix` module with the `MatrixOps` trait and three implementations: `NaiveBackend`
  (pure Rust, feature `naive-matrix`), `NalgebraBackend` (feature `nalgebra`) and
  `NdarrayBackend` (feature `blas-backend`, BLAS through `ndarray-linalg`).
- `unified` module with `KalmanFilterTrait`, `UnifiedKalmanFilter` and `IntoUnified`;
  `KalmanFilterTrait` and `UnifiedKalmanFilter` are re-exported at the crate root.
- `KalmanFilterBuilder::build_unified()`, which returns a `UnifiedKalmanFilter<T>`. With the
  `nalgebra` feature it uses the compile-time sized `StaticKalmanFilter` for the
  (state, measurement) dimensions (1, 1), (2, 1), (3, 1), (4, 2) and (6, 3), and the
  Vec-backed `KalmanFilter` otherwise.
- Features `naive-matrix` (default), `blas-backend`, `ndarray-linalg`, `ndarray-linalg-blas`,
  `openblas-system` and `openblas-static`.

<!-- DES-59: state the backend selection priority and the scalar bound each backend requires once backend selection is settled. -->

### Changed

- The default feature set is `std` and `naive-matrix`.
- Dependencies: `rand` 0.8 → 0.9, `rand_distr` 0.4 → 0.5, `opencv` 0.95 → 0.97.
- The crate uses Rust edition 2024, which needs Rust 1.85 or later.

### Unchanged

- `KalmanFilterBuilder::build()` still returns `KalmanResult<KalmanFilter<T>>`, and
  `KalmanFilter`'s `predict`, `update`, `state` and `covariance` are still inherent methods,
  so no trait import is needed.

### Removed

- The optional `adskalman` and `once_cell` dependencies.
- The `legacy` and `adskalman` features, and with them the `legacy` module, the
  `ModernKalmanFilter` alias and the `adskalman` re-export. A build that enabled either
  feature against 1.0.1 fails feature resolution against 1.1.0.

<!-- D7: list the behaviour fixes (DES-56, DES-57, DES-60, DES-62, DES-63) as they land. -->

## [1.0.1]

Last release before this changelog was started; see the
[crate documentation](https://docs.rs/kalman_filters/1.0.1) for its API.
