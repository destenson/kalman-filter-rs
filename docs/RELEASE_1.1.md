# Release 1.1

## Summary

1.1.0 ships the matrix-backend layer and the backend-agnostic filter API, and nothing else new. It succeeds v1.0.1.

The code for both features exists but is not reachable in any released version: v1.0.1 has no `matrix` module and no backend choice, and the backend-agnostic API arrived by changing the return type of `KalmanFilterBuilder::build`, which would break v1.0.1 callers. So 1.1 is the release that makes an already-written feature real and reachable without breaking the 1.x compatibility promise. It adds no filter variant.

Three consequences follow from that choice, and they are the spine of the release:

- `build()` keeps its v1.0.1 signature; the unified API is reached through a new `build_unified()`.
- An enabled backend feature selects that backend, by a documented priority, rather than being silently outranked.
- Every claim the crate makes about itself — features, backends, speedup, numerical method — either becomes true in 1.1 or is removed from the docs.

**The compatibility rule for this release: nothing that compiled against v1.0.1 may stop compiling, and nothing public may be removed or renamed.** Every change that violates that rule is held for 2.0; see D9.

Basis of this document: the published `v1.0.1` documentation on docs.rs, the crate's committed source, and the 22 tracked issues in the `kalman_filters` Linear project (DES-55 … DES-76). No claim below comes from running the code; each one cites the file and line it rests on, and the runs that would settle the rest are the release gate plus [Open questions](#open-questions).

Scope is tracked as project milestones on that project: **`1.1.0`** holds the fifteen issues this release is wrong without, **`1.2`** the six deferred items, and **`2.0`** the breaking track (DES-75). This document is the reasoning; the milestone is the checklist.

---

## Starting point: what v1.0.1 provides

Everything in this section is read from the published crate; it is the baseline 1.1.0 is measured against.

| Property of v1.0.1 | Value |
| --- | --- |
| Public modules | `builder`, `builders`, `ensemble`, `error`, `extended`, `filter`, `information`, `logging`, `particle`, `scented`, `types`, `unscented`, `validation` — **no `matrix`, no `unified`** |
| `KalmanFilterBuilder::build` | `fn build(self) -> KalmanResult<KalmanFilter<T>>` |
| Crate-root re-exports | the seven filter types, the six per-filter builders, `KalmanError`, `KalmanResult`, `KalmanScalar`, `NonlinearSystem`, `JacobianStrategy`, `UKFParameters`, `ResamplingStrategy`, `EnsembleStatistics` — **no `KalmanFilterTrait`, no `UnifiedKalmanFilter`** |
| Dependency set | `rand` 0.8, `rand_distr` 0.4, `opencv` 0.95 (optional), `nalgebra` 0.34 (optional), `rayon`, `serde`, `prometheus-client`, `tracing-subscriber`, plus optional `adskalman` and `once_cell` |

## Target: what v1.1.0 adds and changes

Each row is a property the released crate must have. The detail of each defect and how to reproduce it lives in the issue, not here.

| # | Property of v1.1.0 | Decided by | Issues |
| --- | --- | --- | --- |
| 1 | A public `unified` module: `KalmanFilterTrait`, `UnifiedKalmanFilter`, `IntoUnified`, re-exported at the crate root, so one caller works whatever backend is enabled. | D1 | DES-55 |
| 2 | `KalmanFilterBuilder::build_unified()` returns the unified filter; `build()` returns exactly what it returns in v1.0.1. | D1 | DES-55, DES-57 |
| 3 | A public `matrix` module with naive, nalgebra and ndarray/OpenBLAS implementations, each selectable alone, and the enabled one actually used. | D2 | DES-59, DES-64 |
| 4 | The shipped filters are numerically correct and reject invalid configuration with an error instead of a panic or NaN. | D7 | DES-56, DES-57, DES-60, DES-62, DES-63 |
| 5 | Every feature, command and numerical claim in the README, `CONTRIBUTING.md`, `BLAS_SETUP.md`, `ROADMAP.md`, `SOTA.md` and `VALIDATION.md` is true of this release. | D3, D5, D8 | DES-64, DES-65, DES-67 |
| 6 | The published package holds source, tests, benchmarks, examples, docs, README, LICENSE and a CHANGELOG — and no host configuration or generated artifact. | D4 | DES-74 |
| 7 | CI builds and tests each backend it can build on a stock runner — naive, nalgebra, and ndarray over system OpenBLAS — and a failing benchmark fails its job. The source-built OpenBLAS provider is documented as untested rather than claimed to work. | D2, D7 | DES-58, DES-68, DES-69, DES-72, DES-76 |

---

## Decisions

### D1 — `build()` keeps its v1.0.1 signature; the unified API is opt-in

`KalmanFilterBuilder::build()` stays `-> KalmanResult<KalmanFilter<T>>`, as published in v1.0.1. The backend-agnostic API is reached through a new method:

```rust
let kf  = KalmanFilterBuilder::new(2, 1) /* … */ .build()?;         // KalmanFilter<f64>, unchanged from v1.0.1
let kf  = KalmanFilterBuilder::new(2, 1) /* … */ .build_unified()?; // UnifiedKalmanFilter<f64>, new in 1.1.0
```

`KalmanFilterTrait` and `UnifiedKalmanFilter` are re-exported from the crate root, so `build_unified()` is usable without a second import path.

**Why.** v1.0.1 is the most recent published 1.x release, so its `build` signature is what existing callers compile against. Changing the return type of the most prominent constructor is a breaking change, and this release is not named as one. `docs/DESIGN.md:123-130` already describes Phase 2 as "keep both APIs available", so the split is the design the crate documented for itself. The flip belongs to 2.0 (D9): there `build()` returns the unified type and `build_unified()` becomes the deprecated alias.

**Consequences the implementation must absorb.**

- `src/builder.rs` needs two thin methods over one shared validation path, not two copies of the body. The nalgebra arm of the current `build` (`src/builder.rs:199-359`) contains an unreachable `#[cfg(not(feature = "nalgebra"))]` block at `:343-358` that disappears with this split.
- The three `*_unified` examples switch to `build_unified()`; `simple_1d`, `position_2d`, `sensor_fusion` and `logging_demo` use `build()`. `benches/kalman_benchmarks.rs:727-760` returns `KalmanFilter<T>`, so the benchmark measures the type v1.0.1 users hold.
- `KalmanFilter<T>` keeps its inherent `predict`, `update`, `state` and `covariance` (`src/filter.rs:186,236,374,379`), so `build()` callers need no trait import — the README quick start compiles with `use kalman_filters::KalmanFilterBuilder;` alone, as in v1.0.1.
- With `nalgebra` enabled, `build()` always yields the Vec-backed filter; the static variants are reachable only through `build_unified()`. This must be stated in the builder docs, because `docs/DESIGN.md:224-227` says the builder picks the static type.

**Rejected:** *ship the unified `build()` in 1.1.0 and note the break in the CHANGELOG* — this is the silent kind of break 1.x users do not expect. *Call the release 2.0.0* — semver-correct, but it discards the 1.x line the project intends to keep, and the additive route costs only a method and an example pass.

### D2 — A backend you enable is the backend you get

Selection is fixed by guard conditions and a documented priority, `blas-backend` > `nalgebra` > `naive-matrix`:

```rust
#[cfg(feature = "blas-backend")]
pub use ndarray_backend::NdarrayBackend as DefaultBackend;
#[cfg(all(feature = "nalgebra", not(feature = "blas-backend")))]
pub use nalgebra_backend::NalgebraBackend as DefaultBackend;
#[cfg(all(feature = "naive-matrix", not(feature = "nalgebra"), not(feature = "blas-backend")))]
pub use naive::NaiveBackend as DefaultBackend;
```

and a unit test asserts `std::any::type_name::<DefaultBackend>()` for each configuration, so the priority is pinned by a test rather than by the comment at `src/matrix/mod.rs:41`.

Target feature surface:

| Feature | Meaning | Notes |
| --- | --- | --- |
| `std` | Default; standard library | unchanged from v1.0.1 |
| `alloc` | `no_std` + `alloc` | no test exercises a `no_std` build |
| `naive-matrix` | Hand-rolled backend | default; unchanged behaviour |
| `nalgebra` | nalgebra backend | wins over `naive-matrix`; `features = ["nalgebra"]` is enough |
| `openblas-system` | ndarray backend over system OpenBLAS | implies the ndarray backend and `ndarray-linalg`; the tested BLAS path |
| `openblas-static` | ndarray backend over OpenBLAS built from source | implies the same; needs a Fortran toolchain and is **documented as untested in 1.1** (DES-76) |
| `prometheus-metrics` | Prometheus instrumentation | unchanged |
| `tracing-subscriber`, `serde`, `parallel`, `opencv` | see D5 | |

**Why.** v1.0.1 offers no backend choice, so 1.1.0 introduces the possibility of choosing badly. The module's own comment documents a priority (`src/matrix/mod.rs:41`), and an arm without a `not(...)` guard defeats it: with `naive-matrix` in the default set, an explicit request for `nalgebra` is discarded and the advertised 9–13x speedup is unreachable. Guards fix that and keep `features = ["nalgebra"]` working on top of the default set — which an exclusivity rule would turn into a compile error. Exclusivity is the better end state and it is what 2.0 does; it is not worth breaking a build in a release that promises not to.

**Consequences the implementation must absorb.**

- **Scalar bounds.** `NalgebraBackend` requires `T: KalmanScalar + na::RealField` (`src/matrix/nalgebra_backend.rs:16`) and `NdarrayBackend` requires `T: KalmanScalar + Float + ScalarOperand` (`src/matrix/ndarray_backend.rs:20-23`), while every filter is bounded on `T: KalmanScalar` alone (`src/filter.rs:32-35`). Introduce one cfg-varying public helper trait, e.g.

  ```rust
  #[cfg(feature = "nalgebra")]    pub trait BackendScalar: KalmanScalar + na::RealField {}
  #[cfg(feature = "blas-backend")] pub trait BackendScalar: KalmanScalar + Float + ScalarOperand {}
  #[cfg(feature = "naive-matrix")] pub trait BackendScalar: KalmanScalar {}
  ```

  with blanket impls for `f32`/`f64`, and bound the filter impls on it. This is the smallest change that compiles in every configuration; a single stronger trait would demand every backend's bounds at once and fail on `naive-matrix`. The bound widens only in configurations v1.0.1 could not express, since the blanket impl still covers every `T: KalmanScalar` whenever `naive-matrix` is the selected backend; the one exception is a caller with a hand-written `KalmanScalar` impl who asks for `nalgebra` or a BLAS provider. Say so in the CHANGELOG and `docs/BLAS_SETUP.md`.
- **`blas-backend` alone is not enough.** `src/matrix/ndarray_backend.rs:104,119,134` call into `ndarray_linalg`, which the `blas-backend` feature does not enable (`Cargo.toml:27-30`), so `--features blas-backend` does not compile. The ndarray backend must pull `dep:ndarray-linalg`, and a BLAS provider feature must be what users enable; `blas-backend` alone becomes a `compile_error!` telling them to pick `openblas-system` or `openblas-static`.
- **`dev` needs no change.** It enables `nalgebra` (`Cargo.toml:20`) and the default set enables `naive-matrix`, so under the priority rule a `dev` build selects `nalgebra`, which is what the feature is for. Confirm with the backend-identity test rather than by reading the `cfg`s.
- **`--all-features` does not work, for an unrelated reason.** It enables both OpenBLAS providers, and `Cargo.toml:34` already warns that only one may be enabled at a time to avoid linker conflicts. Under the priority rule the backend itself resolves cleanly (BLAS wins), so the constraint is about providers, not backends. Keep the CI matrix free of `--all-features` and say in `CONTRIBUTING.md` that it is not a supported invocation.
- **CI must build each backend, within what a stock runner can build.** Extend the `features` matrix in `.github/workflows/rust.yml` with `--no-default-features --features std,nalgebra`, `--no-default-features --features std,openblas-system` (with `libopenblas-dev` installed), and `--features nalgebra` on top of the defaults so the priority rule itself is exercised. Negative jobs stay only for configurations v1.0.1 could not build either: no backend at all, and `blas-backend` without a provider.
- **`openblas-static` is documented as untested, not claimed to work.** It builds OpenBLAS from source and needs a Fortran toolchain, which no job installs, so `docs/BLAS_SETUP.md` presents `openblas-system` as the tested path and labels the source-built provider plainly as untested. A scheduled job that builds it is required, and it is a 1.2 item (DES-76) — not a reason to hold this release, but not something the docs may paper over in the meantime.
- **Docs build.** docs.rs passes no features by default and does not use `--all-features` unless asked, so the default build stays safe. Add `[package.metadata.docs.rs] features = ["nalgebra"]` so that `StaticKalmanFilter` and the nalgebra variants of `UnifiedKalmanFilter` appear on docs.rs at all; without it they are invisible because they are `cfg`-gated. With the priority rule this is a valid configuration, not a conflict.
- **Panic parity.** `NdarrayBackend` turns dimension mistakes into `.expect("Invalid matrix dimensions…")` panics (`src/matrix/ndarray_backend.rs:27-180`) while `NaiveBackend` does not. `MatrixOps` is infallible by design, so the fix is to validate dimensions once at the call site or to make the ndarray path return the same `KalmanError`; since 1.1.0 turns the backend into a user choice, a switch must not change whether a program panics.

**Rejected:** *enforce exclusivity with `compile_error!` when two backends are enabled* — it is the stricter and clearer end state, and turning `features = ["nalgebra"]` into a compile error is a break that has to be paid for somewhere; that somewhere is 2.0. *`compile_error!` for a completely missing backend* — kept, because `--no-default-features --features std` produces no build and a clear message costs nothing. *Rename the feature to `ndarray-backend`* — truthful, but `blas-backend` has never compiled in any configuration, so keeping the name costs nothing and a rename adds a breaking edge for no benefit.

### D3 — Performance numbers only from a recorded run

The release must not introduce a performance claim it cannot show. `src/matrix/mod.rs:9-13`, `src/matrix/ndarray_backend.rs:7-8` and `README.md:222` claim a 9–13x speedup, and nothing in the repository records a measurement; `docs/SOTA.md:84` says outright that "actual benchmarks have not been run yet".

The gate for 1.1: after D2 lands, run `benches/kalman_benchmarks.rs` with the naive, nalgebra and OpenBLAS backends on one machine, record the command, the machine and the comparison table in the release notes, and replace the doc claims with the measured range — or delete the claim. Documentation numbers must be traceable to that run.

**Rejected:** *leave the claim and let the benchmark workflow find it later* — a performance number nobody measured is the same class of untruth as a feature that does not exist, and it is the one users quote back.

### D4 — The package contains only what belongs

Tracked as DES-74.

The manifest sets no `[package] exclude`, and `.gitignore` covers only build output, so every tracked file ships. Four of them must not:

- `.cargo/config.toml` — it pins `CPLUS_INCLUDE_PATH` to an absolute compiler include directory that exists on one machine only (`.cargo/config.toml:15`). Cargo reads that file from a package root, so it overrides a downstream user's C++ include path whenever they build a crate that generates C++ bindings.
- `TODO.md` at the repository root — it describes the state of the v0.9 line (`:121`) and lists v1.0 blockers that are closed.
- `tarpaulin-report.html` — a coverage HTML artifact.
- `.github/` — CI definitions, useless in the tarball.

Changes: delete `.cargo/config.toml` from the repository (and gitignore `.cargo/`), delete `tarpaulin-report.html`, delete the root `TODO.md` and `docs/TODO.md` (the latter is a single stale description of logging infrastructure that `src/logging.rs` already implements), and add a `[package] exclude` list covering `.github/`, `.cargo/`, `tarpaulin-report.html` and `TODO.md` as a guard against their return. Keep `docs/`, `tests/`, `benches/` and `examples/` in the package: they are useful in the tarball and conventional. The committed `Cargo.lock` stays: it pins the repository's own builds for reproducible CI, and Cargo includes a minimized copy in every published package regardless, so it cannot constrain a dependent's resolution.

Add `CHANGELOG.md` at the repository root with a `## [1.1.0]` section. It must state: the new `matrix` and `unified` modules; `build_unified()`; the backend features, the selection priority, and which scalar bound each backend requires; that `build()` is unchanged; the dependency moves relative to v1.0.1 (`rand` 0.8 → 0.9, `rand_distr` 0.4 → 0.5, `opencv` 0.95 → 0.97, `adskalman` and `once_cell` dropped); and the behaviour fixes listed in D7. It must **not** describe the 1.1 development cycle.

Set `Cargo.toml:3` to `1.1.0` as the last commit before tagging.

Two manifest corrections go with this: `[target.'cfg(target_os = "macos")'.features]` (`Cargo.toml:55-57`) is not a table Cargo reads, so the `accelerate` feature it defines does not exist and `--features accelerate` is rejected at feature resolution (DES-64 records the reproduction). Delete it, and delete `docs/BLAS_SETUP.md`'s Apple Accelerate, Intel MKL, netlib and blis instructions, which name features that are commented out in `Cargo.toml:41-46`. The guide must describe only what the manifest offers.

### D5 — A declared feature does something or says it does not

`serde`, `parallel` and `opencv` pull dependencies and change no behaviour: there is no `cfg(feature = "serde")`, no `rayon::` and no `opencv::` under `src/`. v1.0.1 declared them too, and the README table (`README.md:142-152`) presents them as working.

For 1.1: keep the feature names — removing them is a breaking change for anyone who lists them — and make the README and `CONTRIBUTING.md` say plainly that they only enable the optional dependency and change no behaviour. Implementing them is DES-73 and belongs to 1.2; removing them if they are still inert belongs to 2.0 (D9).

**Rejected:** *implement `parallel` now* — it changes floating-point reduction order in the ensemble and particle filters, so it needs an equivalence test against the serial path and a benchmark, which is a bigger job than the rest of this release's correctness work combined.

### D6 — Delete dead code now, keep the warning suppressions until 1.2

The release deletes `src/legacy.rs`. It is not declared in `src/lib.rs:42-62`, nothing enables it, it would collide with `filter::KalmanFilter` if wired in, and its `correct()` discards update errors. The README also advertises features named `legacy` and `adskalman` that the manifest does not define.

Keep `#![allow(unused, non_snake_case)]` in `src/lib.rs:40` and the per-module `#![allow(unused)]` lines for 1.1, and keep the "DO NOT CHANGE" note meaningful by replacing the bare note with the reason: the suppressions are removed in 1.2 together with the warning sweep they are hiding (DES-67). Removing them in 1.1 would surface an unknown number of warnings in files this release already edits, and a warning sweep is not part of a backend release.

### D7 — The release ships no test that cannot fail, and no filter that panics on bad input

`tests/quickcheck_invariants.rs:230-232` returns `true` unconditionally, so one property test proves nothing, and `docs/VALIDATION.md:27` records "4/6" as if that were a coverage number. It is replaced with a real invariant against the current `InformationFilter` API, or deleted and the count corrected. `docs/VALIDATION.md:126` also tells the reader to run modules in `tests/validation/`, a directory that does not exist.

The same rule covers the shipped filters: a wrong-sized matrix, a zero-particle filter, a one-member ensemble, a `alpha = 0` UKF or an ill-conditioned covariance must produce a `KalmanError` or a correct number, never a panic or a NaN. That is the substance of DES-56, DES-57, DES-60, DES-62 and DES-63, each with a regression test.

### D8 — Docs describe the release, not a plan

- `README.md`: version in the install snippet, the feature table (real features only, D5), the backend section (D2's commands), no Joseph-form claim — the covariance update is `P = (I - K H) P` plus symmetrization (`src/filter.rs:329-349`); no `legacy` or `adskalman` features; `build_unified()` documented.
- `CONTRIBUTING.md`: `Rust 1.70 or later` (`:19`) is false — the manifest sets `edition = "2024"` (`Cargo.toml:10`), which needs 1.85. Declare a `rust-version` equal to the oldest toolchain the gate verifies, and state the same number in the README badge and `CONTRIBUTING.md`.
- `docs/BLAS_SETUP.md`: rewrite to the D2 feature set, naming `openblas-system` as the tested path and marking `openblas-static` untested (DES-76).
- `docs/ROADMAP.md`: rewrite `Current State Assessment` against what 1.1.0 actually provides, and remove the stale per-item `Status:` lines and its dated footer. Open work is tracked in Linear; the roadmap keeps the long-term shape.
- `docs/SOTA.md`: drop the `v1.0.0` reference and the "Cross-validation Tests ✅" row, and reconcile the "actual benchmarks have not been run yet" note with whatever D3 measures — numbers, or a plain statement that no comparison exists.
- `docs/VALIDATION.md`: restate the test inventory and remove the `tests/validation/` reference.

### D9 — The breaking changes get one home: a 2.0 track, branched at the tag

Tracked as DES-75.

Everything that would break a v1.0.1 build, or remove or rename a public item, is collected here rather than sprinkled through 1.x releases:

- `build()` returns `UnifiedKalmanFilter<T>`, and `build_unified()` becomes the deprecated alias.
- Backend exclusivity is enforced: more than one backend feature is a `compile_error!`, `--all-features` is rejected explicitly, and `dev` loses `nalgebra`.
- Filter struct fields become private behind accessors, which `docs/ROADMAP.md:160-207` already schedules for a major.
- `NonlinearSystem` splits into `NonlinearDynamics` and `LinearizableSystem` (`docs/ROADMAP.md:330-365`).
- The `serde`, `parallel` and `opencv` features are removed if they still do nothing (DES-73), rather than kept as documented no-ops.

**The branch is `2.0-dev`, created from the `v1.1.0` tag once it is pushed — not before.** A branch opened before the tag would diverge over exactly the files 1.1 rewrites: `src/builder.rs`, `src/matrix/mod.rs`, `Cargo.toml`'s feature table, and the README. Every 1.1 commit would then become a merge on the branch that is supposed to stay nearly empty, and the two versions of the feature table would drift while 1.1 is still unverified. Branching at the tag gives 2.0 a base that is already green under the gate below, and costs nothing: the 2.0 work does not start any earlier either way.

The cost of deferring is one method name (`build_unified`) that 2.0 deprecates, and one guard condition in `src/matrix/mod.rs` that 2.0 replaces with a `compile_error!`. That is a cheap toll for a coherent compatibility story.

**Rejected:** *open `2.0-dev` now* — see above; duplicated work on the files 1.1 rewrites. *No 2.0 track, keep breaking changes inside 1.x* — then every minor release carries a compatibility question and the CHANGELOG has to explain which breaks landed where.

---

## Work order

Sequenced so each step leaves the tree buildable and the later steps do not have to be redone.

1. **Package hygiene** (D4, D6, DES-74): remove `.cargo/config.toml`, `tarpaulin-report.html`, root `TODO.md`, `docs/TODO.md`, `src/legacy.rs`; add `[package] exclude`; delete the macOS manifest table.
2. **Backend selection** (D2, DES-59): `BackendScalar`, guarded `cfg`s, the backend-identity test, provider features, the two `compile_error!` messages for configurations v1.0.1 could not build either. Largest step.
3. **Builder split** (D1, DES-55): `build()` / `build_unified()` over one validation path; examples, benchmark and doctests follow.
4. **Correctness fixes**, each with a regression test: DES-56 (EnKF inflation applied around the pre-forecast mean), DES-57 (builder panics on wrong-sized matrices), DES-60 (condition estimate reports a singular matrix as well conditioned), DES-62 (zero particles, one-member ensemble), DES-63 (`alpha = 0` UKF returns NaN).
5. **Test and CI truth** (D2 CI jobs, D7, DES-69, DES-72): replace the no-op property test; bound the `quickcheck` generator so it cannot emit `inf`/`NaN` and flake the publish gate; make the benchmark workflow fail with `set -o pipefail` or `bash -eo pipefail`.
6. **Docs rewrite** (D3, D5, D8).
7. **Measure** (D3), then write `CHANGELOG.md` and the release notes from the result.
8. **Release**: set the version to `1.1.0`, tag `v1.1.0`, let `publish.yml` publish, confirm docs.rs builds.

---

## Release gate

Every line must pass before the tag. None of them has a recorded result in the repository, so this is the gate to run, not a status report.

| # | Command | Expected |
| --- | --- | --- |
| 1 | `cargo test --release` (default features) | green, three consecutive runs (the publish gate must not be flaky) |
| 2 | `cargo test --release --no-default-features --features std,naive-matrix` | green |
| 3 | `cargo test --release --no-default-features --features std,nalgebra` | green |
| 4 | `cargo test --release --features nalgebra` (defaults on) | green, and the backend-identity test reports `NalgebraBackend` — the priority rule, exercised |
| 5 | `cargo test --release --no-default-features --features std,openblas-system` | green, with `libopenblas-dev` installed |
| 6 | `cargo check --no-default-features --features std` | fails with the no-backend `compile_error!` |
| 7 | `cargo check --no-default-features --features std,blas-backend` | fails with the "pick an OpenBLAS provider" `compile_error!` |
| 8 | `cargo check --examples && cargo bench --no-run && cargo test --doc` | green |
| 9 | `cargo package --list` | no `.cargo/`, no `TODO.md`, no `tarpaulin-report.html`, no `.github/` |
| 10 | `cargo publish --dry-run` | succeeds |
| 11 | `cargo doc --no-deps --features nalgebra` | builds; `StaticKalmanFilter` appears |
| 12 | `README.md` quick start compiled as a doctest | green — the snippet every new user copies |

---

## Alternatives rejected for the release as a whole

**Ship 1.1 without the compatibility split.** Cheapest, and wrong: it breaks v1.0.1 callers, ships an advertised backend that no feature combination selects, publishes host configuration, and documents four things the code does not do. A published version cannot be withdrawn.

**Add a headline feature so the minor bump feels earned.** An RTS smoother (roadmap item 11) would be the natural candidate. Rejected: semver does not require a feature for a minor bump, the new public surface (`matrix`, `unified`, `build_unified`) already justifies it, and adding a numerical algorithm before the backends and the tests underneath it are verified compounds the risk this release exists to retire.

**Do the whole cleanup now: private fields, warning sweep, duplicated inversion, seedable RNG.** Rejected: private fields are a breaking change the roadmap already schedules for a later major (`docs/ROADMAP.md:203-207`), and DES-61, DES-66 and DES-67 are refactors whose risk is unrelated to shipping the backends. They stay tracked for 1.2.

---

## Risks

| Risk | Mitigation |
| --- | --- |
| The `BackendScalar` bound changes the bound on public filter types. | It widens only where v1.0.1 could not express the configuration (nalgebra, BLAS); the default build's blanket impl still covers every `T: KalmanScalar`. The CHANGELOG states the one case that can break: a hand-written `KalmanScalar` impl combined with `nalgebra` or a BLAS provider. If it turns out to matter, the fallback is to implement each backend's traits for `T: KalmanScalar` and convert internally, at a cost in speed. |
| The priority rule is silent when a caller enables two backends, which is the class of bug this release fixes. | The priority is documented in `src/matrix/mod.rs`, in the README, and pinned by a test that asserts which type `DefaultBackend` is under each configuration. Two-backend selection becomes a hard error in 2.0 (D9). |
| The OpenBLAS CI job is slow or flaky (build from source needs a Fortran toolchain). | CI uses `openblas-system`, which needs only `libopenblas-dev`; `openblas-static` is documented as untested in 1.1 and gets its own scheduled job in 1.2 (DES-76). |
| Restoring `build()` re-touches the files DES-55 already changed, risking a regression in examples or the benchmark. | Gate items 8 and 12 cover exactly the targets DES-55 broke. |
| docs.rs builds on five targets by default; a backend-gated crate may fail on one of them. | The nalgebra feature is pure Rust, so cross-target builds should hold; confirm on the docs.rs build log rather than assuming. |

---

## Open questions

1. **DES-58's premise no longer matches the manifest.** The issue says the default set is `["std", "naive-matrix", "openblas-static", "opencv", "parallel", "serde"]`; `Cargo.toml:19` reads `["std", "naive-matrix"]`. Verify with `cargo tree -e normal --depth 1` and close the issue rather than re-fixing it.
2. **Which MSRV is real?** Edition 2024 sets a floor of 1.85, but the dependency set (`nalgebra` 0.34, `rand` 0.9) may set a higher one. Resolve with a `cargo msrv`-style bisect or by installing a candidate toolchain in CI; then declare `rust-version` and quote it in the docs.

---

## After the tag

**Open the 2.0 track** (D9): create `2.0-dev` from `v1.1.0`, and record the five items listed there as the branch's scope, so the branch starts empty and every commit on it is a deliberate break.

**Continue 1.2 on `master`**, in parallel with 2.0. Already tracked and deliberately excluded here: a scheduled job that builds and tests `openblas-static` (DES-76), seedable RNG for the particle and ensemble filters (DES-61), the duplicated Gauss-Jordan inversions and the information filters bypassing the matrix backend (DES-66), the remaining dead imports and the warning-suppression sweep (DES-67), a non-Linux CI matrix (DES-70), 32-bit dimension-overflow coverage (DES-71), and the serde/parallel/opencv features (DES-73). The only file the two tracks contend for is `src/matrix/mod.rs`, where 1.2 must not touch the backend guards that 2.0 replaces.

The roadmap's larger items — smoothers, adaptive tuning, IMM, square-root forms, constrained filtering, GPU, `no_std` fixed-size filters, WASM demos — are unaffected by this release and stay in `docs/ROADMAP.md` as long-term direction rather than commitments.
