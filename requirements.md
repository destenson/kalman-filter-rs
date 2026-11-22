# Requirements ledger

Every requirement that comes up during the tournament gets exactly one decision.
That covers ideas from the brief, the spec agent, auditors, the user, judges, probes
and the synthesis.

| Decision | Meaning | Required column |
|---|---|---|
| `KEEP` | In this run's SPEC.md. | `Ref`: the behavior ID, e.g. `B12` |
| `DEFER` | Deliberately left for a later tournament. | `Ref`: a tracker issue (link or key) |
| `DROP` | Rejected. | `Why` |
| `?` | Not decided yet. `tourney freeze` refuses while any remain. | — |

Sources: `brief`, `user`, `spec`, `audit:<lens>`, `fold`, `judge:<lens>`, `probe:<name>`,
`synthesis`, `previous:<run>`.

Candidates never see this file. SPEC.md may list non-goals, but must never say that
something is planned for later, because candidates would design toward it.

| ID | Requirement | Source | Decision | Why | Ref |
|---|---|---|---|---|---|
| R1 | Covariance update is Joseph-form or equally stable; solve with S, never invert it | brief | KEEP | | B11, Q1 (DES-147) |
| R2 | P stays symmetric PSD on ill-conditioned models, and every successful update leaves P PSD | brief, audit | KEEP | | B11, B21 (DES-147, DES-145) |
| R3 | No heap allocation in steady-state predict/update | brief | KEEP | | B12 (DES-153) |
| R4 | No panic reachable from the public API (zero dims, overflowing dimension products, bad lengths) | brief, audit | KEEP | | B3, B15, B19 (DES-152) |
| R5 | Construction inputs validated: lengths, PSD p0/q/r, finite values | brief, spec | KEEP | | B2, B4, B5, B6 (DES-151, DES-150) |
| R6 | Results and validity independent of units/scale (uniform and per component) | spec, audit, difftest | KEEP | | B10, B18, B22 (DES-148, DES-149) |
| R7 | predict/update never commit a non-finite estimate; errors leave the filter unchanged | audit | KEEP | | B9, B20 (DES-146) |
| R8 | Logging/metrics never interleaved with the math | brief | KEEP | | Q4 |
| R9 | One validated construction path, plus at most one delegating front end | brief, user | KEEP | | B16, Q5 (DES-155) |
| R10 | No public mutable fields; no accepted-but-ignored inputs | brief, spec | KEEP | | B13, B14 (DES-155) |
| R11 | Time-varying F, Q, H, R per step, so EKF/UKF/CKF can be built on the core | user | DEFER | keep this spec small; next tournament | DES-143 |
| R12 | DimensionMismatch reports the exact supplied shape | audit:claims-vs-code | DROP | field values deliberately unspecified in the contract | |
| R13 | Control input (x = F·x + B·u) | brief non-goal; hidden probe | ? | proposed DEFER: the probe measures how well each design takes it; decide after results | |
| R14 | Fixed-interval (RTS) smoother | brief non-goal; hidden probe | ? | proposed DEFER: same as R13 | |
| R15 | Treat a NaN measurement component as "missing" (skip it) instead of rejecting the update | user review | ? | proposed DEFER: rejecting is the safe default (B6) | |
| R16 | Defined behavior for R with an exactly zero diagonal entry | fold | ? | proposed DROP: unspecified on purpose; difftest will show if candidates diverge | |
| R17 | ndarray/BLAS backends | brief non-goal | ? | proposed DEFER to the existing backend issues | DES-59, DES-142 |
| R18 | no_std support (do not foreclose it) | brief non-goal | ? | proposed DEFER: needs a tracker issue | |
| R19 | Ergonomic construction: helper constructors and/or builders with fewer parameters, all delegating to `new`'s validation | user (after freeze) | KEEP | added at synthesis/build as an owner amendment; relaxes B16/Q5; measured by the ergonomic-constructors probe | B16 amended (DES-155) |
| R20 | Fallible constructors are named `try_*` and return `KalmanResult<Self>` (`try_new`, `try_build`, ...); plain names only for infallible ones | user (after freeze) | KEEP | final build uses try_new; temporary `new` forward removed by a mechanical rename after verification | spec §2 interface amended (DES-155) |
| R21 | The core needs no matrix library: nalgebra stays an optional feature, never a required dependency | user (after candidates) | KEEP | rules out candidate C as the base; A chosen | Q6 amended; DES-143 |
