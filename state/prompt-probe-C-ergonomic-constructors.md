Environment facts (not part of the request):
- Work only inside /home/dennis/src/kalman-filters/.tournament/linear-core/worktrees/probe-C-ergonomic-constructors ; that is the root of the repository copy you are editing.
- Always run cargo with CARGO_HOME=/home/dennis/src/kalman-filters/.tournament/linear-core/cargo-home exported, or the dependency cache is read-only and the build fails.
- Use -j 4 when building so parallel agents do not starve the machine.
- If DESIGN_NOTES.md does not exist in this copy, read the code and SPEC.md instead, and say so in your notes.

You are a maintainer who just inherited this codebase, which you have never seen before.
A feature request has arrived:

---
Users find the eight-slice `LinearKalmanFilter::new` hard to use for common cases. Add
ergonomic ways to construct a filter that take fewer parameters:

1. A builder (or equivalent) with sensible defaults. The caller supplies the model
   matrices F and H. The initial state defaults to zero. P0, Q and R can each be given as
   a full matrix, a diagonal, or a single scalar variance (meaning σ²·I).
2. A convenience constructor for the common constant-velocity model: `dims` spatial
   dimensions, time step `dt`, process-noise intensity `q`, and measurement variance `r`.
   Positions are measured directly.

Requirements: every path goes through the same validation as `new` (no second set of
rules), a missing or invalid piece is an error (never a panic), and both f32 and f64
work. Add tests showing that a filter from each helper produces exactly the same states
and covariances as an equivalent filter built with `new`, over 20 predict/update steps.
---

Implement it the way a careful maintainer would:
- First read enough of the code (start with `DESIGN_NOTES.md` and `SPEC.md`) to find
  where the change belongs.
- Follow the codebase's existing design and conventions. Do not restructure it to suit
  your taste. If the design makes the feature awkward, do the least-bad thing and say so.
- Keep every existing test passing (`cargo test --quiet --test spec_linear_kf && cargo test --quiet --lib`) and add tests for the new feature
  in new test files.
- **Never modify files under `SPEC.md tests/spec_linear_kf.rs`.**
- Do not refactor unrelated code. Do not commit; the harness commits.

Your final message goes to design reviewers. Make it 4-8 lines:
- Where the change landed (files) and why there.
- What made it easy (a seam or abstraction that already fit).
- What made it hard (what you had to work around, duplicate, or thread through).
- Anything you are unsure is correct.
