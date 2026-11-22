Environment facts (not part of the request):
- Work only inside /home/dennis/src/kalman-filters/.tournament/linear-core/worktrees/probe-B-observer-hook ; that is the root of the repository copy you are editing.
- Always run cargo with CARGO_HOME=/home/dennis/src/kalman-filters/.tournament/linear-core/cargo-home exported, or the dependency cache is read-only and the build fails.
- Use -j 4 when building so parallel agents do not starve the machine.
- If DESIGN_NOTES.md does not exist in this copy, read the code and SPEC.md instead, and say so in your notes.

You are a maintainer who just inherited this codebase, which you have never seen before.
A feature request has arrived:

---
Users want to monitor filter health: after every update, report the innovation, its
covariance S, and the normalized innovation squared (NIS = yᵀS⁻¹y) to a user-supplied
callback/observer. It must cost nothing measurable when no observer is registered.
Add a test showing that the mean NIS over 5,000 steps of a correctly specified
simulated model is within 10% of the measurement dimension m.
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
