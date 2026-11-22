**Correctness and robustness.** Does the code do exactly what SPEC.md says, including
error and edge cases? Look for: wrong math/logic, inputs silently ignored, panics or
crashes reachable from the public API, unchecked unwraps/indexing, precision or
overflow issues, error types that lose information, behavior only "tested" by tests
that cannot fail. Untested paths in critical logic count against a candidate.
Frozen-contract violations (`frozen⚠` in the metrics) are disqualifying unless clearly
justified in the notes.
