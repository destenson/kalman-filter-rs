(Files starting with "_" are ignored. Copy this to e.g. probes/add-export.md.)

A good probe is a realistic next feature request that:
- is NOT mentioned in SPEC.md or the brief (candidates must not design for it),
- is small enough for one agent session (roughly 50-300 lines in a reasonable design),
- cuts across the design in a way that distinguishes good structure from bad
  (touches the core abstraction, not just adds a leaf function),
- has a clear, testable acceptance criterion stated in the request.

Use 2-3 probes that stress different axes (a new variant of the core concept; a
cross-cutting concern such as serialization or telemetry; a performance or API change).

Example:
  Users need to persist and restore filter state. Add `save`/`load` to a JSON string
  that round-trips exactly (state, covariance, all model matrices). A filter restored
  from a saved state must produce bit-identical results on the next 100 steps.
