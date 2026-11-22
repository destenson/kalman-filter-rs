**Functional core, imperative shell.** All domain logic is pure functions over
immutable values: inputs in, new values out, no hidden state, no I/O, no logging inside.
A thin outer shell owns mutation, I/O, configuration, and telemetry, and calls the core.
Errors are values. Make the core trivially unit-testable.
Avoid: methods that mutate shared state deep in the logic; side effects in the core.
Likely failure mode: allocation/copy overhead, awkward APIs for stateful use.
