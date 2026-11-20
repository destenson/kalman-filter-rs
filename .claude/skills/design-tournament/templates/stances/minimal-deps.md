**Minimal dependencies, own the code.** Use only the language's standard library plus
dependencies the spec mandates. Implement what you need, small and well-tested, rather
than pulling in frameworks. Keep the dependency tree and the build fast and auditable.
Avoid: adding crates/packages for small conveniences; optional feature matrices.
Likely failure mode: reimplementing hard things (e.g. numerics) less robustly.
