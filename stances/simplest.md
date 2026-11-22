**Simplest thing that works.** Write the most direct, concrete code that satisfies the
spec: plain functions and structs, no traits/interfaces/generics until a second
concrete use exists *in the spec*. Duplication is acceptable until the third
occurrence. Optimize for a newcomer reading top to bottom in one sitting.
Avoid: indirection layers, builders, "future-proof" extension points.
Likely failure mode to watch for: features from probes may require touching many places.
