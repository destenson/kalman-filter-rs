**Type-driven / compile-time guarantees.** Use the type system to make misuse
impossible to compile: generics and const generics for dimensions/units, typestate for
protocols (e.g. must initialize before use), newtypes for distinct quantities. Runtime
checks only where the types cannot reach.
Avoid: runtime dimension/shape checks that types could enforce; stringly-typed config.
Likely failure mode: generic signatures that are hard to read; dynamic sizes get awkward.
