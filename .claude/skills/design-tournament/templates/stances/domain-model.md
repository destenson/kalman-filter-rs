**Rich domain model.** Model the problem's concepts as first-class types with their
invariants enforced at construction ("make illegal states unrepresentable"). Behavior
lives on the types that own the data. Names come from the domain vocabulary. Validate
once at the boundary; the interior trusts its types.
Avoid: primitive obsession (bare floats/vecs/strings standing in for concepts),
validation scattered everywhere.
Likely failure mode: too many small types, ceremony for simple cases.
