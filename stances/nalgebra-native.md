**Lean on nalgebra entirely.** One filter type generic over nalgebra `Dim`, so static
and dynamic dimensions go through the same code via `OMatrix<T, N, N>` and friends.
Use nalgebra's Cholesky and in-place operations. No custom matrix layer at all:
delete src/matrix (keep only what other modules need to compile, as thin shims).
Avoid: wrapping nalgebra in your own matrix abstraction.
Likely failure mode: generic bounds sprawl (`DefaultAllocator: Allocator<…>`).
