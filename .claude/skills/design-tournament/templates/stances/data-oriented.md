**Data-oriented / performance-first.** Start from the data layout and the hot path.
Preallocate; aim for zero allocation in steady-state operations; reuse buffers; keep
data contiguous; batch operations; know the cost of every call. Abstractions are
allowed only where they compile away.
Avoid: per-call heap allocation, needless copies, dynamic dispatch on hot paths.
Likely failure mode: API ergonomics and readability suffer; premature specialization.
