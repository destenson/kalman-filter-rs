Users find the eight-slice `LinearKalmanFilter::new` hard to use for common cases. Add
ergonomic ways to construct a filter that take fewer parameters:

1. A builder (or equivalent) with sensible defaults. The caller supplies the model
   matrices F and H. The initial state defaults to zero. P0, Q and R can each be given as
   a full matrix, a diagonal, or a single scalar variance (meaning σ²·I).
2. A convenience constructor for the common constant-velocity model: `dims` spatial
   dimensions, time step `dt`, process-noise intensity `q`, and measurement variance `r`.
   Positions are measured directly.

Requirements: every path goes through the same validation as `new` (no second set of
rules), a missing or invalid piece is an error (never a panic), and both f32 and f64
work. Add tests showing that a filter from each helper produces exactly the same states
and covariances as an equivalent filter built with `new`, over 20 predict/update steps.
