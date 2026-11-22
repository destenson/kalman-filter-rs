**Every input that can make the public API panic, hang, or silently misbehave.**
Systematically enumerate the input space of each public entry point: empty and
zero-sized, mismatched lengths and shapes, non-square where square is assumed,
NaN/±∞, subnormals, extreme magnitudes, repeated calls in unusual orders (update
before predict, many updates without predict), and state after an error (is it
unchanged?). A test that panics is a confirmed defect if the spec says errors must be
returned. Also look for inputs that are accepted but ignored.
