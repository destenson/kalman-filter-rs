**Behavior against an independent reference oracle.** Write a small, obviously-correct
reference implementation of the core behavior *inside your test file* (slow and naive
is fine), then compare the code under audit against it on many seeded random inputs
and on deliberately hard ones: boundary sizes, extreme magnitudes or lengths, unusual
but valid combinations, long sequences of operations, every supported type/precision.
Look for: wrong results, off-by-one and ordering errors, precision or overflow loss,
accumulated drift, thresholds that are absolute where they should be relative,
results that differ across supported types more than the types explain.
