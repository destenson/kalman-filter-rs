# Metrics

| target | build | tests | passed | failed | LOC | files | lint | frozen⚠ | out-of-scope | unwraps | vec_allocs | cfg_attrs | cost $ | turns |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A (data-oriented) | ✅ | ✅ | 140 | 0 | 965 | 5 | 160 | 0 | 30 | 8 | 2 | 2 | 7.16 | 154 |
| B (functional-core) | ❌ | ✅ | 144 | 0 | 1061 | 6 | 149 | 0 | 29 | 16 | 3 | 2 | 8.00 | 247 |
| C (nalgebra-native) | ✅ | ✅ | 137 | 0 | 747 | 4 | 137 | 0 | 29 | 14 | 1 | 1 | 7.62 | 5 |
| final | ✅ | ✅ | 146 | 0 | 1247 | 6 | 170 | 0 | 30 | 11 | 2 | 2 | 5.63 | 176 |
| spec | ✅ | ❌ | 14 | 18 | 3744 | 12 | 141 | 0 | 0 | 27 | 206 | 99 | – | – |

**Out-of-scope changes:**
- A: benches/kalman_benchmarks.rs, examples/logging_demo.rs, examples/metrics_server.rs, examples/position_2d.rs, examples/position_2d_unified.rs, examples/sensor_fusion.rs, examples/sensor_fusion_unified.rs, examples/simple_1d.rs, examples/simple_1d_unified.rs, src/builders/information.rs, src/extended.rs, src/information/conversion.rs, src/information/distributed.rs, src/information/extended.rs, src/information/filter.rs
- B: benches/kalman_benchmarks.rs, examples/logging_demo.rs, examples/metrics_server.rs, examples/position_2d.rs, examples/position_2d_unified.rs, examples/sensor_fusion.rs, examples/sensor_fusion_unified.rs, examples/simple_1d.rs, examples/simple_1d_unified.rs, src/builders/information.rs, src/extended.rs, src/information/conversion.rs, src/information/distributed.rs, src/information/extended.rs, src/information/filter.rs
- C: Cargo.toml, benches/kalman_benchmarks.rs, examples/logging_demo.rs, examples/position_2d.rs, examples/position_2d_unified.rs, examples/sensor_fusion.rs, examples/sensor_fusion_unified.rs, examples/simple_1d.rs, examples/simple_1d_unified.rs, src/builders/information.rs, src/extended.rs, src/information/conversion.rs, src/information/distributed.rs, src/information/extended.rs, src/information/filter.rs
- final: benches/kalman_benchmarks.rs, examples/logging_demo.rs, examples/metrics_server.rs, examples/position_2d.rs, examples/position_2d_unified.rs, examples/sensor_fusion.rs, examples/sensor_fusion_unified.rs, examples/simple_1d.rs, examples/simple_1d_unified.rs, src/builders/information.rs, src/extended.rs, src/information/conversion.rs, src/information/distributed.rs, src/information/extended.rs, src/information/filter.rs

LOC excludes frozen test paths. Lower LOC is not automatically better; read it together with the probe results.
