# Differential test

270 cases. Consensus = the reference oracle. Tolerance: abs 1e-09 + rel 1e-07·|consensus| (cases may set their own rel tol).

| target | agree | value≠ | status≠ | missing | max rel err |
|---|---|---|---|---|---|
| spec (original code) | 249 | 5 | 16 | 0 | 6.5e-02 |
| A (data-oriented) | 263 | 2 | 5 | 0 | 6.7e-02 |
| B (functional-core) | 262 | 2 | 6 | 0 | 7.5e-01 |
| C (nalgebra-native) | 266 | 3 | 1 | 0 | 1.8e-02 |
| final | 265 | 0 | 5 | 0 | – |

## Disagreements (most dissent first, up to 30)

- `illcond00` consensus `ok`: spec:rel1.8e-02, A:error:SingularMatrix, B:error:SingularMatrix, C:rel1.8e-02, final:error:SingularMatrix
- `illcond04` consensus `ok`: spec:rel6.5e-02, A:error:SingularMatrix, B:rel7.5e-01, C:error:SingularMatrix, final:error:SingularMatrix
- `illcond18` consensus `ok`: spec:rel5.4e-04, A:rel6.7e-02, B:rel8.2e-04, C:rel1.5e-02, final:error:SingularMatrix
- `edge_nan_q` consensus `error:NonFiniteInput`: spec:ok, A:error:InvalidNoiseCovariance, B:error:InvalidInput
- `edge_zero_meas_dim` consensus `error:ZeroDimension`: spec:ok, A:error:InvalidDimension, B:error:InvalidInput
- `edge_zero_state_dim` consensus `error:ZeroDimension`: spec:ok, A:error:InvalidDimension, B:error:InvalidInput
- `illcond19` consensus `ok`: A:rel2.7e-04, C:rel7.1e-04, final:error:SingularMatrix
- `edge_inf_f` consensus `error:NonFiniteInput`: spec:ok, B:error:InvalidInput
- `edge_nan_measurement` consensus `error:NonFiniteInput`: spec:ok, B:error:InvalidInput
- `edge_bad_p0` consensus `error:InvalidCovariance`: spec:ok
- `illcond10` consensus `ok`: final:error:SingularMatrix
- `illcond15` consensus `ok`: spec:rel2.8e-04
- `illcond16` consensus `ok`: spec:rel6.6e-04
- `scale00_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale04_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale08_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale12_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale16_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale20_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale24_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale28_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale32_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance
- `scale36_1e-6` consensus `ok`: spec:error:InvalidNoiseCovariance

A target in the minority is not automatically wrong (the majority can share a bug, and the original code is one voter). Investigate each disagreement cluster.
