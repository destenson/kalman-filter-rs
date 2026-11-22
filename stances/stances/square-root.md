**Square-root / factored form.** Propagate a Cholesky factor of P (or a UDUᵀ
factorization) instead of P itself, so positive semi-definiteness holds by
construction. P is exposed only as a computed view. Measurement updates use a
numerically stable factored update (e.g. QR-based or Bierman/Potter style).
Avoid: ever forming and storing a full P that can drift non-PSD.
Likely failure mode: complexity for small gains at low dimension; harder extensions.
