Users want to monitor filter health: after every update, report the innovation, its
covariance S, and the normalized innovation squared (NIS = yᵀS⁻¹y) to a user-supplied
callback/observer. It must cost nothing measurable when no observer is registered.
Add a test showing that the mean NIS over 5,000 steps of a correctly specified
simulated model is within 10% of the measurement dimension m.
