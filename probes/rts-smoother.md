Add a Rauch–Tung–Striebel fixed-interval smoother: given a filter configuration and a
sequence of measurements, return smoothed states and covariances for every step.
The smoothed covariance trace must be ≤ the filtered trace at every step, and on a
1-D random-walk model the results must match a direct implementation of the textbook
RTS recursion to within 1e-9.
