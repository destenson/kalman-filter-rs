Add control input support: predict with x = F·x + B·u, where B is n×k and u (length k)
is supplied per predict call. Filters built without B must behave exactly as before.
Add tests, including one where a constant-acceleration control on a
position/velocity model produces the analytically expected position and velocity
after 10 steps (no measurement updates, Q = 0).
