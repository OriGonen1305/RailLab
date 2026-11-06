# Geometry and physics

Coordinates are meters, Y points upward, speed is meters per second, and the cart follows the point list in its original order. The model describes one point mass held to the track. The independent variable is traveled arc length `s`; there is no animation clock.

## From input points to Bézier segments

Input points are interpolation anchors, not raw Bézier control handles. A generic Bézier curve does not pass through every control point, so the program first constructs a natural parametric cubic spline through the anchors.

Let `h_i = |P_(i+1) - P_i|` be the chord lengths. Solve the tridiagonal system for the vector second derivatives `M_i`:

```text
M_0 = M_(n-1) = 0

h_(i-1) M_(i-1) + 2(h_(i-1) + h_i) M_i + h_i M_(i+1)
    = 6 [(P_(i+1) - P_i)/h_i - (P_i - P_(i-1))/h_(i-1)]
```

Both X and Y use the same chord-length parameter. The spline is C² in that parameter. For each interval, compute its endpoint derivatives `D_start`, `D_end`, then form:

```text
Q0 = P_i
Q1 = P_i     + (h_i/3) D_start
Q2 = P_(i+1) - (h_i/3) D_end
Q3 = P_(i+1)

B(u) = (1-u)^3 Q0 + 3(1-u)^2 u Q1 + 3(1-u)u^2 Q2 + u^3 Q3
0 <= u <= 1
```

The natural boundary condition gives zero endpoint second derivatives. Non-monotone X is permitted, so the path can have vertical segments, loops and self-intersections. Intersections are simply different positions along the traversal; they do not become junctions. A repeated nonadjacent point is allowed, but adjacent points must be at least 0.05 m apart. The spline may overshoot the anchors; the preview shows that actual curve before confirmation.

Analytical derivatives give the unit tangent and signed curvature:

```text
T = B'(u) / |B'(u)|
n = (-T_y, T_x)                         left normal
kappa = (x' y'' - y' x'') / |B'(u)|^3
```

The code examines stationary values of `|B'|²` and rejects curves with nearly zero derivative. This avoids dividing by a zero tangent at cusps. For regular curves, tangent direction and curvature are continuous at joins even when neighboring intervals use different local `u` scales.

Arc length is `integral |B'(u)| du`. Adaptive five-point Gauss–Legendre integration builds a 64-cell lookup table per segment. A binary search and bracketed Newton refinement invert the table to find `u(s)`. Uniform increments in Bézier `u` would not correspond to uniform physical distances.

## Speed and dissipation

Let `w = v²`, `F(s)` be the selected segment's signed applied force in newtons, `r` be the constant opposing acceleration, and `b = rho * CdA / (2m)` the quadratic drag coefficient per unit mass. The applied vector is `F(s) T(s)`: positive force propels along the local forward tangent, negative force brakes opposite travel, and zero disables it. The forward tangential dynamics are:

```text
a_t = F(s)/m - g T_y - r - b v²
dw/ds = 2F(s)/m - 2g T_y - 2r - 2b w
```

The simulation steps through distance. Without aerodynamic drag it evaluates the energy formula directly, with constant-resistance loss `ell(s) = r*s`:

```text
dell/ds = r + b v²
W_motor(s) = integral from 0 to s of F(q) dq
v² = v0² + 2g(y0 - y(s)) - 2ell(s) + 2W_motor(s)/m
```

With drag, subtracting accumulated losses from initial energy would destroy tiny remaining kinetic energies. Instead, the solver propagates `w` using the integrating-factor solution over each interval of length `h`:

```text
a(s) = F(s)/m - r - g*T_y(s)
w(s+h) = exp(-2*b*h)*w(s) + 2*integral[0,h] exp(-2*b*(h-q))*a(s+q) dq
```

The exponential decay and constant part of `a` are integrated exactly, using `expm1` to retain precision for small arguments. Five-point Gauss quadrature integrates the remaining changing-gravity term, using two half-steps per output interval. Dissipated energy is then recovered from the energy balance for reporting; it is never subtracted from initial energy to recover the drag-limited speed. On a horizontal segment with zero net driving acceleration this preserves `v = v0*exp(-b*s)`, even after most of the initial energy has dissipated.

Each integration interval ends at or before the next confirmed point's exact arc distance. Within it the force is constant, so applied work advances by `F * delta_s`, including intermediate integration evaluations and a refined stopping position. Braking contributes negative work only up to that stop. `W_motor` (and the existing `propulsion_work_J` CSV column) denotes this signed applied work; braking is not also added to the resistance/drag loss `D`. At a point boundary, the reported force and acceleration use the outgoing segment; the final point uses the last segment. Work and speed stay continuous. No force or work from an unreached segment is included. Setting all forces to zero recovers the unpowered equations.

If the cart decelerates to zero at a boundary, a positive outgoing net acceleration can start it there. The comparison allows only an energy-scaled floating-point roundoff bound, not a fixed distance or speed threshold. With drag, the bound carries arithmetic error from earlier steps, attenuated by the same exponential decay as kinetic energy. It is used only at an actual segment boundary with positive outgoing acceleration, never as a speed cutoff. Stops farther short than that numerical uncertainty still terminate the run; regressions include a real gap of `1e-12 m` before a motor, both after resistance and after braking with or without drag.

The constant resistance term is an approximate energy-loss model, not a model of static friction, rolling contact or wheel slip. The point mass can travel only forward in this analysis. From rest it starts only if the initial net forward acceleration is positive.

## Acceleration, rail force and G-load

```text
a_n = v² kappa
a_vector = a_t T + a_n n
|a| = sqrt(a_t² + a_n²)

N = m (v² kappa + g T_x)
normal_G = N / (m g)
felt_G = sqrt(normal_G² + ((F(s)/m - r - b v²)/g)²)
```

`N` is the signed aggregate **normal** rail force. The centripetal term alone is not the rail force: gravity also has a normal component. A stationary cart on a flat rightward track has zero acceleration and `normal_G = 1`.

The sign convention follows the left normal to the direction of travel. The normal G plot can therefore be negative. The ideal rail constraint may pull as well as push; a negative value does not initiate free flight. `felt_G` is the magnitude of the resultant non-gravitational force, including signed propulsion/braking and opposing tangential resistance and drag, divided by `mg`. Applied segment force has no direct normal component, but its effect on speed changes the normal load on curves. It is not a multi-body passenger or restraint model.

## Energy and time plots

```text
K = m v² / 2
U = m g y                              datum: y = 0
D = m ell                              dissipated energy
E_mechanical = K + U
K + U + D - W_motor = initial total energy

delta_t ~= 2 delta_s / (v_previous + v_next)
```

The time formula is exact for constant tangential acceleration on an interval and an approximation otherwise. It remains finite for an ordinary uphill turning point where speed reaches zero with nonzero deceleration. At an equilibrium energy barrier, travel time can diverge; the program flags a near-zero terminal acceleration and does not present the elapsed-time summary as an ordinary finite ride time.

For a horizontal segment where propulsion balances constant resistance and only quadratic drag changes the speed, the clock is integrated exactly: `delta_t = expm1(b*delta_s)/(b*v_previous)`. Positive speeds have no arbitrary clock cutoff: uniform motion at `1e-13 m/s` over 10 m takes `1e14 s`. Nonfinite times are retained as infinity and flagged, rather than replaced by zero. CSV exports include the analysis-wide `time_resolution_limited` column (0 or 1), and the CLI reports unresolved time explicitly. On general curves, the time formula above remains a step-dependent estimate; agreement on the physical stopping position does not imply equal coarse/fine travel-time estimates.

The inclination plot uses `atan2(T_y, T_x)` in degrees, wrapped to `[-180, 180]`. A jump at the wrap boundary is a change in angle representation, not a discontinuity in the track.

## Numerical method and limits

- The selected step is a maximum distance step. The integrator also lands at segment boundaries and extrema of effective potential `g*y + (r - F/m)*s`, including unpowered segments with resistance. Solving `T_y = (F/m - r)/g` prevents an energy barrier from being hidden between height samples. It checks half-step and endpoint speed and refines a detected first zero by bisection. It does not continue through an unreachable hill or extrapolate graphs onto the untraversed track.
- For strong drag the step is further limited to `b * delta_s <= 0.1` to resolve changing gravity and the sampled plots. The decay itself is exponential, not an explicit RK stability approximation. Positive speeds are retained while representable; time close to equilibrium or beyond floating-point range is flagged as unresolved.
- Narrow features can still require smaller steps. Compare both the shape and reported peaks after halving the step. The maximum-speed/load cards report maxima of the sampled data, not proven continuous extrema. Local curvature and load can be very large for tightly spaced or poorly placed anchors.
- The `energyResidual` diagnostic checks the accounting identity. Because reported losses are recovered from that identity in the drag solver, a small residual alone is not an independent accuracy test. Tests compare drag with an analytical solution and curved propelled motion with an independent fine-step RK4 reference.
- Up to 128 points, coordinates within ±10 km, a track length of at most 100 km and at most 200,000 physics samples are supported. Reaching the sample limit is reported as incomplete analysis.
- Geometry plots follow the traversed samples, like the physics plots. The track view shows the full confirmed path, including any unreachable portion.
- The path is open. Coincident first/last positions do not enforce a periodic seam. Train length, wheel contacts, motor power/traction limits, brake hardware/thermal limits, free flight, rollback, banking and structural dynamics are outside this model. Segment propulsion and braking are ideal prescribed forces. A brake cannot launch a resting cart backward; motion remains forward-only.

The application is an educational numerical-analysis tool. Its results are not ride-design certification.

## Independent analytical checks

For a straight incline with constant downward tangent component `-sin(theta)`, no losses and length `L`:

```text
v_end² = v0² + 2gL sin(theta)
t_end = 2L / (v0 + v_end)
normal_G = cos(theta)
```

For a horizontal segment with constant signed applied force, drag and resistance:

```text
w_eq = (F/m - r)/b
w(s) = w_eq + (w0 - w_eq) exp(-2bs)   for b > 0, until the first zero
```

With no losses, a 100 kg cart starting from rest under 100 N over 5 m receives 500 J: `v_end = sqrt(2 * 500 / 100) = sqrt(10) ≈ 3.162278 m/s`. A following unpowered flat segment keeps this speed.

With no drag or resistance, a 100 kg cart entering at 2 m/s under -100 N has `a_t = -1 m/s²`. It stops after `s = v0² / (2*|a_t|) = 2 m`, in 2 s, with `W_motor = -200 J`. A 1 m braking segment instead contributes -100 J and leaves `v = sqrt(2) ≈ 1.414214 m/s` for a following unpowered flat segment.

For an uphill straight track without losses:

```text
s_stop = v0² / (2g sin(theta))
t_stop = v0 / (g sin(theta))
```

These expressions are tested directly, separately from the numerical stepping implementation.
