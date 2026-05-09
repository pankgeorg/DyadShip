# Investigation: Why does the rudder turn the ship too fast?

## Question

After the FixedTranslation refactor, the rudder produces a strong yaw moment
via its 52 m lever arm. The autopilot was tuned for the pre-refactor model
and is now unstable. Is the *plant* itself too sensitive — i.e., is yaw
resistance under-modelled — or is the issue purely in the controller?

## What our `Hull3DOF` models

The current `Hull3DOF.dyad` has decoupled, per-axis linear+quadratic drag
applied through a `WorldForceTorque` resolved in body frame:

```
forcer.force_x = Fx_extra - Du * u - Du_quad * u * |u|
forcer.force_y = Fy_extra - Dv * v - Dv_quad * v * |v|
forcer.torque = Mz_extra - Dr * r - Dr_quad * r * |r|
```

Defaults in `ShipTransient`: `Du = 5000`, `Du_quad = 4000`, `Dv = 1e6`,
`Dv_quad = 10000`, `Dr = 1e7`, `Dr_quad = 5e5`.

The three axes are **independent**. There is no cross-coupling between
surge, sway, and yaw — sway speed never affects yaw moment, yaw rate never
generates sway force, etc.

## What ShipSIM models — `HidrodynamicXYY` [sic] (MMG)

Component `ShipSIM.Components.HidrodynamicXYY` [sic — upstream misspelling
preserved for grep] (Components.mo:236) implements the **MMG standard
method** for ship maneuvering, citing Yasukawa 2015, Jialun 2020,
Taimuri 2020. Our port (Strategy A below) will spell it `HydrodynamicXYY`
or `HullMMG`. The forces and yaw moment are:

```
NonDimXY = 0.5·ρ·Lpp·d·U²
NonDimN  = 0.5·ρ·Lpp²·d·U²

# Non-dimensional sway and yaw rate
v = SpeedLocal[2] / U                # sway / total apparent speed
r = AngularSpeed[3] · Lpp / U        # yaw·length / speed (Froude-style)

# Surge force
F_x = -sign(u) · sum(R0 + R1·u + R2·u² + R3·u³)         # resistance curve
    + NonDimXY · (X_vv·v² + X_vvvv·v⁴ + X_rr·r² + X_vr·v·r)

# Sway force
F_y = NonDimXY · (Y_v·v + Y_vvv·v³ + Y_r·r + Y_rrr·r³ + Y_vrr·v·r² + Y_vvr·v²·r)

# Yaw moment
M_z = NonDimN  · (N_v·v + N_vvv·v³ + N_r·r + N_rrr·r³ + N_vrr·v·r² + N_vvr·v²·r)
```

Plus **added mass** terms (separate `WorldForceAndTorque`):

```
F_x_added = -Δ·(mx·d(u)/dt − my·v·r)        # mx ≈ 5%, my from Zhou 1983
F_y_added = -Δ·(my·d(v)/dt + mx·u·r)
M_z_added = -Δ·Jz·d(r)/dt                    # Jz from Zhou 1983
```

Hydrodynamic derivative coefficients are **derived from ship geometry**
(`Lpp`, `B`, `Draft`, `Cb`) using empirical formulae from Clarke 1982,
Smitt 1970, Khattab 1984, Lee & Shin 1998, Kijima 1990, Yoshimura &
Masumoto 2011. So a user who knows hull form gets reasonable maneuvering
behavior without per-ship calibration.

## Differences that matter

1. **Cross-coupling**. ShipSIM's yaw moment depends on **sway** (`N_v·v`,
   `N_vvv·v³`, `N_vrr·v·r²`, `N_vvr·v²·r`), and the surge force depends on
   yaw rate (`X_rr`, `X_vr`). Ours doesn't.

   Practical effect: when our ship is rotating, the rotation slows surge
   only via lost-thrust-direction; in MMG it *additionally* slows surge
   via X_rr·r², and the sway induced by the turn (Y_r·r) feeds back into
   yaw via N_v·v — a classic stabilizing coupling.

2. **Cubic yaw damping**. ShipSIM has `N_rrr·r³` and `N_r·r` weighted by
   `NonDimN ∝ U²`. At U = 5 m/s, r = 0.1 rad/s the linear yaw damping
   alone is ~1.5 MN·m — comparable to our `Dr·r ≈ 1 MN·m`. **But our
   model has no `r³` term**, so high yaw rates are under-damped. The
   ratio `N_rrr/N_r ≈ 0.05/0.015 ≈ 3` for our hull — at r = 0.5 rad/s
   the cubic term contributes 25× more than the linear, which would
   strongly resist runaway rotation we currently see.

3. **Speed-scaled damping**. ShipSIM's Y_r and N_r are dimensionless
   coefficients multiplied by `0.5·ρ·Lpp·d·U²·(Lpp/U) = 0.5·ρ·Lpp²·d·U`.
   So **yaw damping grows with speed**. Ours is constant. At low U our
   damping is *too much*; at high U it's *not enough*.

4. **Added mass**. The hull's effective mass for yaw acceleration is
   `Iz · (1 + Jz)` where Jz is ~5% of displacement (ours is `Iz` only).
   Effect on stability is small at our speeds.

5. **Resistance curve**. ShipSIM uses a polynomial in u (cubic by default,
   from a fit to towing-tank data). We use linear+quadratic. For surge
   that's roughly equivalent within our operating range — not the issue.

## Diagnosis

The runaway behavior of the autopilot is **mostly the controller** (a
naive PD on heading-error against a strong-rudder plant), but ShipSIM's
hull would also be **harder to spin** because:

- the cubic `N_rrr·r³` damping kicks in once r > ~0.2 rad/s, sharply
  resisting further rotation;
- the cross terms convert spin into sway, which the large `Y_v` and
  `Y_vvv` damp (since broadside motion meets a lot of water);
- in steady-state turn, `Y_r·r` and `N_v·v` create a self-balancing
  closed loop that the controller doesn't have to fight.

So both a better hull *and* a better autopilot are needed. Either one
alone won't be enough — but with both, the system should be tunable.

## Strategies (in order of leverage)

**Strategy A — Implement MMG-style hull (`HullMMG.dyad`).**
Largest fidelity gain. Adds ~30 parameters (most computed from
`Lpp`, `B`, `Draft`, `Cb`) and 6-8 cross-coupling terms. Re-parameterize
existing analyses to use it. Backward-compatible: keep `Hull3DOF` for
analyses that don't need maneuvering accuracy.

**Strategy B — Cheap proxy: heavy `Dr_quad` and add `Dr_cube`.**
Bump `Dr_quad` to ~5e6 (10× current), add a `Dr_cube·r·r²` term to
mimic `N_rrr·r³`. Doesn't capture cross-coupling but makes spin much
harder. ~5 lines of code.

**Strategy C — Speed-scaled damping.**
Multiply `Dr` by `(1 + |u|·Lpp / ν_ref)` so yaw drag scales with speed.
Captures point #3 above without full MMG.

**Strategy D — Fix the autopilot.**
Saturated PI with integral wind-up protection (or LQR / MPC). Doesn't
fix the missing physics but stops the runaway.

**Recommendation**: do **A + D** in that order. A is more work but
gives a defensible "real ship" baseline against which D can be tuned.
B is a useful middle step to confirm the hypothesis cheaply.

## Empirical test of Strategy B

Added `Dr_cube` parameter to `Hull3DOF` and ran with `Dr_cube = 1e9`,
`k_rudder = +0.5`, deadband = 3°. Diag of `psi` vs `t`:

```
t=10s:   psi=0.003 rad,   course_diff=0.10,  cmd=+0.05°
t=100s:  psi=10.7 rad,    course_diff=1.95,  cmd=+0.88°
t=1000s: psi=146.6 rad,   course_diff=-2.04, cmd=-1.09°
t=3600s: psi=539 rad,     course_diff=1.10,  cmd=+0.47°
```

Yaw *rate* dropped from ~0.3 rad/s (no cubic damping) to ~0.15 rad/s
(with `Dr_cube = 1e9`) — cubic damping does its job — but **the
closed loop is still diverging**: psi grows monotonically while the
autopilot's command oscillates between + and − without ever reversing
the rotation.

That tells us the dominant problem is controller design, not the missing
hull damping: the PD law's K_eff effectively has the wrong sign relative
to the rudder→yaw chain on the *lever-arm-corrected* plant, so any
non-zero gain produces monotonic divergence rather than the damped
oscillation a P-on-heading controller should give.

## Verdict

- **Plant fidelity**: yes, our hull is missing the MMG cross-couplings and
  the cubic yaw damping. Adding them (Strategy A) is worthwhile and
  straightforward — most coefficients are computed from `Lpp`, `B`,
  `Draft`, `Cb` via the empirical formulae in Components.mo:248-292.
- **Stability**: missing damping is not the load-bearing cause of the
  autopilot runaway. A proper controller redesign (Strategy D) is
  required regardless of hull fidelity.
- **`Dr_cube`** (newly added to `Hull3DOF`) is kept as an opt-in
  parameter, defaulting to 0. Useful for any downstream user who wants
  a closer-to-MMG response without the full port.

## Recommended follow-ups

1. **Port `HidrodynamicXYY` [sic] → `HydrodynamicXYY` or `HullMMG`**
   (Strategy A). Implements the full MMG damping matrix and added-mass
   couplings; gives a defensible "real ship" baseline.
2. **Replace the `SimpleAutoPilot` PD law** with a heading-aware PI
   that uses ψ directly (not `atan(vel_y, vel_x)`) and clamps the
   integral term. Verifies sign convention in unit-test isolation
   before going closed-loop with the strong-rudder plant.
3. (Optional) Vendor the `XYZ_v`, `XYZ_r`, etc. MMG coefficients with
   Lee/Shin/Kijima/Yoshimura formulae as derived parameters in dyad
   so a user only sets `Lpp/B/Draft/Cb` to get reasonable maneuvering.
