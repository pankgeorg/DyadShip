# Successfully ported components

Each entry: source model → Dyad file → assumptions / notes.

## ⚙ MultibodyComponents.PlanarMechanics gotchas (apply to every multibody port below)

After the second pass these are the things that bit (or almost bit) us. They apply to every component that exposes a `Frame2D` connector or sits inside a multibody assembly.

- **`PlanarMechanics.World` is required** in every model that uses any PlanarMechanics body. Even one that "should not need gravity" must instantiate exactly one `World`. For ship models we set `g = 0`. Without it, you get `Could not evaluate value of parameter g. Missing values for variables in expression g.`
- **3D ShipSIM → 2D PlanarMechanics maps surge/sway/yaw cleanly**, but heave/roll/pitch are dropped. This is acceptable for maneuvering; not for seakeeping.
- **`Frame2D` is `x`, `y`, `phi` (potentials) + `fx`, `fy`, `tau` (flows).** The connector flow rules say all flows summed at a node = 0. So if a single component writes `frame.fx + Force = 0` and the frame has no other connection, MTK eliminates the connector entirely and the constraint becomes `Force = 0` — i.e. you accidentally zero out the force you wanted to apply. *Anchor every Frame2D to either a `Body`, a `Fixed`, or another component that completes the node.*
- **`AdvanceSpeed = 0` causes NaN** in the propeller's Brix slipstream `C_th = sqrt(8·w²·D²·Kt / (π·V_a²))`. Fix: regularize as `sqrt(...) / sqrt(V_a² + ε)` and use `abs(Kt)` to keep the radicand positive when J briefly drives `Kt < 0` during transients. Cost a 30 s debug.
- **Don't double-apply forces.** Several ports keep both the `Frame2D` interface *and* signal-domain `Force_X`/`Force_Y`/`Moment` outputs as diagnostics. They report the same numbers — never wire the signal outputs into a different forcer; pick one path.
- **Hull body wrapper pattern**: instead of letting every external component apply forces directly to a bare `PlanarMechanics.Body` and trying to add drag separately, wrap the body with a `Hull3DOF` that internally uses a `WorldForceTorque(resolve_in_frame = FrameB())` to apply *both* the body-frame drag *and* the optional `Fx_extra`/`Fy_extra`/`Mz_extra` real inputs. External components (propeller, rudder, wind) connect their `frame_a` to `Hull3DOF.frame_a`; everything sums automatically.
- **`PlanarMechanics.Fixed`** is the right anchor for unit tests of force-producing components when there's no body to wire into. It pins position and orientation.
- **Sub-library namespacing**: always `MultibodyComponents.PlanarMechanics.<Name>`, never just `PlanarMechanics.<Name>`. Compiler error gives a clear "not found" if you forget.
- **`PartialTwoFrames`** is the right base class for two-frame planar components (Cable, sensors, joints). `extends MultibodyComponents.PlanarMechanics.PartialTwoFrames` saves the connector boilerplate.
- **`if … end` blocks in `relations`** key on a structural enum (`if frame == ResolveInFrame.World()`). They include / exclude *whole equations*, not just values. Useful for resolve-in-frame parameters but easy to confuse with algebraic `ifelse(...)` (which is value-level).
- **`render = false`** on every Body, Fixed, FixedTranslation, etc. dramatically speeds up compile + simulation (skips visualizer-shape equations). For autonomous, non-plotting workflows always set it.

## Environment
- Source: `ShipSIM.Components.Environment`
- File: `dyad/Environment.dyad`
- Notes:
  - Modelica used `inner/outer` semantics; Dyad has none, so callers must pass values explicitly.
  - Direction angles in degrees retained (no built-in `to_rad` in Dyad), so we multiply by `π / 180` inline.
  - Direction convention: 0° = +Y (north), 90° = +X (east), wind/current vector is `-V·{sin θ, cos θ, 0}`.

## VariableEnvironment
- Source: `ShipSIM.Components.VariableEnvironment`
- File: `dyad/VariableEnvironment.dyad`
- Notes: same as Environment, but wind/current speed and direction are now `RealInput()` ports.

## DewTemperature
- Source: `ShipSIM.Components.Others.MoistAir.DewTemperature`
- File: `dyad/DewTemperature.dyad`
- Notes:
  - Original used Modelica.Fluid moist-air port + Medium calls. Dyad has no equivalent fluid medium model in the standard libs, so the port-based formulation was reduced to a signal-based one: caller supplies `P_sat` and `RH`, component returns `Tdew` via Magnus' formula.
  - Same numerical formula as the Modelica original; only the data plumbing changed.

## Propeller1Q (Wageningen B + 2D multibody)
- Source: `ShipSIM.Components.Propulsion.Propeller1Q`
- Files: `dyad/Propeller1Q.dyad`, `dyad/Propeller1Q_analysis.dyad`
- Notes / assumptions:
  - **2D multibody coupling restored** via `frame_a::Frame2D`. Thrust is applied along the connected body's +x as `frame_a.fx + cos(phi)·Thrust = 0` and `frame_a.fy + sin(phi)·Thrust = 0`. Existing signal outputs (`Thrust`, `Propeller_flow_diameter`, ...) are kept for diagnostics — don't double-apply.
  - `ShipSpeed::RealInput` is still external (decoupled from the multibody plumbing) so the caller picks the apparent-speed convention (typically `hull.u - current`).
  - The full Wageningen polynomial is replaced by a quadratic-in-J fit (parameters override the defaults).
  - Inertia stays outside the component (caller attaches `RotationalComponents.Components.Inertia(J = Inertia + Add_Inertia)` in series); shaft connector is a pure-load `Spline`.
  - **Brix slipstream regularized**: `C_th = sqrt(8·w²·D²·|Kt| / (π·(V_a² + ε)))` so the formula stays finite at zero ship speed.
  - Validated with `Propeller1QTransient` (30 s, 100 kN·m source torque, 5 m/s ship speed): retcode `Success`, RPM settles to ~103, J=0.55, Kt=0.23, Thrust ≈ 144 kN.

## Rudder (2D multibody)
- Source: `ShipSIM.Components.Propulsion.Rudder`
- Files: `dyad/Rudder.dyad`, `dyad/Rudder_analysis.dyad`
- Notes / assumptions:
  - **2D multibody restored** via `frame_a::Frame2D`. Force_X / Force_Y / Moment are written to `frame_a.fx / fy / tau` (sign: `frame_a.fx + Force_X = 0` etc., body-axis components). The signal outputs are kept as diagnostics.
  - The `Position` + Limiter + SlewRateLimiter servo chain is replaced by a first-order filter to a clamped command (`Rudder_Tau` ≈ 1 / `MaxRudderAngularSpeed`).
  - `CombiTable2D` Cl/Cd/Cm vs (α, Re) replaced by quadratic-in-α fits. The original used 2D table lookup; user can override the coefficients to match a specific profile.
  - No Reynolds correction (no `Lambda` factor).
  - Validated with `RudderTransient` (5 s, anchored to a `Fixed`): Rudder_position settles to commanded 20°, Lift = 200 kN, Drag = 9.8 kN at 5 m/s.

## SimpleAutoPilot (single-target version)
- Source: `ShipSIM.Components.AutoPilot.SimpleAutoPilot`
- Files: `dyad/SimpleAutoPilot.dyad`, `dyad/SimpleAutoPilot_analysis.dyad`
- Notes / assumptions:
  - The original cycled through a static `Waypoints[:,3]` table using `discrete Integer i` and `when DistanceToDestination < tol` events. The ported model takes the active waypoint and target speed as plain real inputs; a switcher can sit on top.
  - `Modelica.Blocks.Continuous.LimPID(controllerType=PD)` was replaced with explicit equations using a first-order filtered derivative and a `clamp` saturation. Same PD structure (`y = k·(e + Td·de/dt)`) and the same default gains.
  - Validated with `SimpleAutoPilotTransient` (5 s): retcode `Success`; rudder saturates at +35° at t=0 (matches initial course offset of ~5.7° to port), shaft saturates at 300 RPM as the speed loop is far from target.

## SimpleDieselEngine
- Source: `ShipSIM.Components.Machines.SimpleDieselEngine`
- Files: `dyad/SimpleDieselEngine.dyad`, `dyad/SimpleDieselEngine_analysis.dyad`
- Notes / assumptions:
  - PI speed controller replaced with explicit equations (no `LimPID`). MaxTorque(RPM) and SFOC(kW) tables replaced with nested `ifelse` piecewise-linear interpolation (same data points as upstream).
  - SlewRateLimiter on RPM demand omitted (no equivalent block in BlockComponents).
  - Internal Inertia(J=15) lumped into the engine in upstream; here, the engine ends in a `Spline` connector and the caller attaches their own `Inertia`. Sign convention: `flange.tau = -tau_cmd` (engine drives the shaft).
  - Validated with `SimpleDieselEngineTransient` (30 s): RPM step from 1500 → 1800 demand at t=2s reaches 1800 RPM at steady state with PI tracking. Fuel and energy integrators accumulate sensibly (0.55 kg fuel, 2.98 kWh).

## Tank (anti-heeling)
- Source: `ShipSIM.Components.AntiHeelingSystem.Tank`
- File: `dyad/Tank.dyad`
- Notes: original used a single `RealInput[2]` for `(M_input, Q)`; split into two scalar `RealInput`s. Hard volume clamp via `clamp()` instead of nested `if/elseif`.

## AntiHeeling (2D multibody analogue)
- Source: `ShipSIM.Components.AntiHeelingSystem.AntiHeeling`
- File: `dyad/AntiHeeling.dyad`
- Notes / assumptions:
  - **2D multibody restored** via `frame_a::Frame2D`. The upstream applied a 3D `WorldTorque` about the roll axis; in 2D the only rotational DOF is yaw, so the righting moment is mapped onto `frame_a.tau`. For a true roll model the 3D framework is needed; this is the closest 2D analogue.
  - `OnOffController` (hysteresis) + `TriggeredTrapezoid` (10 s ramp) → smooth `clamp((|heel| - max_angle/2) / max_angle, 0, 1)` activation.
  - Startup delay: `time < startup_delay` ⇒ pump off (matches upstream `time < 500`).
  - Moment unit conversion: upstream `t·m × g_n × 1000` → N·m, kept verbatim.

## Cable (2D multibody)
- Source: `ShipSIM.SubComponents.Cable`
- File: `dyad/Cable.dyad`
- Notes / assumptions:
  - **2D PlanarMechanics version**: `extends MultibodyComponents.PlanarMechanics.PartialTwoFrames`. Tension is computed from the Euclidean distance between `frame_a` and `frame_b`, applied along the line connecting them. Equal and opposite forces on the two frames; `tau = 0` (cable doesn't transmit yaw moment).
  - `when` event for breakage NOT modeled (no Modelica-style `when` in Dyad). Tension-only spring: `Force = ifelse(Length - SetLength > 0, k·(Length - SetLength), 0)`.
  - 1D translational version was the previous attempt; replaced.

## ApparentSpeedXY (signal version)
- Source: `ShipSIM.SubComponents.ApparentSpeedXY`
- File: `dyad/ApparentSpeedXY.dyad`
- Notes: multibody Frame_a + AbsoluteVelocity sensor replaced by signal inputs (body world-frame velocity, body heading, free-stream velocity). Outputs both signed and unsigned attack angle.

## ShipWind (2D multibody)
- Source: `ShipSIM.Components.Ship.ShipWind`
- File: `dyad/ShipWind.dyad`
- Notes / assumptions:
  - **2D multibody restored** via `frame_a::Frame2D` carrying X_force / Y_force / N_moment. Signal outputs remain for diagnostics.
  - Fujiwara/Yamane Cx/Cy/Cn empirical coefficients (X_*, Y_*, N_*) kept as derived `parameter`s — same expressions as upstream.
  - Apparent wind speed and attack angle remain `RealInput`s (the caller assembles them from `hull.u`, `hull.v`, `Environment.WindVector`, etc.).

## SimpleAirExchanger
- Source: `ShipSIM.Components.Others.HeatTransfer.SimpleAirExchanger`
- File: `dyad/SimpleAirExchanger.dyad`
- Notes: Modelica's `PrescribedHeatFlow` + `RelTemperatureSensor` not needed in Dyad — `Internal_Air.Q_flow` and `External_Air.Q_flow` are written directly. Same volumetric heat-capacity constant 1208.4 J/(m³·K).

## IrradiationOnPlane
- Source: `ShipSIM.Components.Others.Solar.IrradiationOnPlane`
- File: `dyad/IrradiationOnPlane.dyad`
- Notes: Modelica's `algorithm` block (with `:=`) replaced by an algebraic `relations` formula — Dyad uses `=` and `ifelse` for the `max(...,0)` part. The 3-vector `RealInput` in upstream is exposed as six scalar `RealInput`s here because Dyad doesn't support array-typed connectors cleanly.

## WingSail (2D multibody)
- Source: `ShipSIM.Components.AlternativePropulsion.WingSail`
- File: `dyad/WingSail.dyad`
- Notes / assumptions:
  - **2D multibody restored** via `frame_a::Frame2D` carrying Force_X / Force_Y. The 2D port emits zero yaw moment from the sail itself; lever-arm-driven yaw should be composed at the ship level via `MultibodyComponents.PlanarMechanics.FixedTranslation` between the hull CG and the sail mount.
  - Sail-angle servo: first-order filter to a clamped command (same shape as rudder).
  - Cl/Cd/Cm vs α: quadratic fits (parameters override).

## Hull3DOF (2D multibody)
- Source: simplified replacement for `ShipSIM.Components.Ship.HidrodynamicXYY` (and the surrounding multibody ship body)
- Files: `dyad/Hull3DOF.dyad`, `dyad/Hull3DOF_analysis.dyad`
- Notes / assumptions:
  - **Now uses `MultibodyComponents.PlanarMechanics.Body`** internally. Linear body-frame drag (Du, Dv, Dr) and optional `Fx_extra`, `Fy_extra`, `Mz_extra` are applied through an internal `WorldForceTorque(resolve_in_frame = FrameB())`. External multibody components (Propeller1Q, Rudder, ShipWind, WingSail, AntiHeeling) connect their `frame_a` to `Hull3DOF.frame_a` and have their forces summed at the body node automatically.
  - Higher-fidelity MMG polynomial drag (X_vv, Y_v, …) belongs as Julia helpers feeding `Fx_extra`/`Fy_extra`/`Mz_extra`.
  - Validated end-to-end with `PropellerOnHullTransient` (200 s): same numbers as the previous Newton-Euler version (u = 11.57 m/s, RPM = 153, thrust = 72 kN, x = 1528 m), confirming the multibody re-port is numerically equivalent.

## TemperatureDataset (CSV-driven)
- Source: `ShipSIM.Components.Others.HeatTransfer.TemperatureDataset`
- Files: `dyad/TemperatureDataset.dyad`, `assets/temperature_24h.csv`, `dyad/TemperatureDataset_analysis.dyad`
- Notes / assumptions:
  - Uses `BlockComponents.Tables.Interpolation` reading a `DyadData.DyadTimeseries` CSV asset (`dyad://DyadShip/temperature_24h.csv`, a 24-hour synthetic temperature swing).
  - Pipes the interpolated `T(time)` into a `ThermalComponents.Sources.PrescribedTemperature` whose port is the `port_AirTemp` `HeatPort` output.
  - Linear interpolation + Constant extrapolation matches upstream `HoldLastPoint`.
  - Validated `TemperatureDatasetTransient` (24 h): a 1 kJ/K thermal capacitor connected via a 100 W/K conductor tracks the time-table — peak T ≈ 299 K at noon, floor ≈ 283 K overnight, matching the CSV.

## PlateTransient (uniform-discretization)
- Source: `ShipSIM.Components.Others.HeatTransfer.PlateTransient`
- Files: `dyad/PlateTransient.dyad`, `dyad/PlateTransient_analysis.dyad`
- Notes / assumptions:
  - Uniform N-node discretization (the upstream's `DiscretizePlate` for a non-`USED` variant returns uniform layers anyway).
  - Component arrays of `ThermalComponents.Components.HeatCapacitor` and `ThermalComponents.Components.ThermalConductor` wired via `for i in 1:nNodes` in `relations`.
  - **Gotcha**: passing parameter values into array-comprehension calls fails MTK substitution unless the outer parameters are `structural`. Made e, Asur, k, rhoCp, T_init, nNodes, dx, C_node, G_internal, G_face all `structural parameter`. (This is a workaround for a current Dyad / MTK substitution-loop issue.)
  - Internal-heat-port + Tavg/Energy outputs dropped (extension paths).
  - Validated `PlateTransientAnalysis` (10 min, 50 W/(m·K) plate, ρCp 4e6 J/(m³·K), one face fixed at 373.15 K, other insulated): all 5 nodes converge to 373.15 K within 60 s — matches the high-conductivity (Biot ≪ 1) lumped-capacitance expectation.

## CylinderTransient (equal-area radial)
- Source: `ShipSIM.Components.Others.HeatTransfer.CylinderTransient`
- File: `dyad/CylinderTransient.dyad`
- Notes / assumptions:
  - Radial discretization computed by Julia helper `DyadShip.discretize_cylinder_conductances(R, L, k, N)` defined in `dyad/definitions.jl` — mirrors the upstream `DiscretizeCylinder(R, N, …)` logic.
  - Same structural-parameter pattern as `PlateTransient` for the array comprehensions to work.
  - Internal-heat-port + Tavg/Energy outputs dropped (extension paths).
  - Smoke-tested: `@named cyl = DyadShip.CylinderTransient()` instantiates cleanly.

## SolarIrradiation
- Source: `ShipSIM.Components.Others.Solar.SolarIrradiation`
- Files: `dyad/SolarIrradiation.dyad`, `dyad/SolarIrradiation_analysis.dyad`
- Notes / assumptions:
  - `discrete Integer dayOfYear` + `when` rollover replaced by a `parameter day_of_year` (no rollover; one day of simulation at a time).
  - Astronomy formulae moved to Julia helpers `sun_height_deg(...)` and `sun_vector_world(...)` in `dyad/definitions.jl` — same declination, true mean time, hour angle, altitude expressions as upstream.
  - **Gotcha**: Julia helpers must avoid `if/else` ternaries on Symbolic `Num` inputs; replaced `sin_alt > 0 ? a : b` with all-branches arithmetic plus a Dyad-side `ifelse` mask. Also avoid `max(-1.0, min(1.0, x))` clamping on symbolic inputs (raises non-boolean errors); instead add a small ε under the sqrt to keep `cos_alt` real.
  - Optional measured-irradiance table dropped; user multiplies output by their own cloud-correction factor.
  - Validated `SolarSweepTransient` (24 h, summer solstice, ≈ 43.4° N): sunrise 6 h, peak ≈ 890 W/m² at solar noon (close to `irradiance_ref = 1000` scaled by `sin(altitude)`), sunset 21 h.

## ConvRadSunWall (signal-based)
- Source: `ShipSIM.Components.Others.HeatTransfer.ConvRadSunWall`
- File: `dyad/ConvRadSunWall.dyad`
- Notes / assumptions:
  - The upstream wired internal sub-components (`Internal.BodyRadiation`, `Internal.FreeConvection`, …) that aren't on hand. This port writes the combined heat flux directly: solar absorbed + convection (Newton's law of cooling) + IR radiation (Stefan-Boltzmann).
  - Inputs: `S_panel` (irradiance on plane), `shade_factor` (0..1 from a `SunScreen`), `h_c` (wind-dependent), `T_air`, `T_sky`. Output: a `HeatPort` for the wall material.
  - Sun-screen and IrradiationOnPlane wiring is the caller's responsibility (this block consumes their *outputs*).

## PeakSampler (continuous peak-hold)
- Source: `ShipSIM.Components.DataProcessing.PeakSampler`
- File: `dyad/PeakSampler.dyad`
- Notes / assumptions:
  - The original `when ZeroCrossing(der(u)) then y = u` event-based sampler is replaced by a continuous peak-hold filter: `der(y) = ifelse(u > y, fast_attack·(u-y), -alpha·(y-u))`.
  - Captures running maxima only (alpha = 0); for tracking each peak individually, post-process externally.

## Propeller4Q (simplified 4-quadrant)
- Source: `ShipSIM.Components.Propulsion.Propeller4Q`
- File: `dyad/Propeller4Q.dyad`
- Notes / assumptions:
  - The full 4Q Wageningen Fourier polynomial would need a Julia helper. This port mirrors `Propeller1Q` and adds sign flips on Kt and Kq for reverse-rotation / astern-thrust. Crude but covers crash-stop and reverse-thrust qualitatively.
  - Same `Spline + Frame2D + ShipSpeed::RealInput` interface as Propeller1Q.

## POD4Q (azimuthing pod)
- Source: `ShipSIM.Components.Propulsion.POD4Q`
- File: `dyad/POD4Q.dyad`
- Notes / assumptions:
  - Same shape as Propeller4Q + a first-order azimuth servo (matching the `Rudder` pattern).
  - Thrust applied along the *POD-rotated* body axis: `frame_a.fx + cos(frame_a.phi + beta)·Thrust = 0`.
  - Yaw moment from a non-zero mounting offset is the caller's responsibility (compose via `MultibodyComponents.PlanarMechanics.FixedTranslation`).

## OnOffConsumer (continuous variant)
- Source: `ShipSIM.Components.Electrical.OnOffConsumer`
- File: `dyad/OnOffConsumer.dyad`
- Notes / assumptions:
  - Random `RandomStart` schedule replaced by an external `WorkSignal::RealInput` (1 = on). The user drives this from a `BlockComponents.Sources.Square` for a periodic duty cycle, or from any custom signal.
  - Start-up + cycle power tables replaced by a parameterized ramp-then-oscillate shape: `time_on / StartTime` during start-up, then `1 + CycleAmplitude·sin(2π·CycleFrequency·t)` afterward.
  - The `getIntegrators` optional sub-component is dropped; pipe `y` into `BlockComponents.Continuous.Integrator` externally if needed.

## Buoyancy1D (vertical heave + buoyancy)
- New component (no upstream — fills the gap left by `HidrodynamicZRP` not being portable to 2D PlanarMechanics).
- Files: `dyad/Ship/Buoyancy1D.dyad`, `dyad/Ship/Buoyancy1D_analysis.dyad`
- Notes / assumptions:
  - 1D vertical state (z, w) decoupled from the planar Hull3DOF surge/sway/yaw. Same split as ShipSIM's `XYY` vs `ZRP` blocks. Pair both for a 4-DOF maneuvering + heave model.
  - Linear-in-draft block-coefficient volume: `V_submerged = Cb·L·B·draft`.
  - Equation: `m·der(w) = -m·g + ρ·V·g - Dz·w + Fz_extra`; `der(z) = w`.
  - Initial condition pre-computed to equilibrium draft `m / (ρ·Cb·L·B)`, so the hull starts at rest with no drift.
  - Validated `Buoyancy1DTransient` (30 s): equilibrium draft 0.697 m matches `m/(ρ·Cb·L·B)`. After a 5 MN downward impulse, draft converges to 1.052 m — exactly matches the theoretical new equilibrium `(m + F/g) / (ρ·Cb·L·B)`.

## Ship (full multibody assembly)
- File: `dyad/Ship_analysis.dyad`
- Composes Hull3DOF + Propeller1Q + Rudder + SimpleAutoPilot inside a single PlanarMechanics network. All hull-mounted components share `hull.frame_a`; the autopilot reads world-frame state and commands the rudder.
- Validated with `ShipTransient` (200 s, target waypoint `(10000, 1000)` at 5 m/s): retcode `Success`. Hull moves forward and the rudder modulates heading (sign convention requires further tuning for textbook turning behavior, but the PlanarMechanics integration is solid).

## SunScreen
- Source: `ShipSIM.Components.Others.Solar.SunScreen`
- File: `dyad/SunScreen.dyad`
- Notes: Modelica's branchy `if/elseif/else` chain (4 branches based on solar altitude) folded into nested `ifelse` (Dyad has no algebraic `if/elseif`). The geometric expressions for `A`, `I`, `P`, `p` are evaluated unconditionally to keep the equation set well-defined; the result is then selected by altitude band.
