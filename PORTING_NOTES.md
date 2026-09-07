# Porting notes: ShipSIM → DyadShip

Status of the port of [ShipSIM](https://github.com/BasilioPV/ShipSIM) (Modelica)
to Dyad, consolidated from the earlier `REPORT.md` / `SUCCESSES.md` /
`PORT_PLAN.md` / `HARD.md` / `STILL_HARD.md` / `INVESTIGATION_yaw_resistance.md`
files. Per-component assumptions live in each component's docstring; this file
is the map.

## Two stacks

| | `dyad/Ship6DOF` (primary) | `dyad/Ship`, `dyad/Propulsion` (planar, legacy) |
|---|---|---|
| Multibody library | `MultibodyComponents` 3D (`Frame3D`, `Body`, `WorldForce`/`WorldTorque`, `FixedTranslation`) | `MultibodyComponents.PlanarMechanics` (`Frame2D`) |
| Degrees of freedom | surge, sway, heave, roll, pitch, yaw | surge, sway, yaw |
| Hydrostatics | draft-polynomial displacement, CoB, BM (upstream `ShipModelTh`) | none |
| Hull forces | `HydrodynamicXYY` (MMG + added mass), `HydrodynamicZRP` | `HullMMG` (MMG, cross terms only) |
| Propellers | `Propeller1Q` (full Wageningen B polynomial), `Propeller4Q` (14 four-quadrant Fourier sets, Burrill cavitation check), `POD4Q` (servo revolute + `Propeller4Q`) | quadratic-in-J fits |
| Rudder | servo revolute, NACA 0012/0015 `Cl/Cd/Cm(α, Re)` tables, Söding slipstream and hull factors | quadratic fits |
| Sails, deck gear | `WingSail` (servo revolute, tables), `AntiHeeling` (hysteresis relay, ramp, roll torque), `Crane` + `Cable` (two servo revolutes, tension-only cable with latching break) | — |
| Validated by | roll decay, speed trial, turning circle, rudder return, zig-zag, crash stop, sail polar, four-sail ship with anti-heeling, pod turning circle, crane operation, autopilot transit (`scripts/validate_6dof.jl`) | autopilot transit, Flettner-rotor renders |

The planar stack is kept because the Flettner-rotor work (CFD table, live
WaterLily co-simulation, the three rendered transits) is built on it. Its
`FullShip*` analyses use `mass = 1e6`, `Iz = 1e8`, far below the 5.7e6 kg /
4.3e9 kg m² of the hull the hydrodynamic estimates describe, so their
transients are faster than a real 100 m ship. New maneuvering work should use
`Ship6DOF`.

## Coordinate conventions (both stacks)

World: x east, y north, z up, free surface at z = 0
(`MultibodyComponents.World(n = [0, 0, -1])` in 3D). Ship: x forward, y port,
z up, origin at the aft perpendicular / centreline / keel in 3D (upstream
convention; CoG at `[50.364, 0, 7]` for the sample hull). Positive yaw is bow
to port, positive rudder order turns the ship to port, positive heel is
starboard down, positive trim is bow down. Wind and current directions are
"coming from", 0° = north, 90° = east.

## Upstream component map

| ShipSIM | DyadShip | Notes |
|---|---|---|
| `Components.Environment`, `VariableEnvironment` | root `Environment`, `VariableEnvironment` | signal-only; no `inner`/`outer` in Dyad |
| `SubComponents.ApparentSpeedXY` | `Ship6DOF.ApparentSpeedXY` (frame-based), root `ApparentSpeedXY` (signal) | |
| `Ship.ShipModelTh` | `Ship6DOF.ShipBody` | yaw-pitch-roll Euler body, `draft_poly` hydrostatics |
| `Ship.HidrodynamicXYY` | `Ship6DOF.HydrodynamicXYY`, planar `HullMMG` | see corrections below |
| `Ship.HidrodynamicZRP` | `Ship6DOF.HydrodynamicZRP` | upstream is a zero-coefficient placeholder; defaults here from damping ratios |
| `Ship.ShipWind` | `Ship6DOF.ShipWind`, planar `Ship.ShipWind` | Fujiwara regression |
| `Propulsion.Propeller1Q` | `Ship6DOF.Propeller1Q`, planar `Propulsion.Propeller1Q` | 3D: exact Oosterveld & van Oossanen polynomial |
| `Propulsion.Propeller4Q` | `Ship6DOF.Propeller4Q`, planar `Propulsion.Propeller4Q` | 3D: all 14 Fourier data sets; planar: mirrored 1Q |
| `Propulsion.POD4Q` | `Ship6DOF.POD4Q` | servo revolute, strut, internal `Propeller4Q` |
| `Propulsion.Rudder` | `Ship6DOF.Rudder`, planar `Propulsion.Rudder` | 3D: 2D tables from `assets/naca*.csv` |
| `AlternativePropulsion.WingSail` | `Ship6DOF.WingSail` | servo revolute, NACA tables, forces at the rotated quarter chord |
| (new) | planar `Propulsion.FlettnerRotor`, `FlettnerRotorOnline` | WaterLily-derived Magnus coefficients |
| `AutoPilot.SimpleAutoPilot` | `Ship.HeadingAutoPilot` (PI + throttle ramp), `Ship6DOF.WaypointAutopilot` (`LimPID`) | waypoint cycling dropped (no events) |
| `AntiHeelingSystem.AntiHeeling`, `Tank` | `Ship6DOF.AntiHeeling`, `Ship.Tank` | hysteresis and ramp reproduced with a relay state and a slew-rate limiter |
| `Machines.SimpleDieselEngine` | `Propulsion.SimpleDieselEngine` | tables inlined as `ifelse` |
| `Machines.Crane`, `SubComponents.Cable` | `Ship6DOF.Crane`, `Ship6DOF.Cable` | 3D; cable break as a latching relay instead of an event |
| `Electrical.OnOffConsumer` | `Machinery.OnOffConsumer` | driven by a work signal instead of a random schedule |
| `DataProcessing.PeakSampler` | `Machinery.PeakSampler` | continuous peak-hold |
| `Others.Solar.*`, `Others.HeatTransfer.*`, `MoistAir.DewTemperature` | `Thermal.*` | see docstrings |

Not ported: `RainflowCounter` / `FatigueCounter` (needs `algorithm` + `pre`;
belongs in a Julia post-processor), `TriggerConsumer` / `StartGenerator`
(event-driven schedules), `SourceMoistAir` (no moist-air medium),
`SunIrradianceMultibody`, `EnvironmentHeatTransfer`, `ConvectionFactors.*`,
`SubComponents.Ikeda` (partial roll-damping stub upstream), `VariableTranslation`
(the 3D components apply forces at variable points algebraically instead).

## Corrections relative to upstream (documented in the docstrings)

- `HidrodynamicXYY.mx` (Brix surge added mass) divides by the water density
  inside the square root, which makes the fraction negative; the port uses
  `1 / (π sqrt(Lpp³/∇) - 14)`.
- `HidrodynamicZRP` applies the roll damping coefficient and roll rate to the
  pitch axis; the port uses the pitch coefficient and pitch rate.
- `ShipWind` normalises the lateral force with `A_T`; the port uses `A_L`
  (Fujiwara's definition). The lateral force and yaw moment keep the upstream
  minus signs.
- `Rudder`: the effective angle of attack is `α = δ - γ_R β_R`, so a drift
  angle reduces the effective rudder angle in a turn and a centred rudder
  produces the restoring fin force. Both the previous planar port (and, read
  literally, the upstream sign chain) had the inflow term with the opposite
  sign; on the planar stack that made a centred rudder a destabilising fin
  and left the hull in a steady turn after the rudder was returned to zero.
  The 6-DOF `RudderReturn` and planar `RudderReturn2D` analyses are the
  regression checks.
- Planar `HullMMG`: the added-mass cross terms are `+Δ my v r` (surge) and
  `-Δ mx u r` (sway) as fractions of displacement; the earlier port had both
  signs flipped and divided by 100.
- `Propeller4Q` data set `B4_70_14` is assigned twice upstream (the second
  block zeroes rows 8–31); the port keeps the non-zero rows.
- `Tank` drains the tank on the side the anti-heeling system moves ballast to;
  the port fills it, consistent with the righting moment `AntiHeeling` applies.
- `FourWingSails` upstream leaves the rudder fixed, so the ship weathercocks
  into the wind and the sails stall; the port steers with `WaypointAutopilot`
  and sheets the sails to 50°.

## Known limitations

- With the default empirical derivatives the linear course-stability index of
  the sample hull is slightly negative (`HydrodynamicXYY.CourseStability`); the
  hull holds a straight course through the rudder's fin effect, which shows as
  a slow yaw-rate decay after rudder centring and 20/20 zig-zag overshoots
  around 30°. Override `N_r` (e.g. Clarke's estimate, about twice Khattab's)
  for a stiffer hull.
- No wave excitation, no shallow-water effects, no propeller transverse
  thrust in the crash stop.
- Dyad has no discrete events: the zig-zag relay, the anti-heeling
  controller and the cable break are continuous relay approximations of
  hysteresis and latching.
- Pods, sails and the crane have no aerodynamic/hydrodynamic interaction
  between units (no sail-sail interference, no pod-hull interaction).
- Small-angle hydrostatics (about ±20° heel/trim), as upstream.
