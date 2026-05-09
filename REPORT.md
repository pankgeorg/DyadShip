# DyadShip — port report

End-of-third-pass report. Pass 1: 18 signal-only ports because `MultibodyComponents`
seemed unavailable. Pass 2: discovered `MultibodyComponents.PlanarMechanics`, re-ported
multibody-using components with 2D coupling restored, and tackled every `HARD.md`
entry. Pass 3: code review (caught body→world frame rotation bugs in `Rudder`,
`ShipWind`, `WingSail`), reorganized into 4 submodules + foundational helpers at the
root, added 5 more analyses.

## Repo layout

```
dyad/
├── Environment.dyad        # foundational
├── VariableEnvironment.dyad
├── ApparentSpeedXY.dyad
├── hello.dyad              # legacy starter
├── Ship/                   # hull body + ship-level forces / control
│   ├── Hull3DOF.dyad           + Hull3DOF_analysis.dyad
│   ├── ShipWind.dyad
│   ├── AntiHeeling.dyad        + AntiHeeling_analysis.dyad
│   ├── Tank.dyad
│   ├── SimpleAutoPilot.dyad    + SimpleAutoPilot_analysis.dyad
│   └── Ship_analysis.dyad      (full multibody assembly)
├── Propulsion/             # engine + propellers + rudder + sail
│   ├── Propeller1Q.dyad        + Propeller1Q_analysis.dyad
│   ├── Propeller4Q.dyad
│   ├── POD4Q.dyad
│   ├── Rudder.dyad             + Rudder_analysis.dyad
│   ├── WingSail.dyad           + WingSail_analysis.dyad
│   └── SimpleDieselEngine.dyad + SimpleDieselEngine_analysis.dyad
├── Machinery/              # deck / auxiliary equipment
│   ├── Crane.dyad              + Crane_analysis.dyad (in STILL_HARD)
│   ├── Cable.dyad
│   ├── OnOffConsumer.dyad      + OnOffConsumer_analysis.dyad
│   └── PeakSampler.dyad        + PeakSampler_analysis.dyad
└── Thermal/                # heat transfer + solar + moist air
    ├── PlateTransient.dyad     + PlateTransient_analysis.dyad
    ├── CylinderTransient.dyad  + CylinderTransient_analysis.dyad
    ├── SimpleAirExchanger.dyad
    ├── ConvRadSunWall.dyad
    ├── TemperatureDataset.dyad + TemperatureDataset_analysis.dyad
    ├── DewTemperature.dyad
    ├── SolarIrradiation.dyad   + SolarIrradiation_analysis.dyad
    ├── IrradiationOnPlane.dyad
    ├── SunScreen.dyad
    └── definitions.jl       # Julia helpers (sun_*, discretize_cylinder_*)

assets/
└── temperature_24h.csv     # 24-h synthetic temperature swing
```

27 components, 15 analyses (all retcode `Success`):

- `World` (top-level legacy starter)
- `Ship.SimpleAutoPilotTransient`, `Ship.AntiHeelingTransient`,
  `Ship.PropellerOnHullTransient`, `Ship.ShipTransient`
- `Propulsion.Propeller1QTransient`, `Propulsion.RudderTransient`,
  `Propulsion.SimpleDieselEngineTransient`, `Propulsion.WingSailTransient`
- `Machinery.PeakSamplerTransient`, `Machinery.OnOffConsumerTransient`
- `Thermal.TemperatureDatasetTransient`, `Thermal.PlateTransientAnalysis`,
  `Thermal.CylinderTransientAnalysis`, `Thermal.SolarSweepTransient`
- `SUCCESSES.md` — per-component notes including assumptions and validation
- `HARD.md` — empty
- `STILL_HARD.md` — components that resisted multiple attempts (RainflowCounter,
  Crane analysis, TriggerConsumer/StartGenerator, VariableTranslation, Ikeda,
  HidrodynamicZRP/ShipModelTh, SourceMoistAir, strict event-PeakSampler) with
  reasons and likely fixes
- `PORT_PLAN.md` — original tier-based roadmap
- `AGENTS.md` — task brief + accumulated learnings (general + multibody-second-pass)
- `dyad.sh` — now exports the four `JULIAUP_*` env vars internally so it runs cleanly

## Headline numbers

- 27 components ported (up from 18 in the first pass).
- All compile cleanly under `dyad-3.0.0-rc5`.
- **15 transient analyses** successful — World, SimpleAutoPilot, AntiHeeling,
  PropellerOnHull, full Ship; Propeller1Q, Rudder, SimpleDieselEngine, WingSail;
  PeakSampler, OnOffConsumer; TemperatureDataset, PlateTransient, CylinderTransient,
  SolarSweep.
- All HARD.md entries either ported (8 of them) or moved to STILL_HARD.md (8 of
  them). HARD.md is empty.
- Components organized into 4 submodules (Ship, Propulsion, Machinery, Thermal) plus
  foundational helpers at the root.

## Code-review fixes (third pass)

- **Body→world frame rotation bug** in `Rudder`, `ShipWind`, `WingSail`: each was
  computing a body-frame `Force_X`/`Force_Y` and writing it directly to
  `frame_a.fx`/`fy`, but PlanarMechanics' `Body` interprets `frame_a.fx`/`fy` as
  *world-frame* forces (`m·der(world_v) = f`). Fixed by rotating through
  `frame_a.phi`. `Propeller1Q` already had the correct rotation (`cos(frame_a.phi) *
  Thrust`). Caught while reviewing the heading drift in `Ship_analysis`.
- **`Body`-less analyses don't need a `World`.** Tests with only `Fixed` + `frame_a`
  (no `Body`) emit MTK errors (`Expected an Initial parameter to exist for variable
  g`) when a `World` is included. Removed `World` from `AntiHeeling_analysis` and
  `WingSail_analysis`.
- **Removed dead code**: `comp_rate` in `AntiHeeling`, `w_init` in
  `SimpleDieselEngine`. Updated stale docstrings ("removes the multibody coupling"
  in `AntiHeeling` was no longer true).

## Second-pass highlights

- **MultibodyComponents.PlanarMechanics integration.** Found at
  `~/.julia/packages/MultibodyComponents/<hash>/dyad/PlanarMechanics/` after
  installing. 2D only (Frame2D), but ship maneuvering is 2D so this maps cleanly.
  Re-ported `Propeller1Q`, `Rudder`, `ShipWind`, `WingSail`, `AntiHeeling`, `Cable`,
  and `Hull3DOF` to use Frame2D coupling instead of the previous signal-only
  workaround. Built a full closed-loop ship model (`ShipTransient`) that combines
  hull + propeller + rudder + autopilot inside one PlanarMechanics network.
- **Asset-driven temperature time-series.** `TemperatureDataset` now reads
  `dyad://DyadShip/temperature_24h.csv` from the `assets/` folder via
  `DyadData.DyadTimeseries` + `BlockComponents.Tables.Interpolation`.
- **Discretized heat transfer.** `PlateTransient` and `CylinderTransient` use Dyad
  component arrays with structural parameters to dodge MTK's parameter-substitution
  issues. The cylinder uses a Julia helper `discretize_cylinder_conductances`
  delivered through `dyad/definitions.jl`.
- **Solar position via Julia helpers.** `SolarIrradiation` outputs sun height and a
  3D sun-vector at any simulation time via `sun_height_deg` / `sun_vector_world`
  helpers.
- **Continuous-time approximations of discrete blocks.** `PeakSampler` (continuous
  peak-hold) and `OnOffConsumer` (RealInput-driven instead of randomly scheduled)
  preserve the structural role without `when` events.
- **Dyad CLI ergonomics.** `dyad.sh` now self-sets the JuliaUp env, so `../dyad.sh
  compile` runs cleanly without any caller env wrangling.

## Patterns that proved out

1. **Multibody → PlanarMechanics.** Replace 3D Frame_a/Frame_b with Frame2D, use
   `WorldForceTorque(resolve_in_frame = FrameB)` to apply body-frame forces, and
   wrap bodies in a Hull3DOF-style helper that sums all force inputs at one node.
2. **CombiTable → polynomial / `ifelse`-piecewise.** Inline small tables (< 10
   points) as nested `ifelse`. For ≥ 50-row datasets, ship a CSV in `assets/` and
   read via `BlockComponents.Tables.Interpolation`.
3. **`when` events → smooth saturating activations / continuous state.** For
   threshold-on/off, use `clamp((|x| - thresh) / band, 0, 1)`. For peak-tracking,
   use a fast-attack continuous max-hold. For random schedules, expose the on/off
   as a `RealInput` so the user supplies their own.
4. **Algebraic `if/elseif` → nested `ifelse`.** Sort branches by ascending threshold.
5. **Use Julia helpers for math-heavy logic** (`sun_*`, `discretize_*`).
   `dyad/definitions.jl` is auto-included by the generated module. Type helper
   signatures untyped (or `Real`) so MTK Symbolic propagation works. Avoid Julia
   ternaries on Symbolic inputs.
6. **Component-array comprehensions need structural parameters.** Anything that
   flows into `[Component(... = outer_param) for i in 1:N]` must be `structural` or
   it'll hit `Could not evaluate value of parameter`-style substitution failures.

## What I'd do next, given more budget

1. **Resolve the Crane algebraic loop** by replacing `Cable` with a `PlanarMechanics.SpringDamper`
   or by adding small body-frame damping. Skeleton already compiles.
2. **A 4Q Wageningen Julia helper** that returns `(Kt, Kq)` as a function of `(beta,
   P_D, Ae_Ao, Z)`, then drop the quadratic-in-J approximation in `Propeller1Q`,
   `Propeller4Q`, and `POD4Q`.
3. **MMG polynomial drag** as another Julia helper feeding `Hull3DOF.Fx_extra`,
   `Fy_extra`, `Mz_extra` — that gets us to a full `HidrodynamicXYY` port.
4. **`Internal.BodyRadiation`, `Internal.FreeConvection`, `Internal.HeatSplitter`** —
   port these `Components.Others.HeatTransfer.Internal.*` building blocks so the full
   `EnvironmentHeatTransfer` and the `useInternalHeatPort` variants of
   `PlateTransient` / `CylinderTransient` can be assembled.
5. **A Julia post-processor for the `RainflowCounter`/`FatigueCounter`** consuming
   the stress time-series from a `TransientAnalysis` result.
6. **3D Multibody library** — when one ships, port `HidrodynamicZRP`, `ShipModelTh`,
   `Crane` (full), `Ikeda`, and `VariableTranslation`.

## How to reproduce / run

From `DyadShip/`:

```sh
# Compile all .dyad files (idempotent; check `generated/*.jl` mtimes for success):
../dyad.sh compile

# Run an analysis from Julia. Analyses live under their submodule's namespace:
../julia-dyad.sh -e 'using DyadShip; using DyadInterface; \
  r = DyadShip.Ship.ShipTransient(); println(r.sol.retcode)'
```
