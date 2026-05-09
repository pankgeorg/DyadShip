# DyadShip Port Plan

Goal: rewrite ShipSIM (Modelica) → DyadShip (Dyad), one component per file, simple →
complex. Status reflects three passes (signal-only → multibody → submodule reorg +
buoyancy). See `SUCCESSES.md` for per-component notes, `STILL_HARD.md` for what
resisted, and `REPORT.md` for the executive summary.

Legend: ✅ done · ⚠ done with simplifications (see SUCCESSES.md) · ⏭ in STILL_HARD ·
🆕 new component, no upstream.

## Inventory (from ShipSIM/ShipSIM/*.mo)

### Tier 1 — Trivial parameter / signal models
- ✅ `Components.Environment` → `Environment.dyad`
- ✅ `Components.VariableEnvironment` → `VariableEnvironment.dyad`
- ✅ `SubComponents.ApparentSpeedXY` → `ApparentSpeedXY.dyad` (signal version of the
  multibody original)
- ✅ `Components.AutoPilot.SimpleAutoPilot` → `Ship/SimpleAutoPilot.dyad` (single-target
  variant; waypoint cycling dropped — needs `when` events)
- ⚠ `Components.Machines.SimpleDieselEngine` → `Propulsion/SimpleDieselEngine.dyad`
  (CombiTable lookups inlined as `ifelse`-piecewise, no SlewRateLimiter, external Inertia)
- ⚠ `Components.Electrical.OnOffConsumer` → `Machinery/OnOffConsumer.dyad`
  (`WorkSignal::RealInput` replaces the random scheduler)
- ⏭ `Components.Electrical.TriggerConsumer` — random schedule + 5-step trigger
- ⚠ `Components.DataProcessing.PeakSampler` → `Machinery/PeakSampler.dyad`
  (continuous peak-hold; strict-event version still hard)
- ⏭ `Components.Others.MoistAir.SourceMoistAir` — needs Modelica.Fluid moist-air medium
- ✅ `Components.Others.MoistAir.DewTemperature` → `Thermal/DewTemperature.dyad`
- ✅ `Components.Others.HeatTransfer.TemperatureDataset` →
  `Thermal/TemperatureDataset.dyad` + `assets/temperature_24h.csv`
- ✅ `SubComponents.Cable` → `Machinery/Cable.dyad` (2D PlanarMechanics; break event
  dropped)

### Tier 2 — Domain physics
- ⚠ `Components.Ship.ShipWind` → `Ship/ShipWind.dyad` (Fujiwara/Yamane coefficients
  preserved; multibody applied via Frame2D)
- ⚠ `Components.Propulsion.Propeller1Q` → `Propulsion/Propeller1Q.dyad`
  (quadratic-in-J Kt/Kq, full Wageningen polynomial pending; Frame2D + Spline)
- ⚠ `Components.Propulsion.Rudder` → `Propulsion/Rudder.dyad` (quadratic Cl/Cd/Cm,
  first-order servo, no Reynolds correction; Frame2D)
- ✅ `Components.AntiHeelingSystem.Tank` → `Ship/Tank.dyad`
- ⚠ `Components.AntiHeelingSystem.AntiHeeling` → `Ship/AntiHeeling.dyad` (smooth
  saturation replaces hysteresis on/off + triggered trapezoid; roll moment maps onto 2D
  yaw)

### Tier 3 — Multibody-heavy / heavy data fits
- ⏭ `Components.Ship.HidrodynamicXYY` — MMG polynomial drag. Replaced by `Hull3DOF` + a
  future Julia helper feeding `Fx_extra/Fy_extra/Mz_extra`.
- ⏭ `Components.Ship.HidrodynamicZRP` — heave/roll/pitch. Replaced by `Buoyancy1D` for
  heave only; full ZRP needs 3D multibody.
- ⏭ `Components.Ship.ShipModelTh` — top-level multibody composition. Built bespoke as
  `Ship/Ship_analysis.dyad` (full PlanarMechanics assembly: hull + propeller + rudder
  + autopilot).
- ⚠ `Components.Propulsion.Propeller4Q` → `Propulsion/Propeller4Q.dyad` (sign-flipped
  1Q for crash-stop; full Wageningen 4Q polynomial pending)
- ⚠ `Components.Propulsion.POD4Q` → `Propulsion/POD4Q.dyad` (Propeller4Q + first-order
  azimuth servo)
- ⚠ `Components.AlternativePropulsion.WingSail` → `Propulsion/WingSail.dyad`
  (quadratic Cl/Cd/Cm, first-order sail-angle servo, no lever-arm yaw)
- ⚠ `Components.Machines.Crane` → `Machinery/Crane.dyad` (compiles; closed-loop
  analysis blocked by tension-only-cable algebraic loop — see STILL_HARD)
- ⏭ `Components.Others.EnvironmentHeatTransfer` — large composed weather model
- ✅ `Components.Others.Solar.SolarIrradiation` → `Thermal/SolarIrradiation.dyad`
  (astronomy in `Thermal/definitions.jl`; date rollover + measured-irradiance table dropped)
- ✅ `Components.Others.Solar.IrradiationOnPlane` → `Thermal/IrradiationOnPlane.dyad`
- ✅ `Components.Others.Solar.SunScreen` → `Thermal/SunScreen.dyad`
- ⏭ `Components.Others.Solar.SunIrradianceMultibody` — multibody panel orientation
- ⚠ `Components.Others.HeatTransfer.PlateTransient` → `Thermal/PlateTransient.dyad`
  (uniform N-node discretization; no internal-heat-port; structural-parameter pattern
  to dodge MTK substitution issues)
- ⚠ `Components.Others.HeatTransfer.CylinderTransient` →
  `Thermal/CylinderTransient.dyad` (uses `discretize_cylinder_conductances` from
  `Thermal/definitions.jl`)
- ✅ `Components.Others.HeatTransfer.SimpleAirExchanger` →
  `Thermal/SimpleAirExchanger.dyad`
- ⚠ `Components.Others.HeatTransfer.ConvRadSunWall` → `Thermal/ConvRadSunWall.dyad`
  (combined heat flux written directly; consumes `IrradiationOnPlane` / `SunScreen`
  outputs externally)
- ⏭ `Components.Others.HeatTransfer.ConvectionFactors.*` — not attempted
- ⏭ `Components.DataProcessing.RainflowCounter` / `FatigueCounter` — Modelica
  `algorithm` block + `pre()` + array push/pop. Right home: Julia post-processor.
- ⏭ `Components.Electrical.StartGenerator` — random schedule + start sequence
- ⏭ `SubComponents.Ikeda` — partial roll damping; no host hull yet
- ⏭ `SubComponents.VariableTranslation` — runtime-variable translation; PlanarMechanics
  `FixedTranslation` parameterizes `r` at compile time

### Tier 4 — New components (no direct upstream)
- 🆕 `Ship/Hull3DOF.dyad` — 3-DOF planar Newton-Euler hull body wrapping
  `PlanarMechanics.Body` + body-frame `WorldForceTorque` (drag + extras). Replacement
  for the multibody scaffolding around `HidrodynamicXYY`.
- 🆕 `Ship/Buoyancy1D.dyad` — 1D vertical heave + buoyancy + gravity. Decoupled from
  the planar Hull3DOF; runs as a separate vertical state with proper `g = 9.80665` and
  `ρ·V_submerged·g` restoring force. Replacement for the heave portion of
  `HidrodynamicZRP`.

## Submodule layout

```
dyad/
├── *.dyad                 # foundational: Environment, VariableEnvironment,
│                          # ApparentSpeedXY, hello
├── Ship/                  # hull body + ship-level forces / control
├── Propulsion/            # engines + propellers + rudder + sail
├── Machinery/             # deck + auxiliary equipment (cable, crane, electrical, data)
└── Thermal/               # heat transfer + solar + moist air (+ definitions.jl helpers)
```

## Strategy actually followed

1. ✅ **Pass 1 — Tier 1 first** (signal-domain ports). 18 components landed without
   needing any multibody.
2. ✅ **Pass 2 — Multibody discovery.** Found `MultibodyComponents.PlanarMechanics`
   shipped (just not in `agent_resources/stdlib_reference/`). Re-ported the 7
   propulsion / hull / wind / sail components with `Frame2D` coupling. Added 5 ports
   from HARD.md (TemperatureDataset, Plate / CylinderTransient, SolarIrradiation,
   ConvRadSunWall, simplified Propeller4Q / POD4Q / OnOffConsumer / Crane).
3. ✅ **Pass 3 — Code review + reorganization + buoyancy.** Fixed body→world frame
   rotation bug in Rudder / ShipWind / WingSail; reorganized into 4 submodules; added
   5 more analyses; added `Buoyancy1D` as the vertical complement to `Hull3DOF`.

## What's still open

Everything in `STILL_HARD.md`. The largest items by usefulness:

1. **MMG drag layer for Hull3DOF** — Julia helper `mmg_polynomial_drag(u, v, r, …)`
   feeding `hull.Fx_extra / Fy_extra / Mz_extra`. Gets us to a faithful
   `HidrodynamicXYY` port.
2. **Wageningen B-series Julia helper** — `wageningen_b_kt_kq(J, P_D, Ae_Ao, Z)` and
   `wageningen_b_4q(beta, …)`. Drops the quadratic-in-J approximation everywhere.
3. **Crane closed loop** — replace tension-only `Cable` with a `PlanarMechanics.Spring`
   or `SpringDamper` to break the algebraic loop.
4. **Rainflow / fatigue counters** — Julia post-processor consuming a `TransientAnalysis`
   stress time-series.
5. **3D Multibody (when shipped)** — unblocks HidrodynamicZRP, ShipModelTh, full Crane,
   Ikeda, VariableTranslation, SunIrradianceMultibody.

## Per-component-file outputs (what landed)

- 27 `.dyad` components.
- 15 `_analysis.dyad` analyses (one for each major component category).
- 1 `assets/temperature_24h.csv` data file.
- 1 `assets/buoyancy1d.png` validation plot.
- 1 `Thermal/definitions.jl` Julia helper module (sun position + cylinder discretization).
- HARD.md emptied; STILL_HARD.md catalogues 8 remaining items with concrete unblockers.
