# DyadShip

A [Dyad](https://help.juliahub.com/dyad/) port of the Modelica naval-architecture
library [ShipSIM](https://github.com/BasilioPV/ShipSIM) by Basilio Puente and
M Dolores Fernandez, extended with a **WaterLily.jl CFD-driven Flettner rotor**
propulsor.

> **This is a rewrite, not a translation.** The Modelica components have been
> reimplemented in Dyad and adapted to the components available in
> `MultibodyComponents`, `RotationalComponents`, `BlockComponents`, and the
> rest of the Dyad standard libraries. Some upstream features that depend on
> Modelica facilities not yet available in Dyad (3D multibody, `when`/discrete
> events, `Modelica.Fluid` moist-air media, full Wageningen-B propeller
> regressions, etc.) have been simplified — see the per-component docstrings
> for the assumptions taken in each case, and `HARD.md` / `STILL_HARD.md` for
> components that were skipped.

The upstream Modelica library lives at
<https://github.com/BasilioPV/ShipSIM> and is distributed under the 3-clause
BSD license. Per-component docstrings cite the originals they're ported from.

## Highlights

- **Closed-loop autopilot transit.** A `HeadingAutoPilot` (PI on heading +
  throttle ramp on approach) steers a `HullMMG` hull from origin to a target
  waypoint with diesel propeller, rudder, and time-varying wind windage.
- **CFD-driven Flettner rotor propulsion.** A new `FlettnerRotor` component
  whose `Cl(ξ), Cd(ξ)` coefficients come from offline
  [WaterLily.jl](https://github.com/WaterLily-jl/WaterLily.jl) simulations of a
  2D rotating cylinder in cross-flow. Reads coefficients live from
  `assets/flettner_coeffs.csv` via two `BlockComponents.Tables.Interpolation`
  blocks. Source pattern is G. D. Weymouth's canonical SpinCyl example.
- **Three-way wind-orientation comparison.** Identical hull / propulsion /
  autopilot stack, three different rotor scenarios:

| Analysis | Rotor | Mean wind from | Time to target |
|---|---|---|---|
| `ShipRenderTransient` | none (diesel only) | NE (45°) | doesn't reach by t=2200 s |
| `ShipFlettnerRenderTransient` | yes | NE (45°), bow-quarter | doesn't reach by t=1500 s |
| `ShipFlettnerFavorableRenderTransient` | yes | N (0°), beam | **reaches at t ≈ 1200 s** |

## Animations

The 1500–2200 s simulations are compressed to 30-second MP4s into `assets/`.
Same axis limits, same target star, same wind-arrow corner across all three
so the comparison is purely about the rotor's contribution. Flettner frames
add a purple rotor glyph on the hull (spin chord visible at ×80 slow-down)
and a crimson rotor-force arrow.

### Baseline — diesel only, NE wind

<video src="assets/ship_animation.mp4" controls width="720">
  Your viewer doesn't render embedded MP4. Open
  <a href="assets/ship_animation.mp4">assets/ship_animation.mp4</a>.
</video>

### Flettner rotor, unfavorable bow-quarter wind

<video src="assets/ship_flettner_animation.mp4" controls width="720">
  <a href="assets/ship_flettner_animation.mp4">assets/ship_flettner_animation.mp4</a>
</video>

Rotor produces ~400 kN of Magnus lift but only 4–28 % of it is in surge —
most of the force is lateral. The hull sideslips, the autopilot uses the
rudder hard, induced drag eats the surge gain. Net result: ~13 % extra
distance vs. baseline (1.7 km more after 1500 s), but average speed is
similar.

### Flettner rotor, favorable beam wind

<video src="assets/ship_flettner_favorable_animation.mp4" controls width="720">
  <a href="assets/ship_flettner_favorable_animation.mp4">assets/ship_flettner_favorable_animation.mp4</a>
</video>

Same rotor, same hull, wind direction rotated 45° so the apparent wind hits
the ship broadside. Magnus lift now decomposes 39–51 % into surge — the
rotor delivers up to **+300 kN of forward thrust on top of the 110 kN
propeller**, and ship surge speed peaks at **8.5 m/s** (vs. 5.2 m/s
baseline). The ship reaches the destination at **t ≈ 1200 s, ~40 % faster
than baseline**.

## Getting Started

Dyad models live in `dyad/`; the Dyad compiler emits Julia into `generated/`.
Do not edit files in `generated/` directly.

```sh
# Compile Dyad → Julia
../dyad.sh compile

# Run an analysis from the REPL
../julia-dyad.sh -e 'using DyadShip; DyadShip.Ship.ShipFlettnerFavorableRenderTransient()'

# Render the three comparison animations into assets/
../julia-dyad.sh scripts/render_all.jl

# Re-characterize the Flettner rotor with WaterLily (writes assets/flettner_coeffs.csv;
# uses scripts/Project.toml so WaterLily isn't on the main library deps)
../julia-dyad.sh --project=scripts scripts/run_waterlily_flettner.jl
```

The wrappers `../julia-dyad.sh` and `../dyad.sh` set the JuliaUp channel and
package server this project expects (`dyad-3.0.0-rc5`, `juliahub.com`); call
them rather than `julia` / the Dyad CLI directly.

## Layout

- `dyad/` — Dyad component sources
  - `dyad/Ship/` — hull, autopilot, anti-heeling, buoyancy, ship-wind force,
    full-ship analyses (with and without Flettner rotor).
  - `dyad/Propulsion/` — propellers (1Q, 4Q, POD), rudder, wing-sail, diesel,
    **Flettner rotor**.
  - `dyad/Machinery/` — crane, cable, on-off consumer, peak sampler.
  - `dyad/Thermal/` — irradiation, sun-screen, plate/cylinder transients.
- `generated/` — auto-generated Julia (do not edit).
- `src/DyadShip.jl` — thin Julia module wrapper around `generated/module.jl`.
- `scripts/` — runnable analysis + rendering + WaterLily characterization
  scripts; carries its own `Project.toml` for the WaterLily dependency.
- `assets/` — bundled datasets (temperature CSV, Flettner coefficient CSV)
  and generated artifacts (animation MP4s, diagnostic PNGs).
- `REPORT.md`, `SUCCESSES.md`, `HARD.md`, `STILL_HARD.md`, `PORT_PLAN.md` —
  porting log from the initial autonomous session.
- `agent_resources/` — local tooling notes (gitignored).

## How the Flettner rotor talks to Dyad

WaterLily.jl can't run live inside the Dyad ODE solver — a single CFD step
costs many milliseconds and the adaptive stepper calls the rhs hundreds of
times per simulated second. Real Flettner designs (Norsepower, Maranet) use
the same pattern as `dyad/Propulsion/FlettnerRotor.dyad`:

1. **Characterize once.** `scripts/run_waterlily_flettner.jl` runs the
   spinning-cylinder simulation over a sweep of spin ratios ξ ∈ [0, 6] (the
   exact `AutoBody(sdf, rotation_map)` + `WaterLily.total_force` pattern
   from Weymouth's `SpinCylOptim.jl`), writes `assets/flettner_coeffs.csv`.
2. **Read live.** Two `BlockComponents.Tables.Interpolation` blocks inside
   `FlettnerRotor` look up `Cl(ξ)` and `Cd(ξ)` per ODE step. CubicSpline
   interpolation keeps the rhs Jacobian smooth across the sweep points.
3. **Decompose into body axes.** Magnus lift is perpendicular to the
   apparent wind, with the sign of `ω` flipping which side it points to.

The CSV stores raw `WaterLily.total_force` (force-on-fluid convention per
`Metrics.jl::pressure_force`); the Dyad component negates internally to
the engineering force-on-body convention. The docstring on
`dyad/Propulsion/FlettnerRotor.dyad` spells this out.

### Live co-simulation (verification only)

`FlettnerRotorOnline` is a sibling component that gets its body-frame force
from a `@register_symbolic` Julia callback instead of the offline-CFD table.
The forces are written by a `DiffEqCallbacks.PeriodicCallback` that fires
every `Δt_cosim = 0.2 s` and steps WaterLily one inner sim-time unit; the
machinery lives in `src/FlettnerCFDLive.jl` (stubs + force lookups) +
`ext/FlettnerCFDLiveExt.jl` (the real WaterLily wiring, behind a package
extension keyed on `WaterLily`). The driver is documented in the docstrings.

This exists to **verify** the offline-table approach against actual live
CFD, not for production runs. CFD wall-time (90 s of sim, 450 callbacks,
WaterLily inner_dt = 0.5):

| Grid | T_STOP | Δt_cosim | Backend | CFD wall | per-step | vs. CPU baseline |
|---|---|---|---|---|---|---|
| `n=128` | 90 s   | 0.2 s | CPU (single-thread) | 458 s | 1.02 s | — |
| `n=128` | 90 s   | 0.2 s | GPU (RTX 2000 Ada Laptop) | 324 s | 0.72 s | 1.4× |
| `n=256` | 90 s   | 0.2 s | GPU | 793 s | 1.76 s | ~2.3× vs estimated CPU |
| `n=128` | 1500 s | 1.0 s | GPU (the full-arrival video below) | 1784 s | 1.19 s | — |

The GPU path is `mem = CUDA.CuArray` passed through `FlettnerCFDLive.init!`
to `WaterLily.Simulation`; the script auto-detects `CUDA.functional()`
and falls back to CPU when no device is present. **Speedup at `n=128` is
modest** because the grid is tiny (~98 k cells) — each WaterLily kernel
finishes in microseconds while the CPU-side dispatch overhead is ~10 µs
per launch, so the GPU spends most of its time waiting for the next
kernel. `n=256` (~400 k cells) starts to amortize the dispatch cost and
also produces noticeably better-matched Cl/Cd vs the offline reference
(e.g., live `Cl` at `t=10` is 2.4 at n=256 vs 0.8 at n=128, table value
~2.5).

Override the grid with `N_GRID=256 ../julia-dyad.sh --project=scripts
scripts/run_flettner_cosim_verification.jl`. Outputs are tagged with
`_n{N}_{cpu|gpu}` so multiple runs don't overwrite each other; the
untagged `flettner_cosim_animation.mp4` / `flettner_cosim_verification.png`
in the repo are the canonical n=256 GPU pair.

The script writes:

- `assets/flettner_cosim_verification.png` — four-panel plot comparing
  rotor surge force, sway force, ship trajectory, and live `Cl(t)` / `Cd(t)`
  against the table-driven `FullShipFlettnerFavorableRender` reference.
- `assets/flettner_cosim_animation.mp4` — full-arrival run: 1500 s of
  simulated transit compressed into 25 s (30 fps, 750 frames). Top panel
  shows the ship's map view as it travels from origin to the (10000, 1000)
  target; bottom panel shows the WaterLily z-vorticity field — actual CFD
  flow around the rotating cylinder — synchronized to the same simulated
  time. You can see the Magnus boundary layer roll up and the wake bias
  swing as the apparent-wind angle shifts under the time-varying wind.

<video src="assets/flettner_cosim_animation.mp4" controls width="720">
  <a href="assets/flettner_cosim_animation.mp4">assets/flettner_cosim_animation.mp4</a>
</video>

The ship in the live cosim arrives at **t ≈ 1200 s** — close to the
table-based favorable run's 1200 s arrival, despite live `Cl` running
~20–30 % under the table at the early ramp-up phase. The
`_n128_gpu` / `_n256_gpu` tagged variants are also bundled for direct
short-window comparison.

Table and live agree on shape, sign, and trajectory; live magnitudes run
~20–30% under the table because the body-rebuild-each-callback approach
restarts the BDIM transient between coupling steps. Good enough to
confirm the table is in the right physical regime.

To run:

```sh
../julia-dyad.sh --project=scripts scripts/run_flettner_cosim_verification.jl
```

The first invocation `Pkg.develop`s the parent project into `scripts/` and
precompiles WaterLily + the package extension.

## License

The Dyad rewrite in this repository is © 2025 Panagiotis Georgakopoulos. The
upstream Modelica `ShipSIM` library is © Basilio Puente and M Dolores
Fernandez, distributed under the 3-clause BSD license.
