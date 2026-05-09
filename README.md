# DyadShip

A [Dyad](https://help.juliahub.com/dyad/) port of the Modelica naval-architecture
library [ShipSIM](https://github.com/BasilioPV/ShipSIM) by Basilio Puente and
M Dolores Fernandez.

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

## Getting Started

Dyad models live in `dyad/`; the Dyad compiler emits Julia into `generated/`.
Do not edit files in `generated/` directly.

1. Open the project in VS Code with the Dyad Studio extension, or work from
   the REPL with `julia --project=.`.
2. At the `pkg>` prompt: `instantiate` (one-time package download), then
   `test` to run the bundled analyses.
3. From the Julia REPL: `using DyadShip`, then run any of the analyses
   defined under `dyad/Ship/`, `dyad/Propulsion/`, `dyad/Machinery/`, or
   `dyad/Thermal/`.

## Layout

- `dyad/Ship/` — hull, autopilot, anti-heeling, buoyancy, ship-wind force.
- `dyad/Propulsion/` — propellers (1Q, 4Q, POD), rudder, wing-sail, diesel.
- `dyad/Machinery/` — crane, cable, on-off consumer, peak sampler.
- `dyad/Thermal/` — irradiation, sun-screen, plate/cylinder transients.
- `agent_resources/` — local tooling notes (gitignored).
- `REPORT.md`, `SUCCESSES.md`, `HARD.md`, `STILL_HARD.md`, `PORT_PLAN.md` —
  porting log from the initial autonomous session.

## License

The Dyad rewrite in this repository is © 2025 Panagiotis Georgakopoulos. The
upstream Modelica `ShipSIM` library is © Basilio Puente and M Dolores
Fernandez, distributed under the 3-clause BSD license.
