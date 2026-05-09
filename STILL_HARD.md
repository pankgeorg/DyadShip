# Still HARD after multiple attempts

Components that resisted being ported even after second-pass attempts with
`MultibodyComponents.PlanarMechanics`, the `dyad/definitions.jl` Julia helper hook,
structural-parameter workarounds, and bigger event-free filter approximations.

## Components.DataProcessing.RainflowCounter / FatigueCounter

- **What it is:** A counter that takes a stream of stress peaks and tabulates them into
  a rainflow histogram for fatigue analysis. The upstream is a Modelica `algorithm`
  block with `pre()`-references, while-loops, and array-index-based push/pop on a
  state stack.
- **Why still HARD:** Dyad models are equation-based with no `algorithm` block, no
  `pre()` operator, no array push/pop, and no `when` events to detect peaks (a
  prerequisite for the counter). All three barriers stand simultaneously.
- **What I tried:** Considered modeling the histogram as a continuous discretization
  with bin counts as `der(bin_n) = δ(stress = bin_center)`-style equations. That blows
  up because Dyad doesn't support delta-shaped equations or array-index dispatch in
  derivatives.
- **Right home:** A Julia post-processing helper in `definitions.jl` that takes the
  array of stress peaks (e.g. extracted from a `TransientAnalysis` result via
  `sol[model.stress.y]`) and returns the rainflow histogram. The Dyad model produces
  the stress, the Julia helper consumes the time-series. Pattern matches how
  `discretize_cylinder_conductances` works.

## Components.Machines.Crane (full version)

- **Status:** A 2D side-view skeleton compiles (pedestal + boom + cable + load) but the
  end-to-end transient analysis hits an `ExtraVariablesSystemException` (3 derivative
  variables, 1 equation) that I couldn't resolve in the time budget.
- **What I tried:**
  1. First attempt used `boom_pivot.phi = boom_angle_state * π/180` directly — that
     gave an extra-equations error because the Revolute's `phi` is a state already.
  2. Second attempt routed the angle through a `RotationalComponents.Sources.Position`
     attached to the Revolute's `flange_a`, with a Fixed grounding the support spline.
     That gave an extra-variables error — I think the cable's tension-only ifelse
     creates an algebraic loop with the load body when the cable is slack.
- **Likely fix:** Replace `Cable` with a stiff `PlanarMechanics.Spring` or a
  `SpringDamper` so there are no `ifelse` branches; OR add a small damper to the load
  body to break the algebraic loop. Out of scope for this pass.

## Components.Electrical.TriggerConsumer / StartGenerator

- **Status:** `OnOffConsumer` was successfully ported as a continuous-time variant
  taking `WorkSignal::RealInput`. The trigger / start-generator variants extend
  `RandomStart` *and* add their own discrete event logic (5-step trigger, generator
  start sequence). Even the deterministic-driven port would lose enough fidelity to
  not be worth it without `when` events.
- **Right home:** A Julia state-machine block consumed via the `func()` parameter
  pattern, fed into the consumer by a `RealInput`.

## Components.SubComponents.VariableTranslation

- **What it is:** A Modelica multibody `FixedTranslation` whose translation vector
  `r` can be modified at runtime via a `RealInput[3]`.
- **Why still HARD:** PlanarMechanics's `FixedTranslation` takes `r` as a `parameter`
  (compile-time). To make it runtime-variable would require rewriting the joint's
  position constraint (`r0 = R(phi)·r` and `frame_b - frame_a = r0`) with `r` as a
  variable. Doable, but each of the helper renderable / shape parameters chain on
  fixed `r` — so it's a non-trivial copy-then-modify.
- **Likely fix:** Copy `FixedTranslation` into our Dyad library, replace `parameter
  r::Length[2]` with `RealInput`s `r_x` and `r_y`, drop the renderable shape, and add
  the position-constraint equation in `relations`. Tactical, not strategic.

## Components.SubComponents.Ikeda

- **What it is:** A partial roll-damping decomposition (B_F frictional, B_W wave, …)
  that the upstream model uses *inside* the `HidrodynamicXYY` / `HidrodynamicZRP`
  hull blocks. It's a math-only block with no ports of its own.
- **Why still HARD:** The full hull (`HidrodynamicZRP`) wasn't ported (3D / roll
  dynamics aren't in `PlanarMechanics`), so Ikeda has no host to plug into.
- **Right home:** Whenever a 3D hull port lands, add Ikeda as a `partial component`
  consumed by `extends`.

## Components.Ship.HidrodynamicZRP / ShipModelTh

- **Status:** Not attempted. These are the heave/roll/pitch hydrodynamics + top-level
  ship model. Both are inherently 3D and don't fit `PlanarMechanics`.
- **Right home:** A 3D Multibody library if/when one ships.

## Components.Others.MoistAir.SourceMoistAir

- **What it is:** A Modelica.Fluid moist-air boundary source.
- **Why still HARD:** Dyad's `HydraulicComponents` is liquid-only; there is no Dyad
  moist-air medium model.
- **Right home:** A new `DyadShip.MoistAirMedia` Julia package, then a Dyad fluid
  source over it. Out of scope.

## Components.DataProcessing.PeakSampler (strict event version)

- **Status:** A continuous peak-hold approximation was successfully ported as
  `PeakSampler.dyad`. The strict event-based version (output = `u` exactly at every
  local extremum, latching between peaks) requires `when der(u) == 0` events, which
  Dyad doesn't expose. The continuous version is good enough for many use cases but
  isn't a pixel-perfect port.
- **Right home:** Same as Rainflow — a Julia post-processing function on the
  simulation result.
