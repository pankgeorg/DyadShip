"""
    DyadShip.FlettnerCFDLive

Live WaterLily.jl co-simulation driver for the `FlettnerRotorOnline` Dyad
component. **Verification-grade, not production**:

- The Dyad component reads its body-frame force from two `@register_symbolic`
  Julia functions (`DyadShip.flettner_live_fx(time)`,
  `DyadShip.flettner_live_fy(time)`). Those functions return the latest
  `STATE.Fx_body` / `STATE.Fy_body` — i.e. they are pure-lookup, so the rhs
  side of the ODE only sees a held-constant force between coupling steps.
- A `PeriodicCallback` installed by the caller fires every `Δt_coupling`
  seconds (typically 0.1–0.5 s). The callback reads the current apparent
  wind speed, apparent wind angle, and rotor ω from the integrator, calls
  `FlettnerCFDLive.step!(...)`, and the next ODE step's rhs evaluations
  pick up the new force.

The actual WaterLily geometry / `sim_step!` calls live in
`ext/FlettnerCFDLiveExt.jl` — that file is a package extension that loads
only when `using WaterLily` is also active in the caller's environment, so
`DyadShip` itself doesn't drag WaterLily into its dependency graph.

If `WaterLily` is not loaded, `init!` and `step!` raise; the `Fx`/`Fy`
readers always work and return the last cached values (0 by default), so
a Dyad model that *contains* `FlettnerRotorOnline` still compiles and
runs — it just won't see any force unless the callback is hooked up.
"""
module FlettnerCFDLive

mutable struct State
    Fx_body::Float64
    Fy_body::Float64
    Cl::Float64
    Cd::Float64
    sim::Any         # WaterLily.Simulation, populated by the extension
    R::Float64       # physical rotor radius [m]
    H::Float64       # physical rotor height [m]
    rho::Float64     # air density [kg/m³]
    sim_t::Float64   # accumulated WaterLily sim time
    inner_dt::Float64 # WaterLily time advance per coupling step
    n_calls::Int      # bookkeeping
end

const STATE = State(0.0, 0.0, 0.0, 0.0, nothing, 0.0, 0.0, 0.0, 0.0, 0.5, 0)

"""
    init!(; R, H, rho=1.29, n=128, Re=1e4, inner_dt=0.5)

Construct the WaterLily simulation for a 2D rotating cylinder in cross-flow.
Body diameter is `n/4` grid cells, domain is `(6n, n)`, Reynolds number `Re`
based on diameter and unit inflow. `inner_dt` is the WaterLily sim-time
advance per coupling step. `R`/`H` are the *physical* rotor radius/height
used to dimensionalize forces.

Forwards to `_init_impl!`, which the `FlettnerCFDLiveExt` package extension
overrides with the actual WaterLily wiring. The default `_init_impl!`
errors with a hint to `using WaterLily`.
"""
init!(args...; kwargs...) = _init_impl!(args...; kwargs...)

"""
    step!(t_dyad, U_app, alpha, omega)

Advance WaterLily by `inner_dt`, update `STATE.Fx_body / Fy_body` to the
body-frame force given current apparent wind speed `U_app`, apparent wind
angle in body frame `alpha`, and rotor angular velocity `omega`. Returns
`(Fx_body, Fy_body)` in Newtons. Forwards to `_step_impl!`, see `init!`.
"""
step!(args...; kwargs...) = _step_impl!(args...; kwargs...)

# Default impls — error unless the WaterLily extension is loaded, which
# overrides them with `function FlettnerCFDLive._{init,step}_impl!(...)`.
# Using a separate impl function (rather than overriding `init!` / `step!`
# directly) sidesteps the "Method overwriting is not permitted during Module
# precompilation" error you'd otherwise hit when the extension reloads.
function _init_impl!(args...; kwargs...)
    error(
        "FlettnerCFDLive.init! requires WaterLily.jl. Do `using WaterLily` first " *
        "(see scripts/run_flettner_cosim_verification.jl for the verification driver)."
    )
end
function _step_impl!(args...; kwargs...)
    error("FlettnerCFDLive.step! requires WaterLily.jl. Call `init!` first.")
end

"Force-on-body x-component (body frame), most recent CFD readout."
Fx() = STATE.Fx_body

"Force-on-body y-component (body frame), most recent CFD readout."
Fy() = STATE.Fy_body

"Reset state — leaves `sim` intact, just zeroes the force buffer."
reset_force!() = (STATE.Fx_body = 0.0; STATE.Fy_body = 0.0; nothing)

end # module
