#!/usr/bin/env julia
#
# Live WaterLily-Dyad co-simulation, for verification of the offline
# Cl/Cd table approach.
#
# Procedure:
#   1. Build the table-based `FullShipFlettnerFavorableRender` analysis and
#      solve as usual (uses `assets/flettner_coeffs.csv` precomputed by
#      `run_waterlily_flettner.jl`).
#   2. Build the *online* analysis component
#      `FullShipFlettnerOnlineRender`, which substitutes `FlettnerRotorOnline`
#      for `FlettnerRotor`. Its force output is driven by two
#      `@register_symbolic` Julia functions whose values are written into
#      `DyadShip.FlettnerCFDLive.STATE` by a `DiffEqCallbacks.PeriodicCallback`
#      firing every `COUPLING_DT` seconds. The callback reads (`U_app`,
#      `α`, `ω`) from the integrator, advances WaterLily.jl by `INNER_DT`
#      of sim time, and stores the resulting body-frame force back.
#   3. Solve the online problem with the callback attached.
#   4. Compare the rotor body-frame force traces and ship trajectories
#      between table-based and live-CFD runs over `t ∈ [0, 90 s]`. Write a
#      side-by-side plot to `assets/flettner_cosim_verification.png`.
#
# Cost: with COUPLING_DT = 0.2 s and INNER_DT = 0.5 in sim units at n=64,
# the run does ~450 WaterLily steps end to end. Expect 1–3 minutes of CFD
# wall time on a CPU.
#
# Usage: `../julia-dyad.sh --project=scripts scripts/run_flettner_cosim_verification.jl`

using Pkg
Pkg.activate(@__DIR__)

# Idempotently dev DyadShip from the parent path. Without this the script
# would have to be run from the parent project, but then WaterLily isn't
# resolved.
# DyadShip is referenced by relative dev-path (it isn't registered).
# Always call Pkg.develop — it's idempotent and updates the manifest's
# dev entry. Pkg.instantiate would otherwise try to resolve DyadShip from
# the registry and fail since it isn't published there.
Pkg.develop(path = joinpath(@__DIR__, ".."))
Pkg.instantiate()

using WaterLily         # loads FlettnerCFDLiveExt as a side effect of also loading DyadShip
using DyadShip
using ModelingToolkit
using DiffEqCallbacks
using SymbolicIndexingInterface
using SciMLBase
using Plots
using Printf
gr()

const ASSETS = abspath(joinpath(@__DIR__, "..", "assets"))
mkpath(ASSETS)

const COUPLING_DT = 0.2
const INNER_DT    = 0.5
const N_GRID      = 128       # match the offline table generation grid
const RE          = 1.0e4
const T_STOP      = 90.0

# -- Stage 1: run the table-based favorable analysis as a reference -----
println("─"^72)
println("[1/3] Running table-based reference (FullShipFlettnerFavorableRender)…")
ref = DyadShip.Ship.ShipFlettnerFavorableRenderTransient()
sol_ref = ref.sol
sys_ref = ref.sys

ts = collect(range(0.0, T_STOP; length = 451))
ref_Fx = [sol_ref(t; idxs = sys_ref.rotor.Force_X) for t in ts]
ref_Fy = [sol_ref(t; idxs = sys_ref.rotor.Force_Y) for t in ts]
ref_u  = [sol_ref(t; idxs = sys_ref.hull.u)        for t in ts]
ref_px = [sol_ref(t; idxs = sys_ref.hull.pos_x)    for t in ts]
ref_py = [sol_ref(t; idxs = sys_ref.hull.pos_y)    for t in ts]
println("  reference t ∈ [0, $T_STOP]  $(length(ts)) samples.")

# -- Stage 2: initialize WaterLily driver -------------------------------
println("─"^72)
println("[2/3] Initializing WaterLily driver and online analysis…")
DyadShip.FlettnerCFDLive.init!(; R = 2.5, H = 24, n = N_GRID,
                                  Re = RE, inner_dt = INNER_DT)

# Build the online model from its Dyad-generated constructor and compile.
# Dyad-generated test components require `name` (use the @named macro).
@named model = DyadShip.Ship.FullShipFlettnerOnlineRender()
sys_full = mtkcompile(model)
prob = ODEProblem(sys_full, Pair[], (0.0, T_STOP))
println("  ODE problem built. unknowns=$(length(unknowns(sys_full)))")

# Build getters for the three signals the callback consumes. `model.rotor.*`
# resolves against the original (pre-compile) system, which is fine — the
# observables survive mtkcompile.
const _get_U     = getsym(sys_full, model.rotor.U_app_obs)
const _get_alpha = getsym(sys_full, model.rotor.alpha_obs)
const _get_omega = getsym(sys_full, model.rotor.omega_obs)

const cb_log_t  = Float64[]
const cb_log_Fx = Float64[]
const cb_log_Fy = Float64[]
const cb_log_U  = Float64[]
const cb_log_a  = Float64[]
const cb_log_om = Float64[]
const cb_log_Cl = Float64[]
const cb_log_Cd = Float64[]

function cosim_affect!(integ)
    t      = integ.t
    U_app  = _get_U(integ)
    alpha  = _get_alpha(integ)
    omega  = _get_omega(integ)
    Fx, Fy = DyadShip.FlettnerCFDLive.step!(t, U_app, alpha, omega)
    push!(cb_log_t,  t)
    push!(cb_log_Fx, Fx)
    push!(cb_log_Fy, Fy)
    push!(cb_log_U,  U_app)
    push!(cb_log_a,  alpha)
    push!(cb_log_om, omega)
    push!(cb_log_Cl, DyadShip.FlettnerCFDLive.STATE.Cl)
    push!(cb_log_Cd, DyadShip.FlettnerCFDLive.STATE.Cd)
    SciMLBase.u_modified!(integ, false)
    return nothing
end

cb = PeriodicCallback(cosim_affect!, COUPLING_DT; initial_affect = true)

# -- Stage 3: solve online + compare ------------------------------------
println("─"^72)
println("[3/3] Solving online analysis with PeriodicCallback (Δt=$(COUPLING_DT) s)…")
println("      (each callback runs one WaterLily sim_step! of $(INNER_DT) sim-time units;")
println("      ~$(Int(round(T_STOP/COUPLING_DT))) callbacks total)")
@time sol_on = solve(prob; callback = cb, reltol = 1e-5, abstol = 1e-7)
println("  online sim t_end=$(sol_on.t[end])  n_samples=$(length(sol_on.t))")
println("  WaterLily total step calls: $(DyadShip.FlettnerCFDLive.STATE.n_calls)")

ts_on  = filter(t -> t <= sol_on.t[end], ts)
# IMPORTANT: rotor.Force_X is an *observable* defined by the registered
# functions `flettner_live_fx(t)` / `flettner_live_fy(t)`, which read
# `FlettnerCFDLive.STATE.Fx_body` / `.Fy_body`. Those globals only hold the
# *most recent* callback's value — they have no per-time-point history,
# so `sol_on(t; idxs=Force_X)` would just give the final force at every t.
# Use the per-callback log (`cb_log_t`, `cb_log_Fx`, `cb_log_Fy`) for the
# actual live time series — those are the values the integrator saw during
# the run between callbacks.
on_t_cb = cb_log_t
on_Fx   = cb_log_Fx
on_Fy   = cb_log_Fy

# Hull state IS persisted in `sol_on.u(t)`, so trajectory + speed are fine.
on_u  = [sol_on(t; idxs = sys_full.hull.u)        for t in ts_on]
on_px = [sol_on(t; idxs = sys_full.hull.pos_x)    for t in ts_on]
on_py = [sol_on(t; idxs = sys_full.hull.pos_y)    for t in ts_on]

println()
println("Comparison at sample points (cb_* is the live force at the coupling step nearest t):")
println("  t        ref Fx (kN)   on Fx (kN)    ref Fy (kN)   on Fy (kN)    ref u   on u")
for t in (10.0, 30.0, 60.0, 90.0)
    t > sol_on.t[end] && continue
    i  = findfirst(==(t), ts)
    j  = findfirst(==(t), ts_on)
    kb = argmin(abs.(on_t_cb .- t))   # nearest callback log entry
    @printf("  %5.1f    %+9.1f      %+9.1f       %+9.1f      %+9.1f       %5.2f   %5.2f\n",
            t, ref_Fx[i]/1e3, on_Fx[kb]/1e3, ref_Fy[i]/1e3, on_Fy[kb]/1e3,
            ref_u[i], on_u[j])
end

println()
println("Live WaterLily callback diagnostic (sampled every 50th callback):")
println("  t        ω (rad/s)   U_app   α (rad)    ξ           Cl_live    Cd_live")
for k in 1:50:length(cb_log_t)
    ξ_k = cb_log_om[k] * 2.5 / max(abs(cb_log_U[k]), 1e-3)
    @printf("  %5.1f      %5.2f    %5.2f   %+6.3f    %+6.3f     %+7.3f    %+7.3f\n",
            cb_log_t[k], cb_log_om[k], cb_log_U[k], cb_log_a[k], ξ_k,
            cb_log_Cl[k], cb_log_Cd[k])
end

println()
println("Rendering comparison plot…")
p_Fx = plot(ts, ref_Fx ./ 1e3; label = "table (offline CFD)", lw = 2, c = :steelblue,
            xlabel = "t [s]", ylabel = "F_X body [kN]",
            title = "Rotor surge force — table vs live WaterLily")
plot!(p_Fx, on_t_cb, on_Fx ./ 1e3;
      label = "live (online CFD, per coupling step)", lw = 1, c = :crimson, seriestype = :steppost)

p_Fy = plot(ts, ref_Fy ./ 1e3; label = "table", lw = 2, c = :steelblue,
            xlabel = "t [s]", ylabel = "F_Y body [kN]",
            title = "Rotor sway force — table vs live WaterLily")
plot!(p_Fy, on_t_cb, on_Fy ./ 1e3;
      label = "live", lw = 1, c = :crimson, seriestype = :steppost)

p_traj = plot(ref_px, ref_py; label = "table", lw = 2, c = :steelblue,
              xlabel = "x [m]", ylabel = "y [m]",
              title = "Ship trajectory (first $(T_STOP) s)",
              aspect_ratio = :equal)
plot!(p_traj, on_px, on_py; label = "live", lw = 2, c = :crimson, ls = :dash)

p_diag = plot(cb_log_t, cb_log_Cl; label = "Cl_live", lw = 2, c = :purple,
              xlabel = "t [s]", ylabel = "coefficient",
              title = "Live WaterLily coefficients over time")
plot!(p_diag, cb_log_t, cb_log_Cd; label = "Cd_live", lw = 2, c = :darkorange)
hline!(p_diag, [0]; c = :gray, lw = 0.5, label = "")

p_all = plot(p_Fx, p_Fy, p_traj, p_diag; layout = (2, 2), size = (1200, 800))
out_png = joinpath(ASSETS, "flettner_cosim_verification.png")
savefig(p_all, out_png)
println("  wrote $out_png")
