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
using CUDA
gr()

# Run the WaterLily side on the GPU if one is present and functional;
# otherwise fall back to CPU. Override with `USE_GPU = false` here to
# force CPU mode regardless. WaterLily's `mem` kwarg routes its flow
# arrays to the corresponding array type.
const USE_GPU = CUDA.functional()
const MEM_TYPE = USE_GPU ? CUDA.CuArray : Array
if USE_GPU
    @info "WaterLily on GPU" device=CUDA.name(CUDA.device()) free_GB=round(CUDA.free_memory()/1e9; digits=2)
else
    @info "WaterLily on CPU (CUDA not functional)"
end

const ASSETS = abspath(joinpath(@__DIR__, "..", "assets"))
mkpath(ASSETS)

# All four knobs are env-overridable so the same script handles short
# verification (defaults) and the full-arrival rendering job.
#
#   T_STOP=1500 COUPLING_DT=1.0 N_GRID=128  ../julia-dyad.sh --project=scripts scripts/...jl
#
# would run the rotor for 1500 s of sim time, callback every 1 s
# (= 1500 CFD steps), at n=128 — long enough for the ship to reach the
# target at ~t=1200 in the favorable wind setup.
const COUPLING_DT = parse(Float64, get(ENV, "COUPLING_DT", "0.2"))
const INNER_DT    = parse(Float64, get(ENV, "INNER_DT",    "0.5"))
const N_GRID      = parse(Int,     get(ENV, "N_GRID",      "128"))
const RE          = parse(Float64, get(ENV, "RE",          "1.0e4"))
const T_STOP      = parse(Float64, get(ENV, "T_STOP",      "90.0"))

# Vorticity-snapshot stride: subsample so the rendered MP4 stays at a
# reasonable length (~25 s at 30 fps ⇒ ~750 frames). At short T_STOP
# every callback snaps; at long T_STOP we skip every N-th.
const N_CALLBACKS_EST = ceil(Int, T_STOP / COUPLING_DT)
const MAX_VID_FRAMES  = 750
const SNAPSHOT_STRIDE = max(1, ceil(Int, N_CALLBACKS_EST / MAX_VID_FRAMES))

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
                                  Re = RE, inner_dt = INNER_DT,
                                  mem = MEM_TYPE)

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

const cb_log_t      = Float64[]
const cb_log_Fx     = Float64[]
const cb_log_Fy     = Float64[]
const cb_log_U      = Float64[]
const cb_log_a      = Float64[]
const cb_log_om     = Float64[]
const cb_log_Cl     = Float64[]
const cb_log_Cd     = Float64[]
const cb_log_vort   = Array{Float32,2}[]   # subsampled WaterLily vorticity
const cb_log_vort_t = Float64[]            # matching times for the snapshots

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

    # Snapshot the WaterLily vorticity field for the animation. SpinCylOptim
    # pattern: write the dimensionless z-curl of velocity into the scratch
    # field `sim.flow.σ`, then snapshot. ~200 KB per frame at n=128. The
    # `Array(…)` wrapper brings the field back from GPU when the sim is
    # running on `mem=CUDA.CuArray`; on CPU it's a cheap no-op `copy`.
    # Subsample at SNAPSHOT_STRIDE so a long T_STOP doesn't produce a
    # 50+ second video — `cb_log_t` index gates the snapshot decision.
    if (length(cb_log_t) - 1) % SNAPSHOT_STRIDE == 0
        sim_h = DyadShip.FlettnerCFDLive.STATE.sim
        WaterLily.@inside sim_h.flow.σ[I] = WaterLily.curl(3, I, sim_h.flow.u) * sim_h.L
        push!(cb_log_vort, Array(sim_h.flow.σ))
        push!(cb_log_vort_t, t)
    end

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
# Sample points: scale with T_STOP so the same script handles short
# verification (90 s) and full-arrival (1500 s).
const _sample_times = if T_STOP <= 120
    (10.0, 30.0, 60.0, 90.0)
else
    (30.0, 300.0, 600.0, 900.0, 1200.0, min(1500.0, T_STOP))
end
for t in _sample_times
    t > sol_on.t[end] && continue
    i_idx  = argmin(abs.(ts .- t))
    j_idx  = argmin(abs.(ts_on .- t))
    kb = argmin(abs.(on_t_cb .- t))
    @printf("  %5.1f    %+9.1f      %+9.1f       %+9.1f      %+9.1f       %5.2f   %5.2f\n",
            t, ref_Fx[i_idx]/1e3, on_Fx[kb]/1e3, ref_Fy[i_idx]/1e3, on_Fy[kb]/1e3,
            ref_u[i_idx], on_u[j_idx])
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
# Tag the output file with the grid size + backend so n=128 and n=256
# runs don't overwrite each other.
const TAG = "n$(N_GRID)_$(USE_GPU ? "gpu" : "cpu")"
out_png = joinpath(ASSETS, "flettner_cosim_verification_$(TAG).png")
savefig(p_all, out_png)
println("  wrote $out_png")

# ─────────────────────────────────────────────────────────────────────────
# Animation: the actual WaterLily vorticity field evolving as the cosim
# advances, with the rotor's instantaneous (ω, ξ, Cl, Cd) annotated. This
# is the most "CFD-flavored" view of the verification — boundary layer
# rolling up around the spinning cylinder, Magnus side lift visible as a
# vortex-shedding bias.
# ─────────────────────────────────────────────────────────────────────────
println("Rendering combined map + WaterLily-vorticity cosim animation…")
nframes = length(cb_log_vort)
fps = 30
println("  $(nframes) frames at $(fps) fps  → $(round(nframes/fps; digits=1)) s video")

# Pre-sample ship trajectory + heading + wind at the vorticity-snapshot
# times (= cb_log_vort_t, which may be a subsample of cb_log_t when
# SNAPSHOT_STRIDE > 1).
ship_t  = cb_log_vort_t
ship_x  = [sol_on(t; idxs = sys_full.hull.pos_x)         for t in ship_t]
ship_y  = [sol_on(t; idxs = sys_full.hull.pos_y)         for t in ship_t]
ship_psi = [sol_on(t; idxs = sys_full.hull.psi)          for t in ship_t]
wspd    = [sol_on(t; idxs = sys_full.wind_speed_now)     for t in ship_t]
wdir    = [sol_on(t; idxs = sys_full.wind_direction_now) for t in ship_t]
# Match-by-time lookups into the full callback log for the per-frame
# annotations (force, ω, ξ, Cl, Cd, U_app, α).
vort_to_cb = [argmin(abs.(cb_log_t .- t)) for t in ship_t]

# Ship glyph (same shapes as render_all.jl).
const SHIP_LEN = 200.0
const SHIP_HALF_BEAM = 70.0
const SHIP_LOCAL = [
    ( SHIP_LEN*0.5,  0.0),
    (-SHIP_LEN*0.5,  SHIP_HALF_BEAM),
    (-SHIP_LEN*0.5, -SHIP_HALF_BEAM),
]

# Map extent — auto-fit to the actual trajectory + target (so short
# T_STOP and full-arrival T_STOP both look right). Buffer the box so the
# ship glyph + arrow stay inside the frame.
const TARGET_X, TARGET_Y = 10000.0, 1000.0
const _x_lo = min(0.0, minimum(ship_x), TARGET_X)
const _x_hi = max(0.0, maximum(ship_x), TARGET_X)
const _y_lo = min(0.0, minimum(ship_y), TARGET_Y)
const _y_hi = max(0.0, maximum(ship_y), TARGET_Y)
const _pad_x = max(800.0, 0.08 * (_x_hi - _x_lo))
const _pad_y = max(400.0, 0.20 * (_y_hi - _y_lo))
const MAP_XLIMS = (_x_lo - _pad_x, _x_hi + _pad_x)
const MAP_YLIMS = (_y_lo - _pad_y, _y_hi + _pad_y)

anim = @animate for k in 1:nframes
    t = ship_t[k]
    cb = vort_to_cb[k]
    ξ_k = cb_log_om[cb] * 2.5 / max(abs(cb_log_U[cb]), 1e-3)
    px, py, ψ = ship_x[k], ship_y[k], ship_psi[k]
    ws, wd = wspd[k], wdir[k]

    # ─── Top panel: ship map view ───────────────────────────────────────
    p_map = plot(ship_x[1:k], ship_y[1:k];
                 label = "", lw = 2, c = :steelblue,
                 xlabel = "x [m] (East→)", ylabel = "y [m] (North↑)",
                 title = "Ship map  |  t = $(round(t; digits=1)) s  |  " *
                         "u = $(round(sol_on(t; idxs=sys_full.hull.u); digits=2)) m/s  |  " *
                         "ω = $(round(cb_log_om[cb]; digits=1)) rad/s",
                 titlefontsize = 9,
                 aspect_ratio = :equal,
                 xlims = MAP_XLIMS, ylims = MAP_YLIMS,
                 legend = false)

    # Ship glyph (small triangle).
    sx = [px + cos(ψ)*lx - sin(ψ)*ly for (lx, ly) in SHIP_LOCAL]
    sy = [py + sin(ψ)*lx + cos(ψ)*ly for (lx, ly) in SHIP_LOCAL]
    plot!(p_map, [sx; sx[1]], [sy; sy[1]];
          seriestype = :shape, c = :crimson, lw = 1, fillalpha = 0.7, label = "")

    # Rotor force arrow from the ship.
    Fx_w = cos(ψ)*cb_log_Fx[cb] - sin(ψ)*cb_log_Fy[cb]
    Fy_w = sin(ψ)*cb_log_Fx[cb] + cos(ψ)*cb_log_Fy[cb]
    # Scale arrow length to be visible on the auto-fit map (longer
    # transits use larger axis extents, so larger scale).
    arrow_scale = max(1e-3, 0.04 * (MAP_XLIMS[2] - MAP_XLIMS[1]) / 4e5)
    plot!(p_map, [px, px + arrow_scale*Fx_w], [py, py + arrow_scale*Fy_w];
          arrow = :head, c = :purple, lw = 2, label = "")

    # Wind arrow in the corner. Scale relative to the map extent.
    _map_w = MAP_XLIMS[2] - MAP_XLIMS[1]
    _map_h = MAP_YLIMS[2] - MAP_YLIMS[1]
    wcx, wcy = MAP_XLIMS[2] - 0.08*_map_w, MAP_YLIMS[2] - 0.12*_map_h
    wθ = deg2rad(wd); L = 0.05 * _map_w
    wlen = L * clamp(0.4 + ws / 20, 0.4, 1.4)
    plot!(p_map, [wcx, wcx - wlen*sin(wθ)], [wcy, wcy - wlen*cos(wθ)];
          arrow = :head, c = :royalblue, lw = 2, label = "")
    annotate!(p_map, wcx, wcy - 0.12*_map_h,
              text("wind $(round(ws; digits=1)) m/s\nfrom $(round(Int, mod(wd, 360)))°",
                   :royalblue, :center, 7))

    # Start + target markers.
    scatter!(p_map, [0.0], [0.0]; ms = 5, c = :black, label = "")
    scatter!(p_map, [TARGET_X], [TARGET_Y]; ms = 9, c = :gold, marker = :star5, label = "")

    # ─── Bottom panel: WaterLily vorticity field ────────────────────────
    ω_field = cb_log_vort[k]'
    title_bot = "WaterLily z-vorticity  |  " *
                "U_app = $(round(cb_log_U[cb]; digits=1)) m/s  |  " *
                "α = $(round(cb_log_a[cb]; digits=2)) rad  |  " *
                "ξ = $(round(ξ_k; digits=2))  |  " *
                "Cl_live = $(round(cb_log_Cl[cb]; digits=2))  |  " *
                "Cd_live = $(round(cb_log_Cd[cb]; digits=2))"
    p_cfd = heatmap(ω_field;
                    c = :RdBu, clims = (-8, 8),
                    aspect_ratio = :equal,
                    title = title_bot, titlefontsize = 9,
                    xlabel = "x [grid cells]  (inflow →)", ylabel = "y [grid cells]",
                    colorbar = false)

    plot(p_map, p_cfd; layout = grid(2, 1, heights = [0.55, 0.45]),
         size = (1200, 700))
end
mp4_path = joinpath(ASSETS, "flettner_cosim_animation_$(TAG).mp4")
mp4(anim, mp4_path; fps = fps)
println("  wrote $mp4_path")
