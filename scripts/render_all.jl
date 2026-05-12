#!/usr/bin/env julia
# Render side-by-side comparison videos of the three ship-transit analyses:
#
#   1. `ShipRenderTransient`                  (diesel propeller only, baseline)
#   2. `ShipFlettnerRenderTransient`          (rotor + unfavorable 45° wind)
#   3. `ShipFlettnerFavorableRenderTransient` (rotor + beam wind, optimal)
#
# Each writes a 30-second MP4 into assets/. The rotor variants overlay a
# spinning circle on the ship's bow and a red rotor-force arrow so the
# Magnus contribution is visible. Same color scheme + axis limits + glyph
# sizes across all three so they can be eyeballed side-by-side.
#
# Usage: ../julia-dyad.sh scripts/render_all.jl

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

try
    @eval using Plots
catch
    Pkg.add("Plots")
    @eval using Plots
end

using DyadShip
gr()

const ASSETS = abspath(joinpath(@__DIR__, "..", "assets"))
mkpath(ASSETS)

const FPS = 30
const DURATION_S = 30.0
const TARGET_XY = (10000.0, 1000.0)

const SHIP_LEN = 600.0
const SHIP_HALF_BEAM = 200.0
const SHIP_LOCAL = [
    ( SHIP_LEN*0.5,  0.0),
    (-SHIP_LEN*0.5,  SHIP_HALF_BEAM),
    (-SHIP_LEN*0.5, -SHIP_HALF_BEAM),
]

# Rotor visualization: small circle on the ship's forward arm, rotation
# indicator line through it.
const ROTOR_CIRCLE_R = 100.0          # plotted radius (world m)
const ROTOR_ARM_BODY = (30.0, 0.0)    # body-frame mount offset (matches Dyad parameter)
const ROTOR_FORCE_SCALE = 8e-3        # m of arrow per N of force (300 kN ≈ 2400 m)

function rotor_glyph!(p, px, py, ψ, omega_phase; color=:purple)
    # mount position in world frame
    mx, my = ROTOR_ARM_BODY
    cx = px + cos(ψ)*mx - sin(ψ)*my
    cy = py + sin(ψ)*mx + cos(ψ)*my
    # circle
    θs = range(0, 2π; length=33)
    plot!(p, cx .+ ROTOR_CIRCLE_R .* cos.(θs), cy .+ ROTOR_CIRCLE_R .* sin.(θs);
          seriestype=:shape, c=color, lw=1, fillalpha=0.55, label="")
    # spin indicator: a chord whose angle is omega_phase (sped up so it's visible)
    plot!(p, [cx - ROTOR_CIRCLE_R*cos(omega_phase),
              cx + ROTOR_CIRCLE_R*cos(omega_phase)],
             [cy - ROTOR_CIRCLE_R*sin(omega_phase),
              cy + ROTOR_CIRCLE_R*sin(omega_phase)];
          c=:white, lw=2, label="")
end

function rotor_force_arrow!(p, px, py, ψ, Fx_body, Fy_body; color=:crimson)
    # body → world rotation, then scale.
    Fx_world = cos(ψ)*Fx_body - sin(ψ)*Fy_body
    Fy_world = sin(ψ)*Fx_body + cos(ψ)*Fy_body
    plot!(p, [px, px + ROTOR_FORCE_SCALE*Fx_world],
             [py, py + ROTOR_FORCE_SCALE*Fy_world];
          arrow=:head, c=color, lw=3, label="")
end

function render_config(; analysis_fn, title_prefix, output_basename,
                       has_rotor::Bool)
    println("─"^70)
    println("Running $analysis_fn …")
    res = analysis_fn()
    sol = res.sol
    sys = res.sys
    println("  done. t ∈ [$(sol.t[1]), $(sol.t[end])], $(length(sol.t)) samples.")

    sim_duration = sol.t[end] - sol.t[1]
    timescale = sim_duration / DURATION_S
    nframes = Int(round(DURATION_S * FPS))
    sample_times = range(sol.t[1], sol.t[end]; length = nframes)

    # Common signals.
    pos_x   = [sol(t; idxs = sys.hull.pos_x) for t in sample_times]
    pos_y   = [sol(t; idxs = sys.hull.pos_y) for t in sample_times]
    psi     = [sol(t; idxs = sys.hull.psi)   for t in sample_times]
    thrust  = [sol(t; idxs = sys.prop.Thrust) / 1e3 for t in sample_times]
    rud_pos = [sol(t; idxs = sys.rudder.Rudder_position) for t in sample_times]
    wspd    = [sol(t; idxs = sys.wind_speed_now) for t in sample_times]
    wdir    = [sol(t; idxs = sys.wind_direction_now) for t in sample_times]

    rotor_omega = nothing
    rotor_Fx = rotor_Fy = nothing
    rotor_phase = nothing
    if has_rotor
        rotor_omega = [sol(t; idxs = sys.rotor.Omega) for t in sample_times]
        rotor_Fx    = [sol(t; idxs = sys.rotor.Force_X) for t in sample_times]
        rotor_Fy    = [sol(t; idxs = sys.rotor.Force_Y) for t in sample_times]
        # Accumulated rotation phase, downscaled so it's eye-readable (real ω
        # ≈ 30 rad/s, way too fast for 30 fps — slow it down for visibility).
        VIS_SLOWDOWN = 80.0
        phase = 0.0
        rotor_phase = Float64[]
        prev_t = sample_times[1]
        for (k, t) in enumerate(sample_times)
            dt = t - prev_t
            phase += rotor_omega[k] * dt / VIS_SLOWDOWN
            push!(rotor_phase, phase)
            prev_t = t
        end
    end

    println("  rendering $nframes frames at $(FPS) fps …")
    anim = @animate for k in 1:nframes
        px, py, ψ = pos_x[k], pos_y[k], psi[k]
        ws, wd = wspd[k], wdir[k]

        extra_title = has_rotor ?
            "  |  rotor ω=$(round(rotor_omega[k]; digits=1)) rad/s" :
            ""

        p = plot(
            pos_x[1:k], pos_y[1:k];
            label = "track", lw = 2, c = :steelblue,
            xlabel = "x [m] (East→)", ylabel = "y [m] (North↑)",
            title = "$(title_prefix)  |  t = $(round(sample_times[k]; digits=0)) s  |  " *
                    "thrust $(round(thrust[k]; digits=0)) kN  |  " *
                    "rudder $(round(rud_pos[k]; digits=1))°  |  " *
                    "wind $(round(ws; digits=1)) m/s from $(round(Int, mod(wd, 360)))°" *
                    extra_title,
            titlefontsize = 8,
            aspect_ratio = :equal,
            xlims = (-1000, 11500),
            ylims = (-1500, 3500),
            legend = false,
        )

        # ship glyph
        sx = [px + cos(ψ)*lx - sin(ψ)*ly for (lx, ly) in SHIP_LOCAL]
        sy = [py + sin(ψ)*lx + cos(ψ)*ly for (lx, ly) in SHIP_LOCAL]
        plot!(p, [sx; sx[1]], [sy; sy[1]];
              seriestype = :shape, c = :crimson, lw = 1, fillalpha = 0.7, label = "")

        # rotor + its force arrow
        if has_rotor
            rotor_glyph!(p, px, py, ψ, rotor_phase[k])
            rotor_force_arrow!(p, px, py, ψ, rotor_Fx[k], rotor_Fy[k])
        end

        # start, target
        scatter!(p, [0.0], [0.0]; ms = 5, c = :black, label = "")
        scatter!(p, [TARGET_XY[1]], [TARGET_XY[2]]; ms = 9, c = :gold, marker = :star5, label = "")

        # corner: compass + wind arrow
        cx, cy = 10500.0, 3000.0
        L = 400.0
        plot!(p, [cx, cx], [cy, cy + L]; arrow = :head, c = :black, lw = 2, label = "")
        annotate!(p, cx, cy + L * 1.25, text("N", :black, :center, 9))

        wθ = deg2rad(wd)
        wind_dx, wind_dy = -sin(wθ), -cos(wθ)
        wlen = L * clamp(0.4 + ws / 20, 0.4, 1.4)
        plot!(p, [cx - 1.5*L, cx - 1.5*L + wlen*wind_dx],
                 [cy, cy + wlen*wind_dy];
              arrow = :head, c = :royalblue, lw = 2, label = "")
        annotate!(p, cx - 1.5*L, cy - 0.4*L,
                  text("wind $(round(ws; digits=1)) m/s\nfrom $(round(Int, mod(wd, 360)))°",
                       :royalblue, :center, 8))
    end

    out = joinpath(ASSETS, output_basename)
    mp4(anim, out; fps = FPS)
    println("  wrote $out")
    return out
end

# Three configs, rendered sequentially. They share precompile cost so doing
# them in one process is much faster than three invocations.
configs = [
    (analysis_fn = DyadShip.Ship.ShipRenderTransient,
     title_prefix = "Baseline (diesel only)",
     output_basename = "ship_animation.mp4",
     has_rotor = false),
    (analysis_fn = DyadShip.Ship.ShipFlettnerRenderTransient,
     title_prefix = "Flettner @ 45° wind (unfavorable)",
     output_basename = "ship_flettner_animation.mp4",
     has_rotor = true),
    (analysis_fn = DyadShip.Ship.ShipFlettnerFavorableRenderTransient,
     title_prefix = "Flettner @ N beam wind (favorable)",
     output_basename = "ship_flettner_favorable_animation.mp4",
     has_rotor = true),
]

for cfg in configs
    render_config(; cfg...)
end
println("Done.")
