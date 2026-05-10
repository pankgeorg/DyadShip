#!/usr/bin/env julia
# Render a sped-up animation of the closed-loop ship analysis.
#
# Output: assets/ship_animation.mp4 — ~30 seconds of video covering the
# 2200-second simulation (≈73× real-time speedup).
#
# The MultibodyComponents Render extension exists but its 3D meshes don't
# render reliably in headless WSL2 (GLMakie has no GL context, CairoMakie
# silently drops the mesh path). Instead we drive a 2D Plots.jl animation
# from the simulated trajectory: a ship glyph (rotated triangle) moves
# along the world-frame track, leaving a fading trail, with a compass +
# wind arrows in the corner.
#
# Usage: ../julia-dyad.sh scripts/render_ship.jl

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

println("Running ShipRenderTransient (2200 s sim)…")
res = DyadShip.Ship.ShipRenderTransient()
sol = res.sol
sys = res.sys
println("  done. t ∈ [$(sol.t[1]), $(sol.t[end])], $(length(sol.t)) samples.")

# Resample onto a uniform time grid for the animation.
fps = 30
duration_s = 30.0                       # video length
sim_duration = sol.t[end] - sol.t[1]
timescale = sim_duration / duration_s   # ≈ 73× speedup
nframes = Int(round(duration_s * fps))
sample_times = range(sol.t[1], sol.t[end]; length = nframes)

pos_x = [sol(t; idxs = sys.hull.pos_x) for t in sample_times]
pos_y = [sol(t; idxs = sys.hull.pos_y) for t in sample_times]
psi = [sol(t; idxs = sys.hull.psi) for t in sample_times]
thrust_kn = [sol(t; idxs = sys.prop.Thrust) / 1e3 for t in sample_times]
rudder = [sol(t; idxs = sys.rudder.Rudder_position) for t in sample_times]

target_x, target_y = 10000.0, 1000.0

# Wind config from Ship_disturbed_analysis.dyad (constant in this clean run).
WIND_SPEED = 15.0
WIND_FROM_DEG = 45.0
wind_θ = deg2rad(WIND_FROM_DEG)
wind_dx, wind_dy = -sin(wind_θ), -cos(wind_θ)   # direction wind blows toward

# Triangle representing the ship — pointing along its body +x. Scale so it's
# visible at world extent ~10000 m.
ship_len = 600.0
ship_half_beam = 200.0
ship_local = [
    ( ship_len*0.5,  0.0),
    (-ship_len*0.5,  ship_half_beam),
    (-ship_len*0.5, -ship_half_beam),
]

println("Rendering $nframes frames at $(fps) fps ($(duration_s) s video, $(round(timescale; digits=1))× speed)…")

anim = @animate for k in 1:nframes
    px = pos_x[k]
    py = pos_y[k]
    ψ = psi[k]

    # Track up to current time, fading trail.
    p = plot(
        pos_x[1:k], pos_y[1:k];
        label = "track", lw = 2, c = :steelblue,
        xlabel = "x [m] (East→)", ylabel = "y [m] (North↑)",
        title = "Ship at t = $(round(sample_times[k]; digits = 0)) s   |   thrust = $(round(thrust_kn[k]; digits = 0)) kN   |   rudder = $(round(rudder[k]; digits = 1))°",
        aspect_ratio = :equal,
        xlims = (-1000, 11500),
        ylims = (-1500, 3500),
        legend = false,
    )

    # Ship triangle, rotated by psi and translated to (px, py).
    ship_world_x = [px + cos(ψ)*lx - sin(ψ)*ly for (lx, ly) in ship_local]
    ship_world_y = [py + sin(ψ)*lx + cos(ψ)*ly for (lx, ly) in ship_local]
    plot!(p, [ship_world_x; ship_world_x[1]], [ship_world_y; ship_world_y[1]];
          seriestype = :shape, c = :crimson, lw = 1, fillalpha = 0.7, label = "")

    # Start, target.
    scatter!(p, [0.0], [0.0]; ms = 5, c = :black, label = "")
    scatter!(p, [target_x], [target_y]; ms = 9, c = :gold, marker = :star5, label = "")

    # Compass + wind glyphs in the upper-right corner of the plot.
    cx, cy = 10500.0, 3000.0
    L = 400.0
    plot!(p, [cx, cx], [cy, cy + L]; arrow = :head, c = :black, lw = 2, label = "")
    annotate!(p, cx, cy + L * 1.25, text("N", :black, :center, 9))
    plot!(p, [cx - 1.5*L, cx - 1.5*L + L*wind_dx], [cy, cy + L*wind_dy];
          arrow = :head, c = :royalblue, lw = 2, label = "")
    annotate!(p, cx - 1.5*L, cy - 0.4*L,
              text("wind 15 m/s\nfrom 45°", :royalblue, :center, 8))
end

mp4_path = joinpath(ASSETS, "ship_animation.mp4")
mp4(anim, mp4_path; fps = fps)
println("Wrote $mp4_path")
