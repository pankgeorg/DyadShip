#!/usr/bin/env julia
# Run ShipTransient (clean) and ShipDisturbedTransient (wind + sea state) and
# produce diagnostic plots into ../assets/.
#
# Usage:  ../julia-dyad.sh scripts/check_autopilot.jl

using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

# Ensure Plots is available without permanently mutating Project.toml.
try
    @eval using Plots
catch
    Pkg.add("Plots")
    @eval using Plots
end

using DyadShip
using ModelingToolkit
gr()

const ASSETS = abspath(joinpath(@__DIR__, "..", "assets"))
mkpath(ASSETS)

println("Running ShipTransient (clean)…")
res_clean = DyadShip.Ship.ShipTransient()
sol_clean = res_clean.sol
sys_clean = res_clean.sys
println("  done. t ∈ [$(sol_clean.t[1]), $(sol_clean.t[end])], $(length(sol_clean.t)) samples.")

println("Running ShipDisturbedTransient (wind + sea state)…")
res_dist = DyadShip.Ship.ShipDisturbedTransient()
sol_dist = res_dist.sol
sys_dist = res_dist.sys
println("  done. t ∈ [$(sol_dist.t[1]), $(sol_dist.t[end])], $(length(sol_dist.t)) samples.")

function ts(sol, sys)
    shaft_w = sol[sys.shaft.w]
    src_tau = sol[sys.src.tau]      # the `tau` input signal (now scaled by pilot.throttle)
    thrust = sol[sys.prop.Thrust]
    u = sol[sys.hull.u]
    return (
        t = sol.t,
        pos_x = sol[sys.hull.pos_x],
        pos_y = sol[sys.hull.pos_y],
        psi = sol[sys.hull.psi],
        u = u,
        v = sol[sys.hull.v],
        vx_world = sol[sys.hull.vx_world],
        vy_world = sol[sys.hull.vy_world],
        rud = sol[sys.rudder.Rudder_position],
        cmd = sol[sys.pilot.rudder],
        thrust = thrust,
        rpm = sol[sys.prop.rpm],
        shaft_w = shaft_w,
        P_engine = src_tau .* shaft_w,
        P_thrust = thrust .* u,
    )
end

function disturbance(sol, sys)
    return (
        t  = sol.t,
        fx = sol[sys.swell_x.y],
        fy = sol[sys.swell_y.y],
        mz = sol[sys.swell_yaw.y],
        wx = sol[sys.wind.X_force],
        wy = sol[sys.wind.Y_force],
        wn = sol[sys.wind.N_moment],
    )
end

c = ts(sol_clean, sys_clean)
d = ts(sol_dist,  sys_dist)

target_x, target_y = 10000.0, 1000.0

# Wind config from the disturbed analysis (kept in sync with Ship_disturbed_analysis.dyad).
WIND_SPEED = 15.0
WIND_FROM_DEG = 45.0      # 0 = N(+Y), 90 = E(+X)
wind_θ = deg2rad(WIND_FROM_DEG)
wind_vec_world = (-WIND_SPEED * sin(wind_θ), -WIND_SPEED * cos(wind_θ))

# Apparent wind = wind − ship_velocity at the disturbed run's final sample.
# Use the body's actual world-frame velocity rather than finite-differencing position.
apparent_world = (wind_vec_world[1] - d.vx_world[end],
                  wind_vec_world[2] - d.vy_world[end])

p_map = plot(
    c.pos_x, c.pos_y; label = "clean", lw = 2, c = :steelblue,
    xlabel = "x [m] (East→)", ylabel = "y [m] (North↑)",
    title = "Track (autopilot toward (10000, 1000), 5 m/s)",
    aspect_ratio = :equal,
    legend = :bottomright,
)
plot!(p_map, d.pos_x, d.pos_y; label = "disturbed", lw = 2, c = :crimson)
scatter!(p_map, [0.0], [0.0]; label = "start", ms = 6, c = :black)
scatter!(p_map, [target_x], [target_y]; label = "target", ms = 8, c = :gold, marker = :star5)

# Place compass + wind glyphs in the upper-right of the current plot bounds.
# Use plot's current limits (after the data has been added) to avoid hard-coded coords.
xl = xlims(p_map)
yl = ylims(p_map)
glyph_x = xl[1] + 0.92 * (xl[2] - xl[1])
glyph_y = yl[1] + 0.92 * (yl[2] - yl[1])
# Length of arrows ~ 8% of plot width.
glyph_L = 0.08 * (xl[2] - xl[1])

# Compass rose (N up).
quiver!(p_map, [glyph_x], [glyph_y]; quiver = ([0.0], [glyph_L]),
    c = :black, lw = 2, label = "")
annotate!(p_map, glyph_x, glyph_y + 1.15 * glyph_L,
    text("N", :black, :center, 10))

# True wind: arrow pointing in the direction the wind BLOWS (= world-frame wind vector).
tw_x = glyph_x - 1.5 * glyph_L
tw_y = glyph_y
tw_dx = wind_vec_world[1] / WIND_SPEED * glyph_L
tw_dy = wind_vec_world[2] / WIND_SPEED * glyph_L
quiver!(p_map, [tw_x], [tw_y]; quiver = ([tw_dx], [tw_dy]),
    c = :royalblue, lw = 2, label = "")
annotate!(p_map, tw_x, tw_y + 0.4 * glyph_L,
    text("true wind\n$(Int(round(WIND_SPEED))) m/s from $(Int(round(WIND_FROM_DEG)))°",
         :royalblue, :center, 8))

# Apparent wind at the disturbed-run final sample (world frame, blowing direction).
ap_x = glyph_x - 1.5 * glyph_L
ap_y = glyph_y - 1.8 * glyph_L
ap_mag = sqrt(apparent_world[1]^2 + apparent_world[2]^2 + 1e-9)
ap_dx = apparent_world[1] / ap_mag * glyph_L
ap_dy = apparent_world[2] / ap_mag * glyph_L
quiver!(p_map, [ap_x], [ap_y]; quiver = ([ap_dx], [ap_dy]),
    c = :darkorange, lw = 2, label = "")
annotate!(p_map, ap_x, ap_y - 0.6 * glyph_L,
    text("apparent wind\n$(round(ap_mag; digits=1)) m/s (final)",
         :darkorange, :center, 8))

# Heading.
p_psi = plot(
    c.t, rad2deg.(c.psi); label = "clean", lw = 2, c = :steelblue,
    xlabel = "t [s]", ylabel = "heading ψ [deg]",
    title = "Heading vs time",
)
plot!(p_psi, d.t, rad2deg.(d.psi); label = "disturbed", lw = 2, c = :crimson)

# Rudder command and filtered position.
p_rud = plot(
    c.t, c.cmd; label = "command (clean)", lw = 1.5, c = :steelblue, ls = :dash,
    xlabel = "t [s]", ylabel = "rudder [deg]",
    title = "Rudder command and filtered position",
)
plot!(p_rud, c.t, c.rud; label = "position (clean)", lw = 2, c = :steelblue)
plot!(p_rud, d.t, d.cmd; label = "command (disturbed)", lw = 1.5, c = :crimson, ls = :dash)
plot!(p_rud, d.t, d.rud; label = "position (disturbed)", lw = 2, c = :crimson)

# Surge speed.
p_u = plot(
    c.t, c.u; label = "clean", lw = 2, c = :steelblue,
    xlabel = "t [s]", ylabel = "surge u [m/s]",
    title = "Surge speed",
)
plot!(p_u, d.t, d.u; label = "disturbed", lw = 2, c = :crimson)
hline!(p_u, [5.0]; label = "target", c = :gold, ls = :dot)

# Propulsion diagnostics.
p_prop = plot(
    c.t, c.thrust ./ 1e3; label = "thrust [kN] (clean)", lw = 2, c = :steelblue,
    xlabel = "t [s]", ylabel = "thrust [kN] / RPM",
    title = "Propeller thrust and shaft RPM",
)
plot!(p_prop, d.t, d.thrust ./ 1e3; label = "thrust [kN] (disturbed)", lw = 2, c = :crimson)
plot!(p_prop, c.t, c.rpm; label = "rpm (clean)", lw = 1.5, c = :steelblue, ls = :dash)
plot!(p_prop, d.t, d.rpm; label = "rpm (disturbed)", lw = 1.5, c = :crimson, ls = :dash)

# Disturbance traces.
dist = disturbance(sol_dist, sys_dist)
p_sea = plot(
    dist.t, dist.fx ./ 1e3; label = "Fx_swell [kN]", lw = 1.5,
    xlabel = "t [s]", ylabel = "force [kN] / moment [MN·m]",
    title = "Sea-state and wind disturbances",
    legend = :outertopright,
)
plot!(p_sea, dist.t, dist.fy ./ 1e3; label = "Fy_swell [kN]", lw = 1.5)
plot!(p_sea, dist.t, dist.mz ./ 1e6; label = "Mz_swell [MN·m]", lw = 1.5)
plot!(p_sea, dist.t, dist.wx ./ 1e3; label = "Fx_wind [kN]", lw = 1.5, ls = :dash)
plot!(p_sea, dist.t, dist.wy ./ 1e3; label = "Fy_wind [kN]", lw = 1.5, ls = :dash)
plot!(p_sea, dist.t, dist.wn ./ 1e6; label = "Mz_wind [MN·m]", lw = 1.5, ls = :dash)

# Power-velocity scatter (operating envelope).
p_pv = scatter(
    c.u, c.P_engine ./ 1e3; ms = 1.5, alpha = 0.4, c = :steelblue, label = "engine in (clean)",
    xlabel = "surge u [m/s]", ylabel = "power [kW]",
    title = "Power vs surge speed",
    legend = :topleft,
)
scatter!(p_pv, d.u, d.P_engine ./ 1e3; ms = 1.5, alpha = 0.4, c = :crimson, label = "engine in (disturbed)")
scatter!(p_pv, c.u, c.P_thrust ./ 1e3; ms = 1.5, alpha = 0.4, c = :navy, label = "thrust·u (clean)", marker = :diamond)
scatter!(p_pv, d.u, d.P_thrust ./ 1e3; ms = 1.5, alpha = 0.4, c = :firebrick, label = "thrust·u (disturbed)", marker = :diamond)

# Resistance vs speed using the HullMMG resistance polynomial
# R(u) = R3·u²·|u| + R2·u·|u| + R1·u  (fit to the upstream towing-tank curve).
u_grid = range(0, 15, 200)
R3 = 914.37; R2 = -5286.3; R1 = 19657.0
drag_mmg = @. R3 * u_grid^2 * abs(u_grid) + R2 * u_grid * abs(u_grid) + R1 * u_grid
p_drag = plot(u_grid, drag_mmg ./ 1e3; lw = 2, c = :steelblue, label = "HullMMG R(u)",
    xlabel = "surge u [m/s]", ylabel = "resistance [kN]",
    title = "Surge resistance — modeled",
    legend = :topleft,
)
hline!(p_drag, [122]; lw = 1.5, c = :gold, ls = :dash, label = "thrust @ ~73 RPM")
# Mark hull speed (1.34·√L_wl with L_wl ≈ Lpp in m).
hull_speed = 1.34 * sqrt(100 / 3.28)
vline!(p_drag, [hull_speed]; lw = 1, c = :purple, ls = :dot, label = "hull speed ≈ $(round(hull_speed; digits=1)) m/s")

# Composite figure.
p_all = plot(p_map, p_psi, p_rud, p_u, p_prop, p_sea, p_pv, p_drag;
    layout = @layout([a{0.5w} [b; c]; [d; e]; f; [g h]]),
    size = (1400, 1700),
)
savefig(p_all, joinpath(ASSETS, "autopilot_check.png"))
println("Wrote $(joinpath(ASSETS, "autopilot_check.png"))")

# Also save individual plots for closer inspection.
savefig(p_map, joinpath(ASSETS, "autopilot_map.png"))
savefig(p_rud, joinpath(ASSETS, "autopilot_rudder.png"))
savefig(p_psi, joinpath(ASSETS, "autopilot_heading.png"))
savefig(p_u,   joinpath(ASSETS, "autopilot_surge.png"))
savefig(p_sea, joinpath(ASSETS, "autopilot_disturbances.png"))
savefig(p_prop, joinpath(ASSETS, "autopilot_propulsion.png"))
savefig(p_pv,   joinpath(ASSETS, "autopilot_power_velocity.png"))
savefig(p_drag, joinpath(ASSETS, "autopilot_resistance.png"))
println("Wrote individual plots to $(ASSETS).")

# Quick numerical summary.
function summary(label, t)
    final_pos = (t.pos_x[end], t.pos_y[end])
    miss = sqrt((target_x - final_pos[1])^2 + (target_y - final_pos[2])^2)
    rms_cmd = sqrt(sum(t.cmd .^ 2) / length(t.cmd))
    println(" $label  final_pos=$(round.(final_pos; digits=1))  miss=$(round(miss; digits=1)) m   surge=$(round(t.u[end]; digits=2)) m/s   rms(cmd)=$(round(rms_cmd; digits=2))°   thrust=$(round(t.thrust[end]/1e3; digits=1)) kN   rpm=$(round(t.rpm[end]; digits=1))   thrust@5s=$(round(t.thrust[searchsortedfirst(t.t, 5.0)]/1e3; digits=1)) kN")
end

println("\nSummary:")
summary("clean    ", c)
summary("disturbed", d)
