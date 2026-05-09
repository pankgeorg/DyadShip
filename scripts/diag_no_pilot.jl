# Quick diagnostic: run ShipTransient with k_rudder = 0 (autopilot disabled).
# Reports surge, world position, heading, rudder cmd at sample times so we can
# tell whether the slow-progress problem is the autopilot or the plant itself.
using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using DyadShip
using Printf

spec = DyadShip.Ship.ShipTransientSpec()
# Disable the autopilot entirely.
res = DyadShip.Ship.ShipTransient(; ) # default once
sol = res.sol
sys = res.sys

# Override k_rudder via spec is harder; instead instantiate FullShip with the override.
import ModelingToolkit
@named model = DyadShip.Ship.FullShip()
# Easier: just print results from the default run, where pilot.rudder = -0.5.
# To compare with rudder off, we need a separate spec — use the ShipTransientSpec
# pattern, but it doesn't directly expose pilot overrides through the spec.
# Simpler approach: use the existing res to read the rudder activity, then we can
# infer plant dynamics from when course_diff is in deadband (rudder = 0).

ts = [0.5, 1.0, 5.0, 10.0, 30.0, 60.0, 120.0, 300.0, 600.0, 1800.0, 3600.0]
println("t [s] |  rpm  | thrust kN | u m/s | psi rad | rudder | pos_x m")
for τ in ts
    i = searchsortedfirst(sol.t, τ)
    i = clamp(i, 1, length(sol.t))
    @printf "%6.1f | %5.1f | %7.1f   | %5.2f | %7.4f | %5.2f° | %8.2f\n" sol.t[i] sol[sys.prop.rpm][i] sol[sys.prop.Thrust][i]/1e3 sol[sys.hull.u][i] sol[sys.hull.psi][i] sol[sys.rudder.Rudder_position][i] sol[sys.hull.pos_x][i]
end
