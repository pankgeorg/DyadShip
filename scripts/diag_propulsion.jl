# Quick propulsion diagnostic: run ShipTransient, print thrust/rpm/surge at key times.
using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
using DyadShip
using Printf

res = DyadShip.Ship.ShipTransient()
sol = res.sol
sys = res.sys

# Time points to sample.
ts = [0.5, 1.0, 5.0, 10.0, 30.0, 60.0, 120.0, 200.0]
println("t [s] | shaft.w | prop.rpm | prop.Thrust [kN] | hull.u | hull.psi | hull.pos_x")
for τ in ts
    i = searchsortedfirst(sol.t, τ)
    i = clamp(i, 1, length(sol.t))
    @printf "%6.1f | %7.3f | %8.2f | %12.2f | %6.3f | %7.4f | %10.2f\n" sol.t[i] sol[sys.shaft.w][i] sol[sys.prop.rpm][i] sol[sys.prop.Thrust][i]/1e3 sol[sys.hull.u][i] sol[sys.hull.psi][i] sol[sys.hull.pos_x][i]
end
