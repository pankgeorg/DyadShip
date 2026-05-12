#!/usr/bin/env julia
#
# Run WaterLily.jl simulations of a rotating cylinder in cross-flow over a sweep of
# spin ratios ξ = ω·R / U, and write the resulting (Cl, Cd) table to
# `assets/flettner_coeffs.csv`. The Dyad `FlettnerRotor` component reads this CSV
# at simulation time via two `BlockComponents.Tables.Interpolation` blocks (one per
# coefficient), so re-running this script and refreshing the CSV is the way to
# update the rotor's physics with newer / higher-resolution CFD data.
#
# This is the offline characterization half of "CFD that interfaces with Dyad":
# WaterLily can't run at every ODE rhs evaluation (per-step cost is many ms,
# Dyad solves O(2000) seconds with hundreds of rhs calls per simulated second),
# so we precompute the dimensionless force coefficients once and Dyad uses the
# table live. Same pattern used in actual Flettner rotor design (Norsepower etc.).
#
# Closely modelled on G. D. Weymouth's `WaterLily.jl_CPC_2024/jl/examples/SpinCylOptim.jl`
# (https://github.com/WaterLily-jl/WaterLily.jl_CPC_2024), which is the canonical
# WaterLily spinning-cylinder example.
#
# Usage: `../julia-dyad.sh scripts/run_waterlily_flettner.jl`
#        (use this repo's dyad julia, but with the scripts/ local environment
#         which pulls in WaterLily + StaticArrays)
#
# Tunables at the bottom: ξ sweep, Re, resolution, time horizon.

using Pkg
Pkg.activate(@__DIR__)
Pkg.instantiate()

using WaterLily
using StaticArrays
using Printf

rot(θ) = SA[cos(θ) -sin(θ); sin(θ) cos(θ)]

"""
    flettner_sim(ξ; n, Re, U, t_end)

2D simulation of a circular cylinder of diameter D = n/4 grid cells, rotating
at angular speed ω = ξ·U/R inside a uniform U inflow domain of size (6n, n).

After `t_end` convective time units the drag (along inflow) and lift (transverse)
are read off `WaterLily.total_force`, time-averaged over the second half of the run.

Returns `(Cl, Cd)` normalized by `0.5·U²·D` (2D coefficients, per unit span).
"""
function flettner_sim(ξ; n=2^7, Re=1.0e4, U=1.0f0, t_end=8.0f0, T=Float32, verbose=false)
    D = T(n / 4)
    R = D / 2
    C = SA{T}[2D, n/2]
    sdf(x, t) = √sum(abs2, x - C) - R
    bodymap(x, t) = rot(ξ * U * t / R) * (x - C) + C
    body = AutoBody(sdf, bodymap)
    sim = Simulation((6n, n), (U, 0), D; ν=U*D/Re, body=body, T=T)

    nstep = 80
    Fs = zeros(T, 2, nstep)
    for k in 1:nstep
        t = t_end * k / nstep
        sim_step!(sim, t; remeasure=true)
        f = WaterLily.total_force(sim)
        Fs[1, k] = f[1]
        Fs[2, k] = f[2]
        verbose && @printf("  step %3d  t=%.2f  Cd=%+.3f  Cl=%+.3f\n",
                            k, t, 2f[1]/(U^2*D), 2f[2]/(U^2*D))
    end
    half = nstep ÷ 2
    F = sum(Fs[:, half+1:end], dims=2) / (nstep - half)
    Cd = Float64(2 * F[1] / (U^2 * D))
    Cl = Float64(2 * F[2] / (U^2 * D))
    return (Cl=Cl, Cd=Cd)
end

# ξ sweep: 0 → 6 in 0.5 steps up to ξ=4, then coarser to ξ=6 (plateau).
const XI_VALUES = Float32[0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0]

println("WaterLily Flettner rotor sweep — n=128, Re=1e4")
println("ξ       Cl       Cd")
results = NamedTuple{(:xi, :Cl, :Cd), Tuple{Float64,Float64,Float64}}[]
for ξ in XI_VALUES
    @printf("ξ=%.2f  running...\n", ξ)
    r = flettner_sim(ξ)
    push!(results, (xi=Float64(ξ), Cl=r.Cl, Cd=r.Cd))
    @printf("ξ=%.2f  Cl=%+.3f  Cd=%+.3f\n", ξ, r.Cl, r.Cd)
end

out = abspath(joinpath(@__DIR__, "..", "assets", "flettner_coeffs.csv"))
open(out, "w") do io
    println(io, "xi,Cl,Cd")
    for r in results
        @printf(io, "%.3f,%.4f,%.4f\n", r.xi, r.Cl, r.Cd)
    end
end
println()
println("Wrote $out")
println()
println("Paste into FlettnerRotor.dyad if you prefer inline-ifelse over CSV:")
println("# Cl(ξ):")
for i in 1:length(results)-1
    a, b = results[i], results[i+1]
    @printf("# %.3f ≤ ξ ≤ %.3f → Cl = %+.4f + %+.4f·(ξ-%.3f)\n",
            a.xi, b.xi, a.Cl, (b.Cl - a.Cl)/(b.xi - a.xi), a.xi)
end
