"""
    FlettnerCFDLiveExt

Package extension wiring `DyadShip.FlettnerCFDLive` to a real WaterLily.jl
2D rotating-cylinder simulation. Loaded automatically when both `DyadShip`
and `WaterLily` are present in the active environment (see Project.toml
`[weakdeps]` and `[extensions]` blocks).

The geometry is non-dimensional: domain `(6n, n)` cells, body diameter
`D = n/4`, unit inflow in +x. The body's spin ratio
`xi_ref[]` drives the rotation rate of the cylinder map. Each `step!`
call:

1. Updates `xi_ref[]` to the signed dimensionless spin ratio
   `ω · R / U_app` (sign carries the Magnus direction).
2. Advances the sim by `STATE.inner_dt` units of sim time.
3. Reads `WaterLily.total_force` (force-on-fluid convention), normalizes
   to engineering force-on-body Cl and Cd, then dimensionalizes using the
   physical air density / rotor radius / rotor height.
4. Decomposes Drag (along apparent wind) + Lift (perpendicular, sign
   carried by signed Cl) into the ship's body axes using `alpha`.

The body map's rotation phase = `xi_ref[] · U_sim · t / R_sim`. When
`xi_ref[]` changes between calls, the cylinder's *instantaneous tangential
surface velocity* (the only thing the BDIM boundary condition cares about)
updates correctly to the new ω. The visible rotation phase jumps too, but
for a circular cylinder this is geometrically invisible.
"""
module FlettnerCFDLiveExt

using DyadShip
using DyadShip: FlettnerCFDLive
using DyadShip.FlettnerCFDLive: STATE
using WaterLily
using StaticArrays

rot(θ) = SA[cos(θ) -sin(θ); sin(θ) cos(θ)]

function FlettnerCFDLive._init_impl!(; R::Real, H::Real, rho::Real = 1.29,
                                       n::Integer = 64, Re::Real = 1.0e4,
                                       inner_dt::Real = 0.5,
                                       mem = Array)
    D = Float32(n / 4)
    R_sim = D / 2
    C = SA{Float32}[2D, n / 2]
    U_sim = 1.0f0
    sdf = (x, t) -> √sum(abs2, x .- C) - R_sim
    # Initial ξ baked in as a literal `Float32` (not a `Ref`) so the body's
    # map closure is bitstype and survives the first `measure!` kernel that
    # WaterLily.Simulation runs on GPU. `_step_impl!` rebuilds the body
    # every callback with the fresh ξ, so this initial value is just a
    # placeholder.
    ξ0 = 0.0f0
    bodymap = (x, t) -> rot(ξ0 * U_sim * t / R_sim) * (x .- C) .+ C
    body = AutoBody(sdf, bodymap)
    # `mem` routes the flow-state arrays to CPU (`Array`, default) or GPU
    # (`CUDA.CuArray` when the caller imports CUDA and passes it through).
    # The SDF/map closures stay Julia and run inside WaterLily's
    # KernelAbstractions kernels on whichever backend `mem` selects.
    #
    # `uBC` is forced to a homogeneous `Tuple{Float32, Float32}`. CPU
    # kernels handle mixed-type tuples (`(1.0f0, 0)` ⇒ `Tuple{Float32, Int64}`)
    # but the CUDA-PTX backend can't compile the variadic `getindex` —
    # GPU `applyV!` errors with `ijl_get_nth_field_checked` unsupported.
    sim = Simulation((6n, n), (U_sim, 0.0f0), D; ν = U_sim * D / Re,
                     body, T = Float32, mem = mem)

    STATE.sim = sim
    STATE.R = R
    STATE.H = H
    STATE.rho = rho
    STATE.sim_t = 0.0
    STATE.inner_dt = inner_dt
    STATE.n_calls = 0
    STATE.Fx_body = 0.0
    STATE.Fy_body = 0.0
    STATE.Cl = 0.0
    STATE.Cd = 0.0

    @info "FlettnerCFDLive driver initialized" R H rho n Re inner_dt D D_sim=Float64(D)
    return STATE
end

function FlettnerCFDLive._step_impl!(t_dyad::Real, U_app::Real, alpha::Real, omega::Real)
    sim = STATE.sim
    sim === nothing &&
        error("FlettnerCFDLive driver not initialized. Call init!(R=..., H=...) first.")

    # Signed dimensionless spin ratio (sign carries the Magnus direction).
    U_floor = max(abs(U_app), 1.0e-3)
    ξ = Float32(omega * STATE.R / U_floor)

    # Rebuild the body each callback with the fresh ξ baked into the closure.
    # The Ref-update-from-outside approach didn't propagate through WaterLily's
    # cached BDIM weights — first verification run showed Cl pinned constant.
    # Body construction is cheap; the fluid state (flow.u, flow.p) persists, so
    # the boundary layer doesn't have to re-form between callbacks.
    D_sim = Float32(sim.L)
    R_sim = D_sim / 2
    n_y = Float32(size(sim.flow.p, 2))
    Cctr = SA{Float32}[2D_sim, n_y / 2]
    U_sim = 1.0f0
    sdf = (x, t) -> √sum(abs2, x .- Cctr) - R_sim
    bodymap = (x, t) -> rot(ξ * U_sim * t / R_sim) * (x .- Cctr) .+ Cctr
    sim.body = AutoBody(sdf, bodymap)

    # Advance the sim by `inner_dt` of sim time.
    STATE.sim_t += STATE.inner_dt
    WaterLily.sim_step!(sim, Float32(STATE.sim_t); remeasure = true)

    # `WaterLily.total_force` is force-on-fluid; negate to engineering convention.
    f = WaterLily.total_force(sim)
    D_sim_f = Float64(sim.L)
    U_sim_phys = 1.0  # we constructed with unit inflow
    Cd = -2 * Float64(f[1]) / (U_sim_phys^2 * D_sim_f)
    Cl = -2 * Float64(f[2]) / (U_sim_phys^2 * D_sim_f)
    STATE.Cd = Cd
    STATE.Cl = Cl

    # Dimensionalize to physical Newtons (per-span 2D coefficient × rotor height).
    q = 0.5 * STATE.rho * U_app^2
    A_proj = 2 * STATE.R * STATE.H   # diameter × height
    Drag = q * A_proj * Cd
    Lift = q * A_proj * Cl           # signed (carries Magnus direction)

    # Body-frame decomposition. Convention (see FlettnerRotor.dyad docstring):
    # `alpha` is `ApparentSpeedXY.AttackAngleSigned`, the angle of the body-relative-
    # to-wind velocity in body frame. Apparent wind direction in body frame is the
    # negation: (-cos(α), -sin(α)). Magnus lift is perpendicular to wind, with
    # sign carried by signed Cl (above).
    ex_wind = -cos(alpha)
    ey_wind = -sin(alpha)
    # Lift direction = +90° CCW rotation of wind direction in body frame.
    lx = -ey_wind   # = sin(alpha)
    ly =  ex_wind   # = -cos(alpha)

    Fx_body = Drag * ex_wind + Lift * lx
    Fy_body = Drag * ey_wind + Lift * ly

    STATE.Fx_body = Fx_body
    STATE.Fy_body = Fy_body
    STATE.n_calls += 1

    return (Fx_body, Fy_body)
end

end # module
