normL1(T, f) = sum( (T[2:end]-T[1:end-1]) .* [ abs(f(T[i])) for i ∈ 1:length(T)-1] ) 
normL2(T, f) = sqrt(sum( (T[2:end]-T[1:end-1]) .* [ abs(f(T[i]))^2 for i ∈ 1:length(T)-1] ))

function initial_condition(ocp::OptimalControlModel) # pre-condition: there is an x0
    x0 = nothing
    constraints = ocp.constraints
    for (_, c) ∈ constraints
        @match c begin
            (:initial , _, f          , v, w) => begin x0=v end
            (:final   , _, f          , _, _) => nothing
            (:boundary, _, f          , _, _) => nothing
            (:control , _, f::Function, _, _) => nothing
            (:control , _, rg         , _, _) => nothing
            (:state   , _, f::Function, _, _) => nothing
            (:state   , _, rg         , _, _) => nothing
            (:mixed   , _, f          , _, _) => nothing
	    end # match
    end # for
    return x0
end

function final_condition(ocp::OptimalControlModel) # pre-condition: there is a xf
    xf = nothing
    constraints = ocp.constraints
    for (_, c) ∈ constraints
        @match c begin
            (:initial , _, f          , _, _) => nothing
            (:final   , _, f          , v, w) => begin xf=v end
            (:boundary, _, f          , _, _) => nothing
            (:control , _, f::Function, _, _) => nothing
            (:control , _, rg         , _, _) => nothing
            (:state   , _, f::Function, _, _) => nothing
            (:state   , _, rg         , _, _) => nothing
            (:mixed   , _, f          , _, _) => nothing
	    end # match
    end # for
    return xf
end

function test_by_shooting(ocp, shoot!, ξ, fparams, sol, atol, title; 
            display=false, flow=:ocp, test_objective=true, objective=nothing)

    # solve
    # MINPACK needs a vector of Float64
    isreal = ξ isa Real
    shoot_sol = fsolve((s, ξ) -> isreal ? shoot!(s, ξ[1]) : shoot!(s, ξ), 
        Float64.(isreal ? [ξ] : ξ), 
        show_trace=display)
    display ? println(shoot_sol) : nothing
    ξ⁺ = isreal ? shoot_sol.x[1] : shoot_sol.x
    
    #
    T = sol.times # flow_sol.ode_sol.t
    n = sol.state_dimension
    m = sol.control_dimension

    x⁺ = nothing
    p⁺ = nothing
    u⁺ = nothing
    if flow == :ocp
        t0, x0, p0⁺, tf, f = fparams(ξ⁺) # compute optimal control solution    
        ocp⁺ = CTFlows.OptimalControlSolution(f((t0, tf), x0, p0⁺))
        x⁺ = t -> ocp⁺.state(t)
        p⁺ = t -> ocp⁺.adjoint(t)
        u⁺ = t -> ocp⁺.control(t)
    elseif flow == :hamiltonian
        t0, x0, p0⁺, tf, f, u = fparams(ξ⁺) # compute optimal control solution    
        z = f((t0, tf), x0, p0⁺)
        x⁺ = t -> z(t)[1:n]
        p⁺ = t -> z(t)[n+1:2n]
        u⁺ = t -> u(x⁺(t), p⁺(t))
    elseif flow == :noflow
        t0, x0, tf, x⁺, p⁺, u⁺ = fparams(ξ⁺)
    else
        error("flow must be :ocp, :hamiltonian or :noflow")
    end

    #
    @testset "$title" begin
        for i ∈ 1:n
            subtitle = "state " * string(i)
            @testset "$subtitle" begin
                @test normL2(T, t -> (x⁺(t)[i] - sol.state(t)[i]) ) ≈ 0 atol=atol
            end
        end
        for i ∈ 1:n
            subtitle = "adjoint " * string(i)
            @testset "$subtitle" begin
                @test normL2(T, t -> (p⁺(t)[i] - sol.adjoint(t)[i]) ) ≈ 0 atol=atol
            end
        end
        for i ∈ 1:m
            subtitle = "control " * string(i)
            @testset "$subtitle" begin
                @test normL2(T, t -> (u⁺(t)[i] - sol.control(t)[i]) ) ≈ 0 atol=atol
            end
        end
        if test_objective
            if !isnothing(objective)
                @testset "objective - perso" begin
                    @test objective(ξ⁺) ≈ sol.objective atol=atol
                end
            elseif !isnothing(ocp.mayer) && isnothing(ocp.lagrange)
                # Mayer case
                @testset "objective - mayer case" begin
                    @test ocp.mayer(t0, x0, tf, x⁺(tf)) ≈ sol.objective atol=atol
                end
            elseif isnothing(ocp.mayer) && !isnothing(ocp.lagrange)
                # Lagrange case
                @testset "objective - lagrange case" begin
                    ϕ(_, _, t) = [ocp.lagrange(t, x⁺(t), u⁺(t))]
                    tspan = (t0, tf)
                    x0 = [0.0]
                    prob = ODEProblem(ϕ, x0, tspan)
                    ode_sol = solve(prob, Tsit5(), reltol=1e-6, abstol=1e-6)
                    @test ode_sol(tf)[1] ≈ sol.objective atol=atol rtol=1e-5
                end
            elseif !isnothing(ocp.mayer) && !isnothing(ocp.lagrange)
                # Bolza case
                @testset "objective - bolza case" begin
                    ϕ(_, _, t) = [ocp.lagrange(t, x⁺(t), u⁺(t))]
                    tspan = (t0, tf)
                    x0 = [0.0]
                    prob = ODEProblem(ϕ, x0, tspan)
                    ode_sol = solve(prob, Tsit5(), reltol=1e-6, abstol=1e-6)
                    @test ocp.mayer(t0, x0, tf, x⁺(tf)) + ode_sol(tf)[1] ≈ sol.objective atol=atol rtol=1e-5
                end
            end
        end
    end

end