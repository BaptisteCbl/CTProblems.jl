EXAMPLE=(:orbital_transfert, :energy, :x_dim_4, :u_dim_2, :lagrange)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "Orbital transfert - energy minimisation - min ∫ ‖u‖² dt"

    # the model
    n=4
    m=2

    x0     = [-42272.67, 0, 0, -5796.72]
    μ      = 5.1658620912*1.0e12
    rf     = 42165.0 ;
    rf3    = rf^3  ;
    m0     = 2000.0
    F_max  = 100.0
    γ_max  = F_max*3600.0^2/(m0*10^3)
    t0     = 0.0
    tf     = 20.0 
    α      = sqrt(μ/rf3);

    ocp = Model()
    state!(ocp, n, [ "x" * ctindices(1), "x" * ctindices(2), "v" * ctindices(1), "v" * ctindices(2)])   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])
    constraint!(ocp, :initial, x0, :initial_constraint)
    constraint!(ocp, :boundary, (t0, x0, tf, xf) -> [norm(xf[1:2])-rf, xf[3] + α*xf[2], xf[4] - α*xf[1]], [0,0,0], :boundary_constraint)
    A = [ 0 0 1 0; 0 0 0 1; 1 0 0 0; 0 1 0 0]
    B = [ 0 0; 0 0; 1 0; 0 1 ]

    constraint!(ocp, :dynamics, (x, u) -> A*([-μ*x[1]/(sqrt(x[1]^2 + x[2]^2)^3);-μ*x[2]/(sqrt(x[1]^2 + x[2]^2)^3);x[3];x[4]]) + B*u)
    objective!(ocp, :lagrange, (x,u) -> 0.5*(u[1]^2 + u[2]^2)) # default is to minimise

    # the solution

    x0 = [x0;0]

    function control(p)
        u = zeros(eltype(p),2)
        u = [p[3], p[4]]
        return u
    end;

    function H(x, p)
        u = control(p)
        h = - 0.5*(u[1]^2 + u[2]^2) + p[1]*x[3] + p[2]*x[4] + p[3]*(-μ*x[1]/norm(x[1:2])^3 + u[1]) + p[4]*(-μ*x[2]/(sqrt(x[1]^2+x[2]^2))^3 + u[2]) + p[5]*0.5*(u[1]^2 + u[2]^2)
        return h
    end

    abstol=1e-12
    reltol=1e-12
    f = Flow(Hamiltonian(H), abstol=abstol, reltol=reltol)

    # shoot function
    function shoot(p0)
        
        s = zeros(eltype(p0), 5)
        xf, pf = f(t0,x0,p0,tf)
        s[1] = sqrt(xf[1]^2 + xf[2]^2) - rf
        s[2] = xf[3] + α*xf[2]
        s[3] = xf[4] - α*xf[1]
        s[4] = xf[2]*(pf[1]+α*pf[4]) - xf[1]*(pf[2]-α*pf[3])
        s[5] = pf[5]
        return s
    
    end;

    # Solve
    S(ξ) = shoot(ξ[1:5])
    jS(ξ) = ForwardDiff.jacobian(S, ξ)
    S!(s, ξ) = ( s[:] = S(ξ); nothing )
    jS!(js, ξ) = ( js[:] = jS(ξ); nothing )

    # Initial guess
    ξ_guess = [131.44483633582556, 34.16617425832973, 249.1573527073169, -23.9732920325726, 0]   # pour F_max = 100N

    #=
    # Solve
    indirect_sol = fsolve(S!, jS!, ξ_guess, show_trace=true, tol=1e-8); println(indirect_sol)
    
    # Retrieves solution
    if indirect_sol.converged
        ξ_sol = indirect_sol.x
    else
        error("Not converged")
    end
    #
    p0 = ξ_sol[1:5]
    =#

    p0 = ξ_guess

    # computing x, p, u
    ode_sol  = f((t0, tf), x0, p0)
    
    x(t) = ode_sol(t)[1:4]
    p(t) = ode_sol(t)[6:9]
    u(t) = control(p(t))
    objective =  ode_sol(tf)[5]

    #
    N=201
    times = range(t0, tf, N)
    #
    sol = OptimalControlSolution() #n, m, times, x, p, u)
    sol.state_dimension = n
    sol.control_dimension = m
    sol.times = times
    sol.state = x
    sol.state_names = [ "x" * ctindices(1), "x" * ctindices(2), "v" * ctindices(1), "v" * ctindices(2)]
    sol.adjoint = p
    sol.control = u
    sol.control_names = [ "u" * ctindices(i) for i ∈ range(1, m)]
    sol.objective = objective
    sol.iterations = 0
    sol.stopping = :dummy
    sol.message = "structure: smooth"
    sol.success = true
    sol.infos[:resolution] = :numerical

    #
    return OptimalControlProblem(title, ocp, sol)

end