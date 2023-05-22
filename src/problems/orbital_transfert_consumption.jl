EXAMPLE=(:orbital_transfert, :consumption, :x_dim_4, :u_dim_2, :lagrange, :u_cons, :non_diff_wrt_u)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "Orbital transfert - consumption minimisation - ∫ ‖u‖ dt "

    # the model
    n=4
    m=2

    x0 = [-42272.67, 0, 0, -5796.72]
    μ      = 5.1658620912*1e12
    rf     = 42165.0 ;
    rf3    = rf^3  ;
    m0     = 2000.0
    F_max = 100.0
    γ_max  = F_max*3600.0^2/(m0*10^3)
    t0     = 0.0
    α      = sqrt(μ/rf3);
    β      = 0.0

    tol    = 1e-9;

    F_max_100  = 100.0

    tf_min = 13.40318195708344 # minimal time for Fmax = 100 N
    tf = 1.5*tf_min

    Th(F) = F*3600.0^2/(10^3)
    u_max = Th(F_max)

    ocp = Model()
    state!(ocp, n, [ "x" * ctindices(1), "x" * ctindices(2), "v" * ctindices(1), "v" * ctindices(2)])   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])
    constraint!(ocp, :initial, x0, :initial_constraint)
    constraint!(ocp, :boundary, (t0, x0, tf, xf) -> [norm(xf[1:2])-rf, xf[3] + α*xf[2], xf[4] - α*xf[1]],[0,0,0], :boundary_constraint)
    constraint!(ocp, :control, u -> u[1]^2 + u[2]^2, 0, 1, :u_cons)
    A = [ 0 0 1 0; 0 0 0 1; 1 0 0 0; 0 1 0 0]
    B = [ 0 0; 0 0; γ_max 0; 0 γ_max ]
    constraint!(ocp, :dynamics, (x, u) -> A*([-μ*x[1]/(norm(x[1:2])^3);-μ*x[2]/(norm(x[1:2])^3);x[3];x[4]]) + B*u)
    objective!(ocp, :lagrange, (x, u) -> norm(u[1:2])) # default is to minimise

    # the solution
    x0 = [x0; 0]

    u0(x,p) = [0, 0]
    u1(x,p) = p[3:4]/norm(p[3:4])
    
    Hc(x,p) = p[1]*x[3] + p[2]*x[4] + p[3]*(-μ*x[1]/norm(x[1:2])^3) + p[4]*(-μ*x[2]/norm(x[1:2])^3)
    H(x,p,u) = -norm(u) + Hc(x,p) + u[1]*p[3]*γ_max + u[2]*p[4]*γ_max + p[5]*norm(u)
    H0(x,p) = H(x,p,u0(x,p)) 
    H1(x,p) = H(x,p,u1(x,p))

    # Flow
    f0 = Flow(Hamiltonian(H0));
    f1 = Flow(Hamiltonian(H1));

    # shoot function
    function shoot(p0, t1, t2, t3, t4)
        
        s = zeros(eltype(p0),9)
        
        x1, p1 = f1(t0, x0, p0, t1)
        x2, p2 = f0(t1, x1, p1, t2)
        x3, p3 = f1(t2, x2, p2, t3)
        x4, p4 = f0(t3, x3, p3, t4)
        xf, pf = f1(t4, x4, p4, tf)
        
        s[1] = norm(xf[1:2]) - rf
        s[2] = xf[3] + α*xf[2]
        s[3] = xf[4] - α*xf[1]
        s[4] = xf[2]*(pf[1] + α*pf[4]) - xf[1]*(pf[2] - α*pf[3])
        s[5] = γ_max*(p1[3]^2 + p1[4]^2) - 1
        s[6] = γ_max*(p2[3]^2 + p2[4]^2) - 1
        s[7] = γ_max*(p3[3]^2 + p3[4]^2) - 1
        s[8] = γ_max*(p4[3]^2 + p4[4]^2) - 1
        s[9] = pf[5]
        return s
    
    end;

    # Solve
    S(ξ) = shoot(ξ[1:5], ξ[6],ξ[7],ξ[8],ξ[9])
    jS(ξ) = ForwardDiff.jacobian(S, ξ)
    S!(s, ξ) = ( s[:] = S(ξ); nothing )
    jS!(js, ξ) = ( js[:] = jS(ξ); nothing )

    # Initial guess
    p0_guess = [0.02698412111231433, 0.006910835140705538, 0.050397371862031096, -0.0032972040120747836, -1.0076835239866583e-23]
    ti_guess = [0.4556797711668658, 3.6289692721936913, 11.683607683450061, 12.505465498856514]
    ξ_guess  = [p0_guess;ti_guess]

    #=
    # Solve
    indirect_sol = fsolve(S!, jS!, ξ_guess, show_trace=true, tol=1e-8); println(indirect_sol)
    
    # Retrieves solution
    if indirect_sol.converged
        ξ_sol = indirect_sol.x
    else
        error("Not converged")
    end

    p0 = ξ_sol[1:5]
    t1,t2,t3,t4 = ξ_sol[6:9]
    =#

    p0 = p0_guess
    t1, t2, t3, t4 = ti_guess

    # computing x, p, u
    f = f1 * (t1, f0) * (t2, f1) * (t3, f0) * (t4, f1)
    ode_sol  = f((t0, tf), x0, p0)
    
    x(t) = ode_sol(t)[1:4]
    p(t) = ode_sol(t)[6:9]
    u(t) = [0,0]*(t ∈ Interval(t1,t2)∪Interval(t3,t4)) +
             p(t)[3:4]/norm(p(t)[3:4])*(t ∈ Interval(t0,t1)∪Interval(t2,t3)∪Interval(t4,tf))
    objective = ode_sol(tf)[5]
    
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
    sol.message = "structure: B+B0B+B0B+"
    sol.success = true
    sol.infos[:resolution] = :numerical

    #
    return OptimalControlProblem(title, ocp, sol)

end