EXAMPLE=(:lqr, :x_dim_2, :u_dim_1, :lagrange)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "lqr - dimension 2 - ricatti"

    # the model
    n=2
    m=1
    t0=0
    tf=5
    x0=[0, 1]
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])
    constraint!(ocp, :initial, x0, :initial_constraint)
    A = [0 1 ; -1 0]
    B = [0 ; 1]
    constraint!(ocp, :dynamics, (x, u) -> A*x + B*u)
    objective!(ocp, :lagrange, (x, u) -> 0.5*(x[1]^2 + x[2]^2 + u^2))

    # the solution
    Q = I
    R = I
    Rm1 = I
    a = x0[1]
    b = x0[2]

    # computing S
    ricatti(S, params, t) = -S*B*Rm1*B'*S - (-Q + A'*S + S*A)
    Sf = zeros(size(A))
    tspan = (tf, 0)
    prob = ODEProblem(ricatti,Sf,tspan)
    S = solve(prob, Tsit5(), reltol=1e-12, abstol=1e-12)

    # computing x
    dyn(x, params, t) = A*x + B*Rm1*B'*S(t)*x
    tspan = (0, tf)
    prob = ODEProblem(dyn,x0,tspan)
    x = solve(prob, Tsit5(), reltol=1e-8, abstol=1e-8)

    # computing u
    u(t) = Rm1*B'*S(t)*x(t) 
    
    # computing p
    ϕ(p, params, t) =  [p[2]+x(t)[1] ; x(t)[2]-p[1]]
    pf = [0;0]
    tspan = (tf, 0)
    prob = ODEProblem(ϕ,pf,tspan)
    p = solve(prob, Tsit5(), reltol=1e-8, abstol=1e-8)

    # computing objective
    ψ(c,params,t) = 0.5*(x(t)[1]^2 + x(t)[2]^2 + u(t)^2)
    tspan = (0,tf)
    c0 = 0
    prob = ODEProblem(ψ,0,tspan)
    obj = solve(prob, Tsit5(), reltol=1e-8, abstol=1e-8)
    objective = obj(tf)
    
    #
    N=201
    times = range(t0, tf, N)
    #
    sol = OptimalControlSolution() #n, m, times, x, p, u)
    sol.state_dimension = n
    sol.control_dimension = m
    sol.times = Base.deepcopy(times)
    sol.state = t -> x(t)
    sol.state_names = [ "x" * ctindices(i) for i ∈ range(1, n)]
    sol.adjoint = t -> p(t)
    sol.control = Base.deepcopy(u)
    sol.control_names = [ "u" ]
    sol.objective = objective
    sol.iterations = 0
    sol.stopping = :dummy
    sol.message = "structure: smooth"
    sol.success = true
    sol.infos[:resolution] = :numerical

    #
    return OptimalControlProblem(title, ocp, sol)

end