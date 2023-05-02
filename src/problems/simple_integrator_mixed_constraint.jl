EXAMPLE=(:integrator, :x_dim_1, :u_dim_1, :lagrange, :mixed_constraint)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "simple integrator - mixed constraint"

    # the model
    n=1
    m=1
    t0=0
    tf=1
    x0=-1
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])
    constraint!(ocp, :initial, x0, :initial_constraint)
    #constraint!(ocp, :control, 0, Inf, :u_cons)
    constraint!(ocp, :mixed, (x,u) -> x + u, -Inf, 0, :mixed_constraint)
    constraint!(ocp, :dynamics, (x, u) -> u)
    objective!(ocp, :lagrange, (x, u) -> -u)

    # the solution
    x(t) = -exp(-t)
    p(t) = 1-exp(t-1)
    u(t) = -x(t)
    objective = exp(-1) - 1
    #
    N=201
    times = range(t0, tf, N)
    #
    sol = OptimalControlSolution() #n, m, times, x, p, u)
    sol.state_dimension = n
    sol.control_dimension = m
    sol.times = Base.deepcopy(times)
    sol.state = Base.deepcopy(x)
    sol.state_names = [ "x" ]
    sol.adjoint = Base.deepcopy(p)
    sol.control = Base.deepcopy(u)
    sol.control_names = [ "u" ]
    sol.objective = objective
    sol.iterations = 0
    sol.stopping = :dummy
    sol.message = "structure: smooth but the mixed constraint is active all over the solution"
    sol.success = true
    sol.infos[:resolution] = :analytical

    #
    return OptimalControlProblem(title, ocp, sol)

end
