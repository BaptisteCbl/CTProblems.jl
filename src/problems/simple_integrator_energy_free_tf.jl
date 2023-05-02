EXAMPLE=(:integrator, :energy, :free_final_time, :x_dim_1, :u_dim_1, :lagrange)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "simple integrator - energy min - free tf"

    # the model
    n=1
    m=1
    t0=0
    x0=0
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, :initial, t0)
    constraint!(ocp, :initial, x0, :initial_constraint)
    constraint!(ocp, :boundary, (t0, x0, tf, xf) -> xf-tf-10, 0, :boundary_constraint)
    constraint!(ocp, :dynamics, (x, u) -> u)
    objective!(ocp, :lagrange, (x, u) -> 0.5u^2) # default is to minimise

    # the solution
    tf = 10
    x(t) = t*(tf+10)/tf
    p(t) = (tf+10)/tf
    u(t) = p(t)
    objective = 1/2 * ((tf+10)^2)/tf
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
    sol.message = "structure: smooth"
    sol.success = true
    sol.infos[:resolution] = :analytical

    #
    return OptimalControlProblem(title, ocp, sol)

end