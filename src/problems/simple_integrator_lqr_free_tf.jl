EXAMPLE=(:integrator, :lqr, :free_final_time, :x_dim_1, :u_dim_1, :bolza)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "simple integrator - bolza cost: tf + ½ ∫ x²+u²"

    # the model
    n=1
    m=1
    t0=0
    x0=0
    xf=1
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, :initial, t0)
    constraint!(ocp, :initial, x0, :initial_constraint)
    constraint!(ocp, :final, xf, :final_constraint)
    constraint!(ocp, :dynamics, (x, u) -> u)
    objective!(ocp, :bolza, (t0, x0, tf, xf) -> tf, (x, u) -> 0.5*(u^2+x^2)) # default is to minimise

    # the solution
    tf = atanh(sqrt(xf^2/(2+xf^2)))
    p0 = xf[1]/sinh(tf)
    x(t) = p0*sinh(t)
    p(t) = p0*cosh(t)
    u(t) = p(t)
    objective = tf + 0.5*xf^2*1/tanh(tf)
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