EXAMPLE=(:integrator, :dim2, :energy, :state_constraint)

@eval function OptimalControlProblem{EXAMPLE}()
    # should return an OptimalControlProblem{example} with a message, a model and a solution

    # 
    msg = "Double integrator - energy min - state constraint"

    # the model
    n=2
    m=1
    t0=0
    tf=1
    x0=[0, 1]
    xf=[0, -1]
    l = 1/9
    ocp = Model()
    state!(ocp, n, ["x","v"])   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])
    constraint!(ocp, :initial, x0)
    constraint!(ocp, :final,   xf)
    constraint!(ocp, :state, Index(1), -Inf, l)
    A = [ 0 1
        0 0 ]
    B = [ 0
        1 ]
    constraint!(ocp, :dynamics, (x, u) -> A*x + B*u)
    objective!(ocp, :lagrange, (x, u) -> 0.5u^2) # default is to minimise

    # the solution (case l ≤ 1/6 because it has 3 arc)
    arc(t) = [0 ≤ t ≤ 3*l, 3*l < t ≤ 1 - 3*l, 1 - 3*l < t ≤ 1]
    x(t) = arc(t)[1]*[l*(1-(1-t/(3*l)))^3, (1-t/(3*l))^2] + arc(t)[2]*[l,0] + arc(t)[3]*[l*(1-(1-(1-t)/(3*l)))^3, -(1-(1-t)/(3*l))^2]
    u(t) = arc(t)[1]*(-2/(3l)*(1-t/(3*l))) + arc(t)[2]*0 + arc(t)[3]*(-2/(3l)*(1-(1-t)/(3*l)))
    p(t) = (0 ≤ t ≤ 3*l)*[2/9*l^2, 2/(3*l)*(1-t/(3*l))] +(3*l < t ≤ 1)*[-2/9*l^2, 2/(3*l)*(1-(1-t)/(3*l))]    
    objective = 4/(9*l)
    #
    N=201
    times = range(t0, tf, N)
    #
    sol = OptimalControlSolution() #n, m, times, x, p, u)
    sol.state_dimension = n
    sol.control_dimension = m
    sol.times = times
    sol.state = x
    sol.state_labels = [ "x" * ctindices(i) for i ∈ range(1, n)]
    sol.adjoint = p
    sol.control = u
    sol.control_labels = [ "u" ]
    sol.objective = objective
    sol.iterations = 0
    sol.stopping = :dummy
    sol.message = "analytical solution"
    sol.success = true

    #
    return OptimalControlProblem{EXAMPLE}(msg, ocp, sol)

end