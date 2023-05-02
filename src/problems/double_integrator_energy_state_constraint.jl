EXAMPLE=(:integrator, :energy, :x_dim_2, :u_dim_1, :lagrange, :x_cons, :order_2)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "Double integrator energy - mininimise ∫ u² under the constraint x₁ ≤ l"

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
    constraint!(ocp, :initial, x0, :initial_constraint)
    constraint!(ocp, :final,   xf, :final_constraint)
    constraint!(ocp, :state, Index(1), -Inf, l, :x_cons)
    A = [ 0 1
        0 0 ]
    B = [ 0
        1 ]
    constraint!(ocp, :dynamics, (x, u) -> A*x + B*u)
    objective!(ocp, :lagrange, (x, u) -> 0.5u^2) # default is to minimise

    # the solution (case l ≤ 1/6 because it has 3 arc)
    arc(t) = [0 ≤ t ≤ 3*l, 3*l < t ≤ 1 - 3*l, 1 - 3*l < t ≤ 1]
    x(t) =  arc(t)[1]*[l*(1-(1-t/(3l))^3), (1-t/(3l))^2] + 
            arc(t)[2]*[l, 0] + 
            arc(t)[3]*[l*(1-(1-(1-t)/(3l))^3), -(1-(1-t)/(3l))^2]
    u(t) =  arc(t)[1]*(-2/(3l)*(1-t/(3l))) + 
            arc(t)[2]*0 + 
            arc(t)[3]*(-2/(3l)*(1-(1-t)/(3l)))
#    p(t) = arc(t)[1]*[2/9*l^2, 2/(3*l)*(1-t/(3*l))] + arc(t)[2]*[0, 0] + arc(t)[3]*[-2/9*l^2, 2/(3*l)*(1-(1-t)/(3*l))]    
    α = -18
    β = -6
    p(t) = arc(t)[1]*[α, -α*t+β] + arc(t)[2]*[0, 0] + arc(t)[3]*[-α, α*(t-2/3)]
    objective = 4/(9l)
    #
    N=201
    times = range(t0, tf, N)
    #
    sol = OptimalControlSolution() #n, m, times, x, p, u)
    sol.state_dimension = n
    sol.control_dimension = m
    sol.times = Base.deepcopy(times)
    sol.state = Base.deepcopy(x)
    sol.state_names = ["x", "v"]
    sol.adjoint = Base.deepcopy(p)
    sol.control = Base.deepcopy(u)
    sol.control_names = [ "u" ]
    sol.objective = objective
    sol.iterations = 0
    sol.stopping = :dummy
    sol.message = "structure: "
    sol.success = true
    sol.infos[:resolution] = :analytical

    #
    return OptimalControlProblem(title, ocp, sol)

end