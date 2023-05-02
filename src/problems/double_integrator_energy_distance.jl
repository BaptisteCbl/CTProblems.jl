EXAMPLE=(:integrator, :energy, :distance, :x_dim_2, :u_dim_1, :bolza)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "Double integrator energy/distance - minimise -x₁ + ∫ u²"

    # the model
    n=2
    m=1
    t0=0
    tf=1
    x0=[0, 0]
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])
    constraint!(ocp, :initial, x0, :initial_constraint)
    A = [ 0 1
        0 0 ]
    B = [ 0
        1 ]
    constraint!(ocp, :dynamics, (x, u) -> A*x + B*u)
    objective!(ocp, :bolza, (t0, x0, tf, xf) -> -0.5xf[1], (x, u) -> 0.5u^2)

    # the solution
    a = x0[1]
    b = x0[2]
    α = 0.5
    β = tf/2
    x(t) = [a+ b*t + t^2/12*(3*tf-t), b + t/4*(2*tf-t)]
    p(t) = [α, -α*t+β]
    u(t) = p(t)[2]
    objective = -0.5*x(tf)[1] + (1/3)*(tf^3/8)
    #
    N=201
    times = range(t0, tf, N)
    #
    sol = OptimalControlSolution() #n, m, times, x, p, u)
    sol.state_dimension = n
    sol.control_dimension = m
    sol.times = Base.deepcopy(times)
    sol.state = Base.deepcopy(x)
    sol.state_names = [ "x" * ctindices(i) for i ∈ range(1, n)]
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