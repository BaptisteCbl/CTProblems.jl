EXAMPLE=(:integrator, :energy, :x_dim_2, :u_dim_1, :lagrange, :noconstraints)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "Double integrator energy - minimise ∫ u²"

    # the model
    n=2
    m=1
    t0=0
    tf=1
    x0=[-1, 0]
    xf=[0, 0]
    t0=0
    tf=1
    x0=[-1, 0]
    xf=[0, 0]
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])

    constraint!(ocp, :initial, x0, :initial_constraint)
    constraint!(ocp, :final, xf, :final_constraint)
    A = [ 0 1
        0 0 ]
    B = [ 0
        1 ]
    constraint!(ocp, :dynamics, (x, u) -> A*x + B*u)
    objective!(ocp, :lagrange, (x, u) -> 0.5u^2) # default is to minimise

    # the solution
    a = x0[1]
    b = x0[2]
    C = [-(tf-t0)^3/6 (tf-t0)^2/2
         -(tf-t0)^2/2 (tf-t0)]
    D = [-a-b*(tf-t0), -b]+xf
    p0 = C\D
    α = p0[1]
    β = p0[2]
    x(t) = [a+b*(t-t0)+β*(t-t0)^2/2.0-α*(t-t0)^3/6.0, b+β*(t-t0)-α*(t-t0)^2/2.0]
    p(t) = [α, -α*(t-t0)+β]
    u(t) = p(t)[2]
    objective = 0.5*(α^2*(tf-t0)^3/3+β^2*(tf-t0)-α*β*(tf-t0)^2)
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
