EXAMPLE=(:integrator, :time, :x_dim_2, :u_dim_1, :mayer, :u_cons)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "Double integrator time - minimise tf under the constraint |u| ≤ γ"

    # the model
    n=2
    m=1
    t0=0
    x0=[-1, 0]
    xf=[0, 0]
    γ = 1
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, :initial, t0)
    constraint!(ocp, :initial, x0, :initial_constraint)
    constraint!(ocp, :final,   xf, :final_constraint)
    constraint!(ocp, :control, -γ, γ, :u_cons)
    A = [ 0 1
        0 0 ]
    B = [ 0
        1 ]
    constraint!(ocp, :dynamics, (x, u) -> A*x + B*u)
    objective!(ocp, :mayer,  (t0, x0, tf, xf) -> tf, :min) 

    # the solution
    a = x0[1]
    b = x0[2]

    t1 = sqrt(-a)
    tf = 2*t1
    α = 1/t1
    β = 1
    p(t) = [α, -α*t + β]
    u(t) = (t<t1)*γ + (t1≤t)*(-γ)
    x2_arc1(t) = t*γ + b
    x2_arc2(t) = (tf-t)*γ + xf[2] 
    x(t) = (t<t1)*[ 0.5*x2_arc1(t)^2 + a, x2_arc1(t)] + (t1≤t)*[-0.5*x2_arc2(t)^2, x2_arc2(t)]
    objective = tf

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
    sol.message = "structure: B+B-"
    sol.success = true
    sol.infos[:resolution] = :analytical

    #
    return OptimalControlProblem(title, ocp, sol)

end