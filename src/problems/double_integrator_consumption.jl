EXAMPLE=(:integrator, :consumption, :state_dim_2, :control_dim_1, :lagrange, :control_constraint, :control_non_differentiable)

@eval function OCPDef{EXAMPLE}()
    # 
    title = "Double integrator consumption - mininimise ∫ |u| under the constraint |u| ≤ γ"

    # the model
    n=2
    m=1
    t0=0
    tf=1
    x0=[-1, 0]
    xf=[0, 0]
    γ = 5
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])
    constraint!(ocp, :initial, x0, :initial_constraint)
    constraint!(ocp, :final,   xf, :final_constraint)
    A = [ 0 1
        0 0 ]
    B = [ 0
        1 ]
    constraint!(ocp, :dynamics, (x, u) -> A*x + B*u)
    constraint!(ocp, :control, -γ, γ, :control_constraint)
    objective!(ocp, :lagrange, (x, u) -> abs(u)) # default is to minimise

    # the solution
    a = x0[1]
    b = x0[2]

    t1(α,β) = (β-1)/α
    t2(α,β) = (β+1)/α

    t1(p0) = t1(p0[1],p0[2])
    t2(p0) = t2(p0[1],p0[2])

    # arc 1
    x_arc_1(t,α,β) = [ a + b*t + 0.5*γ*t^2, b + γ*t]
    x_arc_1(t, p0) = x_arc_1(t, p0[1], p0[2])

    c(α,β) = x_arc_1(t1(α,β),α,β)[1]
    d(α,β) = x_arc_1(t1(α,β),α,β)[2] 

    # arc 2
    x_arc_2(t,α,β) = [c(α,β)+ d(α,β)*(t-t1(α,β)), d(α,β)]
    x_arc_2(t, p0) = x_arc_2(t, p0[1], p0[2])

    e(α,β) = x_arc_2(t2(α,β),α,β)[1]
    f(α,β) = x_arc_2(t2(α,β),α,β)[2]

    # arc 3
    fp(α,β) = f(α,β) + γ*t2(α,β)
    ep(α,β) = e(α,β) - fp(α,β)*t2(α,β) + γ/2*t2(α,β)^2
    x_arc_3(t,α,β) = [ep(α,β) + fp(α,β)*t - γ/2*t^2, fp(α,β) - γ*t]
    x_arc_3(t, p0) = x_arc_3(t, p0[1], p0[2])

    g(α,β) = x_arc_3(tf,α,β)[1]
    h(α,β) = x_arc_3(tf,α,β)[2]

    # solve
    #S(α,β) = [g(α,β),h(α,β)] - xf
    function shoot!(s,α,β)
        s[1] = g(α,β) - xf[1]
        s[2] = h(α,β) - xf[2]
    end

    #using MINPACK
    p0_ini = [4.472135954998979, 2.2360679774998067]#[1.5*2/tf, 1.5]
    ξ = [p0_ini[1],p0_ini[2]]
    nle = (s, ξ) -> shoot!(s, ξ[1], ξ[2])
    indirect_sol = fsolve(nle, ξ, show_trace = false)
    #println(indirect_sol)
     
    # using the result of the newton method ([4.472135954998979, 2.2360679774998067])
    p0 = indirect_sol.x
    x(t) = (t ≤ t1(p0))*x_arc_1(t,p0) + (t1(p0) < t < t2(p0))*x_arc_2(t,p0) + (t ≥ t2(p0))*x_arc_3(t,p0)
    p(t) = [p0[1],-p0[1]*t + p0[2]]
    u(t) = (t ≤ t1(p0))*γ + (t1(p0) < t < t2(p0))*0 + (t ≥ t2(p0))*(-γ)
    objective = γ*(t1(p0) + tf - t2(p0))

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
    sol.message = "structure: B+B0B-"
    sol.success = true
    sol.infos[:resolution] = :numerical

    #
    return OptimalControlProblem(title, ocp, sol)

end