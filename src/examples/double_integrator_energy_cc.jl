EXAMPLE=(:integrator, :dim2, :energy , :constraint)

@eval function OptimalControlProblem{EXAMPLE}()
    # should return an OptimalControlProblem{example} with a message, a model and a solution

    # 
    msg = "Double integrator - energy min - control constraint"

    # the model
    n=2
    m=1
    t0=0.0
    tf=1.0
    x0=[-1.0, 0.0]
    xf=[0.0, 0.0]
    γ = 2.0
    ocp = Model()
    state!(ocp, n)   # dimension of the state
    control!(ocp, m) # dimension of the control
    time!(ocp, [t0, tf])
    constraint!(ocp, :initial, x0)
    constraint!(ocp, :final,   xf)
    A = [ 0.0 1.0
        0.0 0.0 ]
    B = [ 0.0
        1.0 ]
    constraint!(ocp, :dynamics, (x, u) -> A*x + B*u[1])
    constraint!(ocp, :control, u -> u, -γ, γ)
    objective!(ocp, :lagrange, (x, u) -> 0.5u[1]^2) # default is to minimise

    # the solution
    a = x0[1]
    b = x0[2]

    t1(α,β) = (β-γ)/α
    t2(α,β) = (β+γ)/α

    # arc 1
    x_arc_1(t,α,β) = [b + γ*t, a + b*t + 0.5*γ*t^2]

    c(α,β) = x_arc_1(t1(α,β),α,β)[1]
    d(α,β) = x_arc_1(t2(α,β),α,β)[2] 

    # arc 2
    dp(α,β) = d(α,β) - (-0.5*α*t1(α,β) + β*t1(α,β))
    cp(α,β) = c(α,β) - (dp(α,β)*t1(α,β) - α/6*t1(α,β)^3 + β/2*t1(α,β)^2)
    x_arc_2(t,α,β) = [cp(α,β) + (dp(α,β)*t  - α/6*t^3 + β/2*t^2), dp(α,β) - α/2*t^2 + β*t]

    e(α,β) = x_arc_2(t2(α,β),α,β)[1]
    f(α,β) = x_arc_2(t2(α,β),α,β)[2]

    # arc 3
    fp(α,β) = f(α,β) + γ*t2(α,β)
    ep(α,β) = e(α,β) - fp(α,γ)*t2(α,β) + γ/2*t2(α,β)^2
    x_arc_3(t,α,β) = [fp(α,β) - γ*t, ep(α,β) + fp(α,β)*t - γ/2*t^2]

    g(α,β) = x_arc_3(tf,α,β)[1]
    h(α,β) = x_arc_3(tf,α,β)[2]

    # solve
    #S(α,β) = [g(α,β),h(α,β)] - xf
    function shoot!(s,α,β)
        s[1] = g(α,β) - xf[1]
        s[2] = h(α,β) - xf[2]
        s
    end

    #using MINPACK
    p0_ini = [20.0, 10.0]
    ξ = [p0_ini[1],p0_ini[2]]
    nle = (s, ξ) -> shoot!(s, ξ[1], ξ[2])
    indirect_sol = fsolve(nle, ξ, show_trace = true)
    println(indirect_sol)
     
    # the result of the newton method is [11.740941920178892, 4.5719839686247665]
    p0 = [11.740941920178892, 4.5719839686247665] 
    println(t1(p0[1],p0[2]),"      ",t2(p0[1],p0[2]))


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