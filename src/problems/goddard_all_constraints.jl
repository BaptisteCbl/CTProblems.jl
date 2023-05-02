EXAMPLE=(:goddard, :all_constraints, :altitude, :x_dim_3, :u_dim_1, :mayer, :x_cons, :u_cons, :singular_arc)

@eval function OCPDef{EXAMPLE}()
    # should return an OptimalControlProblem{example} with a message, a model and a solution

    # 
    title = "Goddard problem with state constraint - maximise altitude"

    # ------------------------------------------------------------------------------------------
    # the model
    #n = 3
    #m = 1
    #
    # parameters
    Cd = 310
    Tmax = 3.5
    β = 500
    b = 2
    t0 = 0
    r0 = 1
    v0 = 0
    vmax = 0.1
    m0 = 1
    mf = 0.6
    x0 = [ r0, v0, m0 ]

    # model
    ocp = Model()

    time!(ocp, :initial, t0) # if not provided, final time is free
    state!(ocp, 3, ["r", "v", "m"]) # state dim
    control!(ocp, 1) # control dim

    # use all possible types of constraints
    # initial condition
    constraint!(ocp, :initial, x0, :initial_constraint)
    # final condition
    constraint!(ocp, :final, Index(3), mf, :final_constraint)
    # state constraint
    constraint!(ocp, :state, x->x[2], -Inf, vmax, :state_con_vmax)
    # control constraint
    constraint!(ocp, :control, u->u, -Inf, 1, :control_con_umax)
    # mixed constraint
    constraint!(ocp, :mixed, (x,u)->x[3], mf, Inf, :mixed_con_mmin)
    # state box
    constraint!(ocp, :state, Index(1), r0, Inf, :state_box_rmin)
    constraint!(ocp, :state, Index(2), v0, Inf, :state_box_vmin)
    # control box
    constraint!(ocp, :control, Index(1), 0, Inf, :control_box_umin)

    objective!(ocp, :mayer,  (t0, x0, tf, xf) -> xf[1], :max)

    function F0(x)
        r, v, m = x
        D = Cd * v^2 * exp(-β*(r - 1))
        return [ v, -D/m - 1/r^2, 0 ]
    end
    function F1(x)
        r, v, m = x
        return [ 0, Tmax/m, -b*Tmax ]
    end
    f(x, u) = F0(x) + u*F1(x)

    constraint!(ocp, :dynamics, f)

    # +++ NB. solution is supposed to be the same as the original goddard
    # although the exact formulation of the constraints is slightly different
    # we can keep the solution below or just set the objective maybe ?

    # ------------------------------------------------------------------------------------------
    # the solution

    u0(x, p) = 0.
    u1(x, p) = 1.
    #
    H0(x, p) = p' * F0(x)
    H1(x, p) = p' * F1(x)
    H01 = Poisson(H0, H1)
    H001 = Poisson(H0, H01)
    H101 = Poisson(H1, H01)
    us(x, p) = -H001(x, p) / H101(x, p) # singular control of order 1
    #
    g(x) = vmax-constraint(ocp, :state_con_vmax)(x) # g(x, u) ≥ 0 (cf. nonnegative multiplier)
    ub(x, _) = -Ad(F0, g)(x) / Ad(F1, g)(x) # boundary control
    μb(x, p) = H01(x, p) / Ad(F1, g)(x)

    # associated flows
    abstol=1e-12
    reltol=1e-12
    f0 = Flow(ocp, u0, abstol=abstol, reltol=reltol)
    f1 = Flow(ocp, u1, abstol=abstol, reltol=reltol)
    fs = Flow(ocp, us, abstol=abstol, reltol=reltol)
    fb = Flow(ocp, ub, (x, _) -> g(x), μb, abstol=abstol, reltol=reltol)
    #
    p0 = [3.945764658668555, 0.15039559623198723, 0.053712712939991955]
    t1 = 0.023509684041475312
    t2 = 0.059737380900899015
    t3 = 0.10157134842460895
    tf = 0.20204744057146434
    
    f1sb0 = f1 * (t1, fs) * (t2, fb) * (t3, f0) # concatenation of the flows
    flow_sol = f1sb0((t0, tf), x0, p0)
    sol = CTFlows.OptimalControlSolution(flow_sol)

    # add to the sol
    sol.objective = flow_sol.ode_sol(tf)[1]
    sol.message = "structure: B+SCB0"
    sol.infos[:resolution] = :numerical

    #
    return OptimalControlProblem(title, ocp, sol)

end