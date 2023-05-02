function test_simple_integrator_state_and_control_constraints_nonautonomous()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:integrator, :state_dime_1, :lagrange, :x_cons, :u_cons, :nonautonomous) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    #
    f0 = Flow(ocp, (t, x, p) -> 0)
    # constraint
    α=1
    uc(t, x, p) = -2*(t-2)
    g(t, x) = constraint(ocp, :x_cons)(t, x)
    μc(t, x, p) = -α*p
    fc = Flow(ocp, uc, (t, x, _) -> g(t, x), μc)

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    #
    function shoot!(s, p0, t1, t2, ν2)
        x1, p1 = f0(t0, x0, p0, t1)
        x2, p2 = fc(t1, x1, p1, t2)
        xf, pf = f0(t2, x2, p2+ν2, tf)
        s[1] = pf
        s[2] = g(t1, x1)
        s[3] = p1 - exp(-α*t1)
        s[4] = t2-2
    end

    # tests
    t1 = 1
    t2 = 2
    p0 = 0.35
    ν2 = -0.15
    ξ  = [p0, t1, t2, ν2]

    function fparams(ξ) 
        p0, t1, t2, ν2 = ξ[1:4]
        fsol1 = f0((t0, t1), x0, p0); sol1 = fsol1.ode_sol
        fsol2 = fc((t1, t2), sol1(t1)[1], sol1(t1)[2]); sol2 = fsol2.ode_sol
        fsol3 = f0((t2, tf), sol2(t2)[1], sol2(t2)[2]+ν2); sol3 = fsol3.ode_sol
        x(t) = (t ≤ t1)*sol1(t)[1:1] + (t1 < t ≤ t2)*sol2(t)[1:1] + (t2 < t)*sol3(t)[1:1]
        p(t) = (t ≤ t1)*sol1(t)[2:2] + (t1 < t ≤ t2)*sol2(t)[2:2] + (t2 < t)*sol3(t)[2:2]
        u(t) = (t ≤ t1)*[0] + (t1 < t ≤ t2)*[-2*(t-2)] + (t2 < t)*[0]
        return t0, x0, tf, deepcopy(x), deepcopy(p), deepcopy(u)
    end

    nle = (s, ξ) -> shoot!(s, ξ...)
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title, flow=:noflow)

end