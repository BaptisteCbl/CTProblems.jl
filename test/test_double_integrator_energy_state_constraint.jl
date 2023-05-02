function test_double_integrator_energy_state_constraint()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:integrator, :energy, :x_dim_2, :u_dim_1, :lagrange, :x_cons, :order_2) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    #
    fs = Flow(ocp, (x, p) -> p[2])
    # constraint
    l = 1/9
    uc(x, p) = 0
    g(x) = constraint(ocp, :x_cons)(x) - l
    μc(x, p) = 0
    fc = Flow(ocp, uc, (x, _) -> g(x), μc)

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    xf_ = final_condition(ocp)
    #
    function shoot!(s, p0, t1, t2, ν1, ν2)
        x1, p1 = fs(t0, x0, p0, t1)
        x2, p2 = fc(t1, x1, p1+ν1*[1, 0], t2)
        xf, pf = fs(t2, x2, p2+ν2*[1, 0], tf)
        s[1:2] = xf_ - xf
        s[3] = g(x1)
        s[4] = x1[2]
        s[5] = p1[2]
        s[6] = p2[2]
    end

    # tests
    t1 = 1/3
    t2 = 2/3
    p0 = [-18, -6]
    ν1 = 18
    ν2 = 18
    ξ  = [p0..., t1, t2, ν1, ν2]

    function fparams(ξ) 
        p0 = ξ[1:2]
        t1, t2, ν1, ν2 = ξ[3:6]
        fsol1 = fs((t0, t1), x0, p0); sol1 = fsol1.ode_sol
        fsol2 = fc((t1, t2), sol1(t1)[1:2], sol1(t1)[3:4]+ν1*[1, 0]); sol2 = fsol2.ode_sol
        fsol3 = fs((t2, tf), sol2(t2)[1:2], sol2(t2)[3:4]+ν2*[1, 0]); sol3 = fsol3.ode_sol
        x(t) = (t ≤ t1)*sol1(t)[1:2] + (t1 < t ≤ t2)*sol2(t)[1:2] + (t2 < t)*sol3(t)[1:2]
        p(t) = (t ≤ t1)*sol1(t)[3:4] + (t1 < t ≤ t2)*sol2(t)[3:4] + (t2 < t)*sol3(t)[3:4]
        u(t) = (t ≤ t1)*sol1(t)[4:4] + (t1 < t ≤ t2)*[0] + (t2 < t)*sol3(t)[4:4]
        return t0, x0, tf, deepcopy(x), deepcopy(p), deepcopy(u)
    end

    nle = (s, ξ) -> shoot!(s, ξ[1:2], ξ[3:6]...)
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title, flow=:noflow)

end