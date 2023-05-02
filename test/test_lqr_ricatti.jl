function test_lqr_ricatti()

    # problem = model + solution
    prob = Problem(:lqr, :x_dim_2, :u_dim_1, :lagrange)
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    B = [0; 1]
    f = Flow(ocp, (x, p) -> B'*p)

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    #
    function shoot!(s, p0)
        xf, pf = f(t0, x0, p0, tf)
        s[1:2] = pf
    end

    # tests
    ξ = [1, 2]
    fparams(ξ) = (t0, x0, ξ, tf, f)
    test_by_shooting(ocp, shoot!, ξ, fparams, sol, 1e-3, title)

end