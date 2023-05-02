function test_simple_exponential_energy()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:exponential, :energy, :x_dim_1, :u_dim_1, :lagrange) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    f = Flow(ocp, (x, p) -> p)

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    xf_ = final_condition(ocp)
    #
    function shoot!(s, p0)
        xf, pf = f(t0, x0, p0, tf)
        s[1] = xf - xf_
    end
 
    # tests
    a = xf_ - x0*exp(-tf)
    b = sinh(tf)
    ξ = a/b
    fparams(ξ) = (t0, x0, ξ, tf, f)
    test_by_shooting(ocp, shoot!, ξ, fparams, sol, 1e-3, title)

end