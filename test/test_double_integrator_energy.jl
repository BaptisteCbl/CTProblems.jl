function test_double_integrator_energy()

    # problem = model + solution
    prob = Problem(:integrator, :energy, :x_dim_2, :u_dim_1, :lagrange, :noconstraints) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    f = Flow(ocp, (x, p) -> p[2])

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    xf_ = final_condition(ocp)
    #
    function shoot!(s, p0)
        xf, pf = f(t0, x0, p0, tf)
        s[1:2] = xf - xf_
    end

    # tests
    ξ = [12, 6]
    fparams(ξ) = (t0, x0, ξ, tf, f)
    test_by_shooting(ocp, shoot!, ξ, fparams, sol, 1e-3, title)

end