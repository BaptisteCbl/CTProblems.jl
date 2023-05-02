function test_double_integrator_energy_distance()

    # problem = model + solution
    prob = Problem(:integrator, :energy, :distance, :x_dim_2, :u_dim_1, :bolza) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    f = Flow(ocp, (x, p) -> p[2])

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    #
    function shoot!(s, p0)
        xf, pf = f(t0, x0, p0, tf)
        s[1:2] = pf - [1/2, 0]
    end

    # tests
    ξ = [1/2, tf/2]
    fparams(ξ) = (t0, x0, ξ, tf, f)
    test_by_shooting(ocp, shoot!, ξ, fparams, sol, 1e-3, title)

end