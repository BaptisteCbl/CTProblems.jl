function test_simple_integrator_energy_free_tf()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:integrator, :energy, :free_final_time, :x_dim_1, :u_dim_1, :lagrange) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    f = Flow(ocp, (x, p) -> p)

    # shooting function
    t0 = ocp.initial_time
    x0 = initial_condition(ocp)
    c(tf, xf) = constraint(ocp, :boundary_constraint)(t0, x0, tf, xf)
    #
    function shoot!(s, p0, tf)
        xf, pf = f(t0, x0, p0, tf)
        s[1] = c(tf, xf)
        s[2] = pf - 2
    end
 
    # tests
    p0 = 2
    tf = 10
    ξ = [p0, tf]
    fparams(ξ) = (t0, x0, ξ[1], ξ[2], f)
    nle = (s, ξ) -> shoot!(s, ξ[1], ξ[2])
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title)

end