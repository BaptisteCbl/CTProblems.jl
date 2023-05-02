function test_simple_integrator_lqr_free_tf()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:integrator, :lqr, :free_final_time, :x_dim_1, :u_dim_1, :bolza) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    f = Flow(ocp, (x, p) -> p)

    # shooting function
    t0 = ocp.initial_time
    x0 = initial_condition(ocp)
    xf = final_condition(ocp)
    #
    function shoot!(s, p0, tf)
        xf_, pf = f(t0, x0, p0, tf)
        s[1] = xf - xf_
        s[2] = 0.5*(pf^2-xf_^2) - 1 # H = 1
    end
 
    # tests
    p0 = 1.5
    tf = 0.5
    ξ = [p0, tf]
    fparams(ξ) = (t0, x0, ξ[1], ξ[2], f)
    nle = (s, ξ) -> shoot!(s, ξ[1], ξ[2])
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title)

end