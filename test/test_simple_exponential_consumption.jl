function test_simple_exponential_consumption()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:exponential, :consumption, :x_dim_1, :u_dim_1, 
        :lagrange, :non_diff_wrt_u) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    f0 = Flow(ocp, (x, p) -> 0)
    f1 = Flow(ocp, (x, p) -> 1)

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    xf_ = final_condition(ocp)
    #
    function shoot!(s, p0, t1)
        x1, p1 = f0(t0, x0, p0, t1)
        xf, pf = f1(t1, x1, p1, tf)
        s[1] = xf - xf_
        s[2] = p1 - 1
    end

    # tests
    p0 = 1/(x0-(xf_-1)/exp(-tf))
    ξ = [p0, -log(p0)]
    function fparams(ξ)
        p0 = ξ[1]
        t1 = ξ[2]
        f = f0 * (t1, f1)
        return (t0, x0, p0, tf, f)
    end

    function objective(ξ)
        t1 = ξ[2]
        return (tf-t1)
    end

    nle = (s, ξ) -> shoot!(s, ξ[1], ξ[2])

    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title, objective=objective)

end