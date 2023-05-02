function test_double_integrator_time()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:integrator, :time, :x_dim_2, :u_dim_1, :mayer, :u_cons) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    γ  = 1
    fm = Flow(ocp, (x, p) -> -γ)
    fp = Flow(ocp, (x, p) -> +γ)

    # shooting function
    t0 = ocp.initial_time
    x0 = initial_condition(ocp)
    xf = final_condition(ocp)
    #
    function shoot!(s, p0, t1, tf)
        x1, p1 = fp(t0, x0, p0, t1)
        xf_, pf = fm(t1, x1, p1, tf)
        s[1:2] = xf_ - xf
        s[3] = p1[2]
        s[4] = pf[1]*xf_[2]+pf[2]*(-γ) - 1
    end

    # tests
    t1 = 1
    tf = 2
    p0 = [1, 1]
    ξ = [p0..., t1, tf]

    function fparams(ξ) 
        # concatenation of the flows
        p0 = ξ[1:2]
        t1, tf = ξ[3:4]
        return t0, x0, p0, tf, fp * (t1, fm)
    end

    nle = (s, ξ) -> shoot!(s, ξ[1:2], ξ[3], ξ[4])
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title)

end