function test_double_integrator_energy_control_constraint()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:integrator, :energy, :x_dim_2, :u_dim_1, :lagrange, :u_cons) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    γ  = 5
    fm = Flow(ocp, (x, p) -> -γ)
    fp = Flow(ocp, (x, p) -> +γ)
    fs = Flow(ocp, (x, p) -> p[2])

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    xf = final_condition(ocp)
    #
    function shoot!(s, p0, t1, t2)
        x1, p1 = fp(t0, x0, p0, t1)
        x2, p2 = fs(t1, x1, p1, t2)
        xf_, pf = fm(t2, x2, p2, tf)
        s[1:2] = xf_ - xf
        s[3] = p1[2] - γ
        s[4] = p2[2] + γ
    end

    # tests
    t1 = 0.25*tf
    t2 = 0.75*tf
    p0 = [11/tf, 6]
    ξ = [p0..., t1, t2]

    function fparams(ξ) 
        # concatenation of the flows
        p0 = ξ[1:2]
        t1, t2 = ξ[3:4]
        return t0, x0, p0, tf, fp * (t1, fs) * (t2, fm)
    end

    nle = (s, ξ) -> shoot!(s, ξ[1:2], ξ[3], ξ[4])
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title)

end