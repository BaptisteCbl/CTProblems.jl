function test_simple_integrator_nonsmooth_turnpike()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:turnpike, :integrator, :state_energy, :x_dim_1, :u_dim_1, :lagrange, :u_cons, :singular_arc) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    fm = Flow(ocp, (x, p) -> -1)
    fp = Flow(ocp, (x, p) -> +1)
    f0 = Flow(ocp, (x, p) ->  0)

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    xf = final_condition(ocp)
    #
    function shoot!(s, p0, t1, t2)
        x1, p1 = fm(t0, x0, p0, t1)
        x2, p2 = f0(t1, x1, p1, t2)
        xf_, pf = fp(t2, x2, p2, tf)
        s[1] = xf_ - xf
        s[2] = x1
        s[3] = p1
    end

    # tests
    t1 = 1
    t2 = 1.5
    p0 = -1
    ξ = [p0, t1, t2]

    function fparams(ξ) 
        # concatenation of the flows
        p0 = ξ[1]
        t1, t2 = ξ[2:3]
        f  = fm * (t1, f0) * (t2, fp)
        return t0, x0, p0, tf, f
    end

    nle = (s, ξ) -> shoot!(s, ξ[1], ξ[2], ξ[3])
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title)

end