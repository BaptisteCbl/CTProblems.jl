function test_simple_exponential_time()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:exponential, :time, :x_dim_1, :u_dim_1, :lagrange) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    f⁺ = Flow(ocp, (x, p) -> 1)
    H⁺(x, p) = p * (-x + 1)

    # shooting function
    t0 = ocp.initial_time
    x0 = initial_condition(ocp)
    xf_ = final_condition(ocp)
    #
    function shoot!(s, p0, tf)
        xf, pf = f⁺(t0, x0, p0, tf)
        s[1] = xf - xf_
        s[2] = H⁺(xf, pf) - 1
    end

    # tests
    γ  = 1
    tf = log((-1-γ)/(xf_-γ))
    p0 = exp(t0-tf)/(γ-xf_)
    ξ = [p0, tf]
    function fparams(ξ)
        p0 = ξ[1]
        tf = ξ[2]
        return (t0, x0, p0, tf, f⁺)
    end

    nle = (s, ξ) -> shoot!(s, ξ[1], ξ[2])
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title)

end