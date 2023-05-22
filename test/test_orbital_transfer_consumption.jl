function test_orbital_transfer_consumption()

    # problem = model + solution
    prob = Problem(:orbital_transfert, :consumption, :x_dim_4, :u_dim_2, :lagrange, :u_cons, :non_diff_wrt_u) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    m0     = 2000
    F_max  = 100
    γ_max  = F_max*3600.0^2/(m0*10^3)
    μ      = 5.1658620912*1e12
    rf     = 42165
    rf3    = rf^3
    α      = sqrt(μ/rf3)
    #
    u(p) = [p[3], p[4]]/sqrt(p[3]^2 + p[4]^2)
    #
    f0 = Flow(ocp, (_, _) -> [0, 0])
    f1 = Flow(ocp, (_, p) -> u(p))

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    c(xf) = constraint(ocp, :boundary_constraint)(t0, x0, tf, xf)
    Φ(x, p) = x[2]*(p[1]+α*p[4]) - x[1]*(p[2]-α*p[3])
    sw(p) = γ_max*(p[3]^2 + p[4]^2) - 1

    #
    function shoot!(s, p0, t1, t2, t3, t4)
        #
        x1, p1 = f1(t0, x0, p0, t1)
        x2, p2 = f0(t1, x1, p1, t2)
        x3, p3 = f1(t2, x2, p2, t3)
        x4, p4 = f0(t3, x3, p3, t4)
        xf, pf = f1(t4, x4, p4, tf)
        #
        s[1:3] = c(xf)
        s[4] = Φ(xf, pf)
        s[5] = sw(p1)
        s[6] = sw(p2)
        s[7] = sw(p3)
        s[8] = sw(p4)
        #
    end

    # tests
    p0 = [0.02698412111231433, 0.006910835140705538, 0.050397371862031096, -0.0032972040120747836]
    ti = [0.4556797711668658, 3.6289692721936913, 11.683607683450061, 12.505465498856514]
    ξ  = [p0; ti]

    function fparams(ξ) 
        p0 = ξ[1:4]
        t1, t2, t3, t4 = ξ[5:8]
        f = f1 * (t1, f0) * (t2, f1) * (t3, f0) * (t4, f1)
        return (t0, x0, p0, tf, f)
    end

    function objective(ξ)
        t1, t2, t3, t4 = ξ[5:8]
        return (t1-t0)+(t3-t2)+(tf-t4)
    end

    nle = (s, ξ) -> shoot!(s, ξ[1:4], ξ[5:8]...)

    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title, objective=objective)

end