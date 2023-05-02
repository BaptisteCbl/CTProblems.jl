function test_orbital_transfer_time()

    # problem = model + solution
    prob = Problem(:orbital_transfert, :time, :x_dim_4, :u_dim_2, :mayer, :u_cons) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    m0     = 2000.0
    F_max  = 100.0
    γ_max  = F_max*3600.0^2/(m0*10^3)
    μ      = 5.1658620912*1.0e12
    rf     = 42165
    rf3    = rf^3
    α      = sqrt(μ/rf3)
    #
    H(x, p, u) =  p[1]*x[3] + p[2]*x[4] + 
                    p[3]*(-μ*x[1]/(sqrt(x[1]^2+x[2]^2))^3 + u[1]) + 
                    p[4]*(-μ*x[2]/(sqrt(x[1]^2+x[2]^2))^3 + u[2])
    u(p) = [p[3], p[4]]*γ_max/sqrt(p[3]^2 + p[4]^2)
    f = Flow(ocp, (_, p) -> u(p))

    # shooting function
    t0 = ocp.initial_time
    x0 = initial_condition(ocp)
    c(tf, xf) = constraint(ocp, :boundary_constraint)(t0, x0, tf, xf)
    Φ(x, p) = x[2]*(p[1]+α*p[4]) - x[1]*(p[2]-α*p[3])

    #
    function shoot!(s, p0, tf)
        xf, pf = f(t0, x0, p0, tf)
        s[1:3] = c(tf, xf)
        s[4] = Φ(xf, pf)
        s[5] = H(xf, pf, u(pf)) - 1
    end

    # tests
    ξ = [0.00010323118914991345, 4.892642780583378e-5, 0.00035679672938385165, -0.0001553613885740003, 13.403181957151876]   # pour F_max = 100N
    fparams(ξ) = (t0, x0, ξ[1:4], ξ[5], f)
    nle = (s, ξ) -> shoot!(s, ξ[1:4], ξ[5])
    test_by_shooting(ocp, nle, ξ, fparams, sol, 1e-3, title)

end