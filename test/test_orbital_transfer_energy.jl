function test_orbital_transfer_energy()

    # problem = model + solution
    prob = Problem(:orbital_transfert, :energy, :x_dim_4, :u_dim_2, :lagrange) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    f = Flow(ocp, (x, p) -> [p[3], p[4]])

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    c(x) = constraint(ocp, :boundary_constraint)(t0, x0, tf, x)
    μ      = 5.1658620912*1.0e12
    rf     = 42165
    rf3    = rf^3
    α      = sqrt(μ/rf3);
    Φ(x, p) = x[2]*(p[1]+α*p[4]) - x[1]*(p[2]-α*p[3])

    #
    function shoot!(s, p0)
        xf, pf = f(t0, x0, p0, tf)
        s[1:3] = c(xf)
        s[4] = Φ(xf, pf)
    end

    # tests
    ξ = [131.44483634894812, 34.16617425875177, 249.15735272382514, -23.9732920001312]   # pour F_max = 100N
    fparams(ξ) = (t0, x0, ξ, tf, f)
    test_by_shooting(ocp, shoot!, ξ, fparams, sol, 1e-3, title)

end