function test_simple_integrator_mixed_constraint()

    # ---------------------------------------------------------------
    # problem = model + solution
    prob = Problem(:integrator, :x_dim_1, :u_dim_1, :lagrange, :mixed_constraint) 
    ocp = prob.model
    sol = prob.solution
    title = prob.title

    # Flow(ocp, u)
    H(x, p, u, η) = p*u - u + η*(x + u) # pseudo-Hamiltonian
    η(x, p) = -(p + 1) # multiplier associated to the mixed constraint
    u(x, p) = -x
    H(x, p) = H(x, p, u(x, p), η(x, p))
    f = Flow(Hamiltonian(H))

    # shooting function
    t0 = ocp.initial_time
    tf = ocp.final_time
    x0 = initial_condition(ocp)
    #
    function shoot!(s, p0)
        xf, pf = f(t0, x0, p0, tf)
        s[1] = pf
    end
 
    # tests
    p0 = 1
    fparams(p0) = (t0, x0, p0, tf, f, u)
    test_by_shooting(ocp, shoot!, p0, fparams, sol, 1e-3, title, flow=:hamiltonian)

end