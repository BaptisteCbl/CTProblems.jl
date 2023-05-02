function test_problem()
   
    # displays
    @test display(Problem(:integrator)) isa Nothing
    @test display(Problems(:integrator)) isa Nothing
    @test display(()) isa Nothing

    # exception
    e = CTProblems.NonExistingProblem((:dummy, ))
    @test_throws ErrorException error(e)
    @test typeof(sprint(showerror, e)) == String
    @test_throws CTProblems.NonExistingProblem Problem(:dummy)

    # get problems
    @test ProblemsDescriptions() isa Tuple
    @test Problems(:integrator) isa Tuple
    @test Problem(:integrator) isa CTProblems.OptimalControlProblem

    # get problems with expression
    @test (@ProblemsDescriptions) isa Tuple
    @test (@ProblemsDescriptions :integrator & :energy) isa Tuple
    @test (@ProblemsDescriptions :integrator) isa Tuple
    @test (@Problems) isa Tuple
    @test (@Problems :integrator & :energy) isa Tuple
    @test (@Problems :integrator) isa Tuple

    # _keep
    e = :(:integrator)
    d = (:integrator, :energy)
    @test CTProblems._keep(d, e) == true
    
    e = :(:integrator)
    d = (:integrator, :energy)
    @test CTProblems._keep(d, Symbol("!"), e) == false
    
    e = :(!(:integrator))
    d = (:integrator, :energy)
    @test CTProblems._keep(d, e) == false
    
    e = :(!(:integrator))
    d = (:exponential, :energy)
    @test CTProblems._keep(d, e) == true
    
    e=:( (!:integrator & :energy) | :goddard )
    d = (:integrator, :energy)
    @test CTProblems._keep(d, e) == false
    
    e=:( (!:integrator & :energy) | :goddard )
    d = (:exponential, :energy)
    @test CTProblems._keep(d, e) == true
    
    e=:( (!:integrator & :energy) | :goddard )
    d = (:goddard, :altitude)
    @test CTProblems._keep(d, e) == true
    
    e=:( (!:integrator & :energy) | :goddard | :dummy )
    d = (:exponential, :energy)
    @test CTProblems._keep(d, e) == true
    
    e=:( (!:integrator & :energy) | :goddard | :dummy )
    d = (:goddard, :altitude)
    @test CTProblems._keep(d, e) == true
    
    e=:( (!:integrator & :energy) | :goddard | :dummy )
    d = (:dummy, :altitude)
    @test CTProblems._keep(d, e) == true

    e=:( !(:exponential & :energy) | :goddard | :dummy )
    d = (:exponential, :energy, :toto)
    @test CTProblems._keep(d, e) == false

    e=:( !(:exponential & :energy) | :toto | :dummy )
    d = (:exponential, :energy, :toto)
    @test CTProblems._keep(d, e) == true

    # list of problems
    ex0 = CTProblems.get_example("dummy_example.jl")
    ex1 = (:dummy,) #(:integrator, :energy, :x_dim_2, :u_dim_1, :lagrange, :noconstraints)
    @test ex0 == ex1

end