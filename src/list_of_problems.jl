# list of problems
problems = ()
problems = add(problems, (:exponential, :dim1, :energy))
problems = add(problems, (:integrator, :dim2, :energy))
problems = add(problems, (:goddard, :state_constraint))
problems = add(problems, (:exponential, :dim1, :energy))
problems = add(problems, (:integrator, :dim1, :absolute, :constraint))
problems = add(problems, (:turnpike, :dim1))
problems = add(problems, (:integrator, :dim1, :energy, :free))
problems = add(problems, (:integrator, :dim1, :squaresum, :free))
problems = add(problems, (:integrator, :dim1, :time, :free, :constraint))
problems = add(problems, (:dummy, )) # to test exception not implemented