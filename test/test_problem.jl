#
prob = Problem(:integrator, :dim2)
@test prob isa CTProblems.OptimalControlProblem

#
prob = Problem(:goddard)
@test prob isa CTProblems.OptimalControlProblem

#
prob = Problem(:integrator, :dim1)
@test prob isa CTProblems.OptimalControlProblem