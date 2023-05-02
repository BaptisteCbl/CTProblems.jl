# using revise to recompile
using Revise
try
    revise(CTProblems)
catch
end

using CTProblems

prob = Problem(:integrator, :energy, :state_dim_2, :control_dim_1, :lagrange, :control_constraint)


display(prob.model)

plot(prob.solution)
