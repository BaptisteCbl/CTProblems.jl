# using revise to recompile
using Revise
try
    revise(CTProblems)
catch
end

using CTProblems

prob = Problem(:integrator, :dim2, :energy, :state_constraint)

display(prob.model)

plot(prob.solution)

#println(prob.message) #if not commented the plot window does not show
