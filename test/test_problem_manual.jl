# using revise to recompile
using Revise
try
    revise(CTProblems)
catch
end

using CTProblems

prob = Problem(:lqr, :x_dim_2, :u_dim_1, :lagrange)

display(prob.model)

plot(prob.solution)
