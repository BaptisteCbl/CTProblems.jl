# using revise to recompile
using Revise
try
    revise(CTProblems)
catch
end

using CTProblems

prob = Problem(:orbital_transfert, :energy, :x_dim_4, :u_dim_2, :lagrange, :singular_arc)

display(prob.model)

plot(prob.solution)
