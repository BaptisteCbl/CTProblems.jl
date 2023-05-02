# [Double integrator: time minimisation](@id DIT)

The time minimisation double integrator problem consists in minimising

```math
    tf 
```

subject to the constraints

```math
    \dot x_1(t) = x_2(t), \quad \dot x_2(t) = u(t), \quad u(t) \in [-1,1]
```

and the limit conditions

```math
    x(0) = (-1,0), \quad x(t_f) = (0,0)
```

You can access the problem in the CTProblems package:

```@example main
using CTProblems
prob = Problem(:integrator, :time, :x_dim_2, :u_dim_1, :mayer, :u_cons)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 400))
```
