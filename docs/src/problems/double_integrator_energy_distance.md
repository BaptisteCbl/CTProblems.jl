# [Double integrator: energy and distance minimisation](@id DIED)

The energy and distance minimisation double integrator problem consists in minimising

```math
    -0.5*x_1(t_f) + 0.5*\int_{0}^{1} {u(t)}^2 \, \mathrm{d}t 
```

subject to the constraints

```math
    \dot x_1(t) = x_2(t), \quad \dot x_2(t) = u(t), \quad u(t) \in \R
```

and the limit conditions

```math
    x(0) = (0,0)
```

You can access the problem in the CTProblems package:

```@example main
using CTProblems
prob = Problem(:integrator, :energy, :distance, :x_dim_2, :u_dim_1, :bolza)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 400))
```
