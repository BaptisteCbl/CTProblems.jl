# [Simple integrator: constraint minimisation mixed constraint](@id SICMMC)

The constraint-min simple integrator problem consists in minimising

```math
    -\int_{0}^{1} u(t) \, \mathrm{d}t
```

subject to the constraints

```math
    \dot x(t) = u(t), u(t) \in [0,\infty], \\
    x(t) + u(t) \in [-\infty,0]
```

and the limit conditions

```math
    x(0) = -1, \quad x(1) = xf.
```

You can access the problem in the CTProblems package:

```@example main
using CTProblems
prob = Problem(:integrator, :x_dim_1, :u_dim_1, :lagrange, :mixed_constraint)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 400))
```
