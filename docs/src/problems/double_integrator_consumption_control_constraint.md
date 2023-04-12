# [Double integrator: consumption minimisation](@id DIC)

The consumption minimisation double integrator problem consists in minimising

```math
    \int_{0}^{1} |u(t)| \, \mathrm{d}t
```

subject to the constraints

```math
    \dot x_1(t) = x_2(t), \quad \dot x_2(t) = u(t), \quad u(t) \in [-5,5]
```

and the limit conditions

```math
    x(0) = (-1,0), \quad x(1) = (0,0).
```

You can access the problem in the CTProblems package:

```@example main
using CTProblems
prob = Problem(:integrator, :consumption, :state_dim_2, :control_dim_1, :lagrange, :control_constraint, :state_non_differentiable, :control_non_differentiable)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 400))
```
