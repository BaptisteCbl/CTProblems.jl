# [Simple integrator: constraint minimisation state and control constraint non autonomous](@id SINA)

The constraint minimisation simple integrator problem consists in minimising

```math
    \int_{0}^{3} e^{-α*t}*u(t) \, \mathrm{d}t
```

subject to the constraints

```math
    \dot x(t) = u(t), u(t) \in [0,3], \\
    1 - x(t) - (t-1)^2 \in [-\infty,0]
```

and the limit conditions

```math
    x(0) = 0, \quad x(1) = xf.
```

You can access the problem in the CTProblems package:

```@example main
using CTProblems
prob = Problem(:integrator, :state_dime_1, :lagrange, :x_cons, :u_cons, :nonautonomous)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 400))
```