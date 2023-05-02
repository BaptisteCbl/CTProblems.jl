# [Double integrator: energy minimisation state constraint](@id DIESC)

The energy minimisation with control constraint double integrator problem consists in minimising

```math
    0.5\int_{0}^{1} {u(t)}^2 \, \mathrm{d}t
```

subject to the constraints

```math
    \dot x_1(t) = x_2(t), \quad \dot x_2(t) = u(t), \quad u(t) \in \R \\
    x_1(t) \leq l
```

and the limit conditions

```math
    x(0) = (0,1), \quad x(1) = (0,-1).
```

You can access the problem in the CTProblems package:

```@example main
using CTProblems
prob = Problem(:integrator, :energy, :x_dim_2, :u_dim_1, :lagrange, :x_cons, :order_2)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 400))
```
