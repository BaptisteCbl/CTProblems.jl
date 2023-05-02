# [Simple integrator: state energy minimisation nonsmooth turnpike](@id SISE)

The state energy minimisation simple integrator problem consists in minimising

```math
    \int_{0}^{2} x(t)^2 \, \mathrm{d}t
```

subject to the constraints

```math
    \dot x(t) = u(t), u(t) \in [0,\gamma], \\
```

and the limit conditions

```math
    x(0) = 1, \quad x(2) = 0.5.
```

You can access the problem in the CTProblems package:

```@example main
using CTProblems
prob = Problem(:turnpike, :integrator, :state_energy, :x_dim_1, :u_dim_1, :lagrange, :u_cons, :singular_arc)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 400))
```
