# [Goddard with state constraints](@id Godda)

```@example main
using CTProblems
```

You can access the problem in the CTProblems package:

```@example main
prob = Problem(:goddard, :classical)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 900))
```
