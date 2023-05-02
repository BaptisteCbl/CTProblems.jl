# [Double integrator: energy minimisation](@id DIE)

The energy minimisation problem consists in minimising the cost functional in Lagrange form

```math
    \frac{1}{2}\int_{t_0}^{t_f} u^2(t) \, \mathrm{d}t
```

subject to the constraints for $t \in [t_0, t_f]$ a.e.

```math
    \dot x_1(t) = x_2(t), \quad \dot x_2(t) = u(t) \in \mathbb{R},
```

and the limit conditions

```math
    x(t_0) = (a, b), \quad x(t_f) = x_f,
```

with $a$, $b \in \mathbb{R}$, $x_f \in \mathbb{R}^2$ and $t_f \ge t_0 = 0$ fixed. Denoting $H(x, p, u) \coloneqq p_1 x_2 + p_2 u - u^2/2$ the pseudo-Hamiltonian (in the normal case), then, by the [Pontryagin Maximum Principle](https://en.wikipedia.org/wiki/Pontryagin%27s_maximum_principle), the maximising control is given in feedback form by

```math
    u(x, p) \coloneqq p_2.
```

```@raw html
<img src="../assets/di_energy_u.svg" style="display: block; margin: 0 auto;">
```

Thus, solving this optimal control problem leads to solve the following boundary value problem:

```math
    \left\{
    \begin{aligned}
        & \dot{x}_1(t) = x_2(t), \quad 
        \dot{x}_2(t) = u(x(t), p(t)), \quad 
        \dot{p}_1(t) = 0, \quad 
        \dot{p}_2(t) = -p_1(t), \\
        & x(0) = (a, b), \quad x(t_f) = x_f.
    \end{aligned}
    \right.
```

where $p(t) = (p_1(t), p_2(t))$ is the adjoint vector.
Integrating first the differential system, we get

```math
    \left\{
    \begin{aligned}
        p_1(t) &= \alpha, \\[0.3em]
        p_2(t) &= -\alpha t + \beta, \\[0.3em]
        x_2(t) &= b + \int_0^t (-\alpha s + \beta) \mathrm{d} s = 
        b + \beta t - \frac{\alpha}{2} t^2, \\[0.3em]
        x_1(t) &= a + b t + \frac{\beta}{2} t^2 - \frac{\alpha}{6} t^3,
    \end{aligned}
    \right.    
```

with $p(0) = (\alpha, \beta)$ the parameters to identify. Solving the boundary value problem, that is finding $(\alpha, \beta)$, is then equivalent to solve the shooting equation

```math
    S(\alpha, \beta) \coloneqq
    \begin{pmatrix}
        \displaystyle a + b\, t_f + \frac{\beta}{2} t_f^2 - \frac{\alpha}{6} t_f^3 \\[0.5em]
        \displaystyle b + \beta\, t_f - \frac{\alpha}{2} t_f^2
    \end{pmatrix}
    - x_f = A\, p_0 - B = 0,
```

where we set

```math
    A \coloneqq 
    \begin{pmatrix}
    -\frac{t_f^3}{6} & \frac{t_f^2}{2} \\[0.5em]
     -\frac{t_f^2}{2} & t_f
    \end{pmatrix},
    \quad
    B \coloneqq ( -a-b t_f, -b) + x_f
    \quad
    \text{and}
    \quad 
    p_0 \coloneqq (\alpha, \beta).
```

Note that the **shooting function** $S$ is linear. If $t_f \ne 0$, then, $A$ is invertible since
$\det A = t_f^4/12$ and the solution is then given by

```math
    p^*_0 = A^{-1} B.
```

For an illusration we set $t_f=1$, $x_0 = (-1, 0)$ and $x_f = (0, 0)$, see the following figure.
The red sphere represents the solution.

```@raw html
<img src="../assets/di_energy_sfun.png" style="display: block; margin: 0 auto;">
```

You can access the problem from the `CTProblems.jl` package:

```@example main
using CTProblems
prob = Problem(:integrator, :energy, :x_dim_2, :u_dim_1, :lagrange, :noconstraints)
```

Then, the model is given by

```@example main
prob.model
```

You can plot the solution.

```@example main
plot(prob.solution, size=(700, 700))
```
