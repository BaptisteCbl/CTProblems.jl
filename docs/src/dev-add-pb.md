# [How to add a new problem](@id add-new-pb)

```@contents
Pages = ["dev-add-pb.md"]
Depth = 2
```

## Find a description

Let us assume you want to add a `planar orbital time minimisation` transfer problem from space mechanics. 
This problem will be described by a `Description`, cf. [`CTBase.jl`](https://github.com/control-toolbox/CTBase.jl),
following the `nomenclature` given in `src/CTProblems.jl`. For instance:

```julia
(:orbital, :time, :x_dim_4, u_dim_2, :mayer)
```

**Nomenclature:**

- `:name_of_the_problem`, and other symbols for a clear description
- if the objective is classical, you can mention it: `:time`, `:energy`, `:consumption`

then

- dimensions: `:x_dim_1`, `:x_dim_2`... `:u_dim_1`, `:u_dim_2`...
- objective type: `:lagrange`, `:mayer`, `:bolza`
- constraint arc (active on the solution): `:x_cons`, `:u_cons`, `:mixed_constraint`
- singular arc (on the solution): `:singular_arc`
- differentiability: `:non_diff_wrt_x`, `:non_diff_wrt_u`

## Create a new file with an empty template

You have to create a new file in the directory `src/problems`. For instance:

```bash
touch src/problems/orbital_planar.jl
```

Add it to the list of problems, in `src/CTProblems.jl`:

```julia
list_of_problems_files = [
    "...",
    "orbital_planar.jl",
]
```

Then, you can write into this file the following empty template.

!!! warning

    It is required to define a variable named `EXAMPLE` containing the description.

```julia
EXAMPLE=(:orbital, :time, :x_dim_4, u_dim_2, :mayer)

@eval function OCPDef{EXAMPLE}()

    # the description
    title = "Planar orbital transfer"

    # the model: to be completed
    ocp = Model()

    # the solution: to be completed
    sol = OptimalControlSolution()

    sol.message = "structure: ..."  # Give the structure as an additional info. 
                                    # For instance, B+S for positive bang followed
                                    # a by singular arc.
                                    
    sol.infos[:resolution] = :numerical # Either :numerical or :analytical
                                        # depending on how you get the solution

    #
    return OptimalControlProblem(title, ocp, sol)

end
```

## Code the model and the solution

For now, you have to complete the model and the solution. You can take example from the already existing problems in `src/problems`. See also the documentation of [`CTBase.jl`](https://github.com/control-toolbox/CTBase.jl). Try to fulfill all the fields of `OptimalControlSolution`.

## Add a unit test

To complete the process, a unit test must be created. See the `test` directory for examples. The unit test should be written by a different person to robustify the process.
