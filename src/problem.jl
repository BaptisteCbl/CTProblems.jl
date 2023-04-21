#
"""
$(TYPEDEF)
"""
abstract type AbstractCTProblem end

"""
$(TYPEDEF)

**Fields**

$(TYPEDFIELDS)

# Example

```@example
julia> using CTProblems
julia> using CTBase
julia> title = "My empty optimal control problem"
julia> ocp = Model()
julia> sol = OptimalControlSolution()
julia> prob = CTProblems.OptimalControlProblem(title, ocp, sol)
julia> prob isa CTProblems.AbstractCTProblem
true
julia> prob isa CTProblems.OptimalControlProblem
true
```
"""
struct OptimalControlProblem <: AbstractCTProblem
    title::String
    model::OptimalControlModel
    solution::OptimalControlSolution
end

"""
$(TYPEDSIGNATURES)

Show the title and the types of the model and the solution of the optimal control problem.

```@example
julia> Problem(:integrator, :energy, :state_dim_2, :control_dim_1)
title           = Double integrator - energy min
model    (Type) = CTBase.OptimalControlModel{:autonomous, :scalar}
solution (Type) = CTBase.OptimalControlSolution
```

"""
function Base.show(io::IO, ::MIME"text/plain", problem::OptimalControlProblem)
    println(io, "title           = ", problem.title)
    println(io, "model    (Type) = ", typeof(problem.model))
    print(  io, "solution (Type) = ", typeof(problem.solution))
end

"""
$(TYPEDSIGNATURES)

Print a tuple of optimal control problems.
"""
function Base.show(io::IO, ::MIME"text/plain", problems::Tuple{Vararg{OptimalControlProblem}})
    println(io, "List of optimal control problems:\n")
    for problem ∈ problems
        println(io, "title           = ", problem.title)
        println(io, "model    (Type) = ", typeof(problem.model))
        print(  io, "solution (Type) = ", typeof(problem.solution))
        println(io, "\n")
    end
end

"""
$(TYPEDSIGNATURES)

Print an empty tuple.
"""
function Base.show(io::IO, ::MIME"text/plain", t::Tuple{})
    print(io, "Tuple{}")
end

# exception
"""
$(TYPEDEF)
"""
abstract type CTProblemsException <: Exception end

# not implemented
"""
$(TYPEDEF)

**Fields**

$(TYPEDFIELDS)
"""
struct NonExistingProblem <: CTProblemsException
    example::Description
end

"""
$(TYPEDSIGNATURES)

Print the error message when the optimal control problem described by `e.example` does not exist. 

"""
function Base.showerror(io::IO, e::NonExistingProblem) 
    print(io, "there is no optimal control problem described by ", e.example)
end

"""
$(TYPEDEF)

A type used to define new problems with a default constructor that throws a [`NonExistingProblem`](@ref)
exception if there is no optimal control problem described by `example`.

# Example

```julia-repl
julia> CTProblems.OCPDef{(:ocp, :dummy)}()
ERROR: there is no optimal control problem described by (:ocp, :dummy)
```

!!! note

    To define a new problem, please refer to the page [How to add a new problem](@ref add-new-pb).

"""
struct OCPDef{description} 
    function OCPDef{description}() where {description}
        throw(NonExistingProblem(description))
    end
end

"""
$(TYPEDSIGNATURES)

Return the list of optimal control problems (without the dummy problem).
"""
_problems_without_dummy() = Tuple(setdiff(problems, ((:dummy,),)))

"""
$(TYPEDSIGNATURES)

Returns the list of problems descriptions consistent with the description, as a Tuple of Description, 
see the page [list of problems descriptions](@ref descriptions-list) for details.

# Example

```@example
julia> ProblemsDescriptions(:integrator, :energy)
```

"""
function ProblemsDescriptions(description...)::Tuple{Vararg{Description}}
    desc = makeDescription(description...)
    problems_list = filter(pb -> desc ⊆ pb, _problems_without_dummy()) # filter only the problems that contain desc
    return problems_list
end

"""
$(TYPEDSIGNATURES)

Returns the list of problems consistent with the description, as a Tuple of OptimalControlProblem, 
see the page [list of problems](@ref problems-list) for details.

# Example

```@example
julia> Problems(:integrator, :energy)
```

"""
function Problems(description...)::Tuple{Vararg{OptimalControlProblem}}
    problems_list = ProblemsDescriptions(description...)
    return map(e -> OCPDef{e}(), problems_list)   # return the list of problems that match desc
end

"""
$(TYPEDSIGNATURES)

Returns the optimal control problem described by `description`.
See the [Introduction](@ref introduction) and page [list of problems](@ref problems-list) for details.

# Example

```@example
julia> Problem(:integrator, :energy)
```

"""
function Problem(description...) 
    example = getFullDescription(makeDescription(description...), problems)
    return OCPDef{example}()
end

"""
$(TYPEDSIGNATURES)

A binding to [CTBase.jl](https://control-toolbox.github.io/CTBase.jl) plot function.

"""
function CTProblems.plot(sol; kwargs...)
    return CTBase.plot(sol; kwargs...)
end

# more sophisticated filters
function _keep(description::Description, e::QuoteNode)
    return e.value ∈ description
end

function _keep(description::Description, s::Symbol, q::QuoteNode)
    @assert s == Symbol("!")
    return q.value ∉ description
end

function _keep(description::Description, s::Symbol, e::Expr)
    @assert s == Symbol("!")
    return !_keep(description, e)
end

function _keep(description::Description, e::Expr)
    @assert hasproperty(e, :head) 
    @assert e.head == :call
    if length(e.args) == 2
        return _keep(description, e.args[1], e.args[2])
    elseif length(e.args) == 3
        @assert e.args[1] == Symbol("|") || e.args[1] == Symbol("&")
        if e.args[1] == Symbol("|") 
            return _keep(description, e.args[2]) || _keep(description, e.args[3])
        elseif e.args[1] == Symbol("&")
            return _keep(description, e.args[2]) && _keep(description, e.args[3])
        end
    else
        error("bad expression")
    end
end

"""
$(TYPEDSIGNATURES)

Returns the list of problems descriptions consistent with the expression, as a Tuple of Description, 
see the [list of problems descriptions](@ref descriptions-list) page for details.

# Example

```@example
julia> ProblemsDescriptions(:(:integrator & :energy))
```

# See also 

[`@ProblemsDescriptions`](@ref) for a more natural usage.

!!! note

    You have to define a logical condition with the combination of symbols and the three 
    operators: `!`, `|` and `&`, respectively for the negation, the disjunction and the conjunction.

"""
function ProblemsDescriptions(expr::Union{QuoteNode, Expr})::Tuple{Vararg{Description}}
    problems_list = filter(description -> _keep(description, expr), _problems_without_dummy()) # filter only the problems that contain desc
    return problems_list
end

"""
$(TYPEDSIGNATURES)

Returns the list of problems descriptions consistent with the expression, as a Tuple of Description, 
see the [list of problems descriptions](@ref descriptions-list) page for details.

# Example

```@example
julia> @ProblemsDescriptions :integrator & :energy
```

!!! note

    You have to define a logical condition with the combination of symbols and the three 
    operators: `!`, `|` and `&`, respectively for the negation, the disjunction and the conjunction.

"""
macro ProblemsDescriptions(expr::Union{QuoteNode, Expr})
    return ProblemsDescriptions(expr)
end
macro ProblemsDescriptions()
    return ProblemsDescriptions()
end

"""
$(TYPEDSIGNATURES)

Returns the list of problems consistent with the expression, as a Tuple of OptimalControlProblem, 
see the page [list of problems](@ref problems-list) for details.

# Example

```@example
julia> Problems(:(:integrator & :energy))
```

# See also

[`@Problems`](@ref) for a more natural usage.

!!! note

    You have to define a logical condition with the combination of symbols and the three 
    operators: `!`, `|` and `&`, respectively for the negation, the disjunction and the conjunction.

"""
function Problems(expr::Union{QuoteNode, Expr})::Tuple{Vararg{OptimalControlProblem}}
    problems_list = ProblemsDescriptions(expr)
    return map(e -> OCPDef{e}(), problems_list)   # return the list of problems that match desc
end

"""
$(TYPEDSIGNATURES)

Returns the list of problems consistent with the description, as a Tuple of OptimalControlProblem, 
see the page [list of problems](@ref problems-list) for details.

# Example

```@example
julia> @Problems :integrator & :energy
```

!!! note

    You have to define a logical condition with the combination of symbols and the three 
    operators: `!`, `|` and `&`, respectively for the negation, the disjunction and the conjunction.

"""
macro Problems(expr::Union{QuoteNode, Expr})
    return Problems(expr)
end
macro Problems()
    return Problems()
end