"""
[`CTProblems`](@ref) module.

Lists all the imported modules and packages:

$(IMPORTS)

List of all the exported names:

$(EXPORTS)

"""
module CTProblems

#
using CTBase
using CTFlows
using DocStringExtensions
using MINPACK
using OrdinaryDiffEq
using ForwardDiff
using LinearAlgebra
#

#
include("problem.jl")

# When adding a problem, please follow the nomenclature given in the doc
# on how to add a problem, see https://control-toolbox.github.io/CTProblems.jl.
#
list_of_problems_files = [
    #"double_integrator_consumption.jl",
    "double_integrator_energy_control_constraint.jl",
    "double_integrator_energy_distance.jl",
    "double_integrator_energy_state_constraint.jl",
    "double_integrator_energy.jl",
    "double_integrator_time.jl",
    "goddard_all_constraints.jl",
    "goddard.jl",
    "lqr_ricatti.jl",
    "orbital_transfert_consumption.jl",
    "orbital_transfert_consumption_diff.jl",
    "orbital_transfert_energy.jl",
    "orbital_transfert_time.jl",
    #"simple_exponential_consumption.jl",
    "simple_exponential_energy.jl",
    "simple_exponential_time.jl",
    "simple_integrator_energy_free_tf.jl",
    "simple_integrator_lqr_free_tf.jl",
    "simple_integrator_mixed_constraint.jl",
    "simple_integrator_nonsmooth_turnpike.jl",
    "simple_integrator_state_and_control_constraints_nonautonomous.jl",
]
include("list_of_problems.jl")
include("utils_problems.jl")
#
export ProblemsDescriptions, Problem, Problems, @ProblemsDescriptions, @Problems
export plot

end # module CTProblems
