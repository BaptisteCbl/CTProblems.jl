using CTProblems
using Test
using MINPACK
using CTFlows
using MLStyle
using CTBase
using LinearAlgebra
using OrdinaryDiffEq
include("utils.jl")

@testset verbose = true showtiming = true "CTProblems" begin
    for name ∈ (
        :double_integrator_consumption,
        :double_integrator_energy_control_constraint,
        :double_integrator_energy_distance,
        :double_integrator_energy_state_constraint,
        :double_integrator_energy,
        :double_integrator_time,
        :goddard_all_constraints,
        :goddard,
        :lqr_ricatti,
        :orbital_transfer_consumption,
        :orbital_transfer_energy,
        :orbital_transfer_time,
        :simple_exponential_consumption,
        :simple_exponential_energy,
        :simple_exponential_time,
        :simple_integrator_energy_free_tf,
        :simple_integrator_lqr_free_tf,
        :simple_integrator_mixed_constraint,
        :simple_integrator_nonsmooth_turnpike,
        :simple_integrator_state_and_control_constraints_nonautonomous,
        :problem,
        :utils,
        )
        @testset "$(name)" begin
            test_name = Symbol(:test_, name)
            include("$(test_name).jl")
            @eval $test_name()
        end
    end
end