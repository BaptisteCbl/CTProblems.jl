function test_utils()

    Interval = CTProblems.Interval
    Intervals = CTProblems.Intervals

    @test 0.5 ∈ Interval(0, 1)
    @test 0.5 ∈ Intervals([Interval(0, 0.5), Interval(0.6, 1)])
    @test 0.5 ∈ Interval(0, 0.1) ∪ Interval(0.5, 1)
    @test 0.5 ∈ Interval(0, 0.1) ∪ (Interval(0.2, 0.4) ∪ Interval(0.5, 1))
    @test 0.5 ∈ (Interval(0, 0.1) ∪ Interval(0.2, 0.4)) ∪ Interval(0.5, 1)
    @test 0.5 ∈ (Interval(0, 0.1) ∪ Interval(0.2, 0.4)) ∪ (Interval(0.5, 0.6) ∪ Interval(0.7, 1))

end