struct Interval
    a::Real
    b::Real
end

Base.:(∈)(x::Real, I::Interval) = I.a ≤ x ≤ I.b

struct Intervals
    val::Vector{Interval}
end

Base.:(∈)(x::Real, I::Intervals) = any(x ∈ Ii for Ii in I.val)

Base.:(∪)(I1::Interval, I2::Interval) = Intervals([I1, I2])
Base.:(∪)(I1::Interval, I2::Intervals) = Intervals([I1, I2.val...])
Base.:(∪)(I1::Intervals, I2::Interval) = Intervals([I1.val..., I2])
Base.:(∪)(I1::Intervals, I2::Intervals) = Intervals([I1.val..., I2.val...])