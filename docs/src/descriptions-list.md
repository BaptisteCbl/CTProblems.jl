# [List of problems descriptions](@id descriptions-list)

To print the complete list of problems descriptions:

```@example main
using CTProblems
ProblemsDescriptions()
```

You can use more sophisticated rules to filter. You simply have to define a logical condition with the combination of symbols and the three operators: `!`, `|` and `&`, respectively for the negation, the disjunction and the conjunction.

Here is an example to get the problems descriptions which does not contain `:lagrange`, or contains `:time` (the `or` is not exclusive):

```@example main
@ProblemsDescriptions !:lagrange | :time
```
