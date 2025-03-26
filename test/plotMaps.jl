include("visualize.jl")
include("./../RINAO.jl/test/evaluateOperatorData.jl")

# Plotting reward map for each data set

toPlot = false

S1 = "./../RINAO.jl/operator_data/Brainard/Brainard_S1.jl"
S2 = "./../RINAO.jl/operator_data/Brainard/Brainard_S2.jl"
S3 = "./../RINAO.jl/operator_data/Brainard/Brainard_S3.jl"
S4 = "./../RINAO.jl/operator_data/Brainard/Brainard_S4.jl"
S5 = "./../RINAO.jl/operator_data/Brainard/Brainard_S5.jl"
S6 = "./../RINAO.jl/operator_data/Brainard/Brainard_S6.jl"
S7 = "./../RINAO.jl/operator_data/Brainard/Brainard_S7.jl"
S8 = "./../RINAO.jl/operator_data/Brainard/Brainard_S8.jl"
S9 = "./../RINAO.jl/operator_data/Brainard/Brainard_S9.jl"
S10 = "./../RINAO.jl/operator_data/Brainard/Brainard_S10.jl"


#include(S1)
#db, vars, inputs = processJSONInputs(inputJSON, toPlot)
#visualizeRewardMap((1,1),(1,1),db,[])


include(S2)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])

```
include(S3)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])

include(S4)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])

include(S5)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])

include(S6)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])

include(S7)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])

include(S8)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])

include(S9)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])

include(S10)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)
visualizeRewardMap((1,1),(1,1),db,[])
```