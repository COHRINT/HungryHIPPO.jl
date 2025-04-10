include("../src/WaveFrontPlanner.jl")
include("visualize.jl")
include("./../RINAO.jl/test/evaluateOperatorData.jl")

# Testing/Debugging HungryHIPPO
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

include(S2)
db, vars, inputs = processJSONInputs(inputJSON, toPlot)

# List of ordered waypoints for the planner to visit
goal = [(19, 83),(26,80),(27,61),(37,63),(56,71)]
 
# Start node
s0 = (24, 100)
 
#hyperparameters struct, hp
hp = weights(1.5,0.8)


obstacles = []

for i in 23:1:29
    push!(obstacles, (i, 70))
    push!(obstacles, (i, 71))
    push!(obstacles, (i, 72))
end

reward = db.reward
print(goal)
path = wavefrontPlanner(reward,s0,goal,hp,obstacles)

visualizeRewardMap(s0,goal,db,path,obstacles)

