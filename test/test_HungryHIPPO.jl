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
waypoints = [(19, 83),(30,67),(41,45),(38,20),(30,9),(17,12)]
 
# Start node
start = (24, 100)
 
#hyperparameters struct, hp
hp = weights(1.5,0.8)


obstacles = []

for i in 35:1:45
    push!(obstacles, (i, 33))
    push!(obstacles, (i, 34))
    push!(obstacles, (i, 35))
end

reward = db.reward


path = wavefrontPlanner(reward,start,goal,hp,obstacles)

visualizeRewardMap(s0,goal,db,path,obstacles)

