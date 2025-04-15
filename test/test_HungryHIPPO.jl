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
goal = [(1,1),(1,103),(66,103),(66,1)]
 
# Start node
s0 = (43, 55)
 
#hyperparameters struct, hp
hp = weights(5.5,0.5)

obstacles = []

reward = db.reward


path = wavefrontPlanner(reward,s0,goal,hp,obstacles)

visualizeRewardMap(s0,goal,db,path,obstacles)

