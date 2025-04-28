include("../src/WaveFrontPlanner.jl")
include("visualize.jl")
include("./../RINAO.jl/test/evaluateOperatorData.jl")

# Testing/Debugging HungryHIPPO

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
goal = [(2,6)]
 
# Start node
s0 = (6,6)
 
#hyperparameters struct, hp
hp = weights(1.75,0.75)

obstacles = [(1,1),(1,2),(1,3),(1,4),(1,5),(1,6),(1,7),(1,8),(1,9),(1,10),(2,10),(3,10),(4,10),(5,10),(6,10),(7,10),(8,10),(9,10),(10,10),(2,1),(3,1),(4,1),(5,1),(6,1),(7,1),(8,1),(9,1),(10,1),(10,2),(10,3),(10,4),(10,5),(10,6),(10,7),(10,8),(10,9),(4,4),(4,5),(4,6),(4,7),(4,8),(5,4),(5,5),(5,6),(5,7),(5,8),(4,3),(5,3),(5,2),(4,2)]

reward = db.reward[1:10,1:10]


path = wavefrontPlanner(reward,s0,goal,hp,obstacles)

visualizeRewardMap(s0,goal,db,path,obstacles)

