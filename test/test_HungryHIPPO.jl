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
goal = [(19, 83),(30,67),(41,45),(38,20),(30,9),(17,12)]
 
# Start node
s0 = (24, 100)
 
#hyperparameters struct, hp
hp = weights(1.5,0.8)

# Performance Metric struct, M
M = Performance(0.0,0)

#obstacles = [(27,72),(27,73),(27,74),(27,75),(27,76),(27,77),(26,72),(26,73),(26,74),(26,75),(26,76),(26,77)]
obstacles = []

X = []
RperD = []
Replans = []

"""
for i in 0.5:0.1:13

    hp.w = i
    M = Performance(0.0,0)

    path, wave_front, M = wavefrontPlanner(db,s0,db.ID2grid[endpt],hp,M,obstacles)

    push!(X,hp.w)
    push!(RperD,M.reward_to_dist)
    push!(Replans,M.wf_update)

end


plotMetrics(X,RperD,"Reward per Distance")
plotMetrics(X,Replans, "# of WaveFront Replans")

"""

reward = db.reward

path, M = wavefrontPlanner(reward,s0,goal,hp,M,obstacles)

#print("Reward per Distance: ", M.reward_to_dist, "\n")
#print("# of replans: ", M.wf_update, "\n")

goal = [(19, 83),(30,67),(41,45),(38,20),(30,9),(17,12)]

#visualizeWaveFront(s0,goal,wave_front,db,path)
visualizeRewardMap(s0,goal,db,path)

