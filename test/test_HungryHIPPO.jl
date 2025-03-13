include("../src/WaveFrontPlanner.jl")
include("visualize.jl")
include("../RINAO.jl/test/evaluateOperatorData.jl")

# Testing/Debugging HungryHIPPO

# Save only one user input for end pos
endpt = inputs.user_points_p[30]
 
# Start node
s0 = (20, 85)
 
#hyperparameters struct, hp
hp = weights(5.3,0.8)

# Performance Metric struct, M
M = Performance(0.0,0)

obstacles = [(27,72),(27,73),(27,74),(27,75),(27,76),(27,77),(26,72),(26,73),(26,74),(26,75),(26,76),(26,77)]


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

path, wave_front, M = wavefrontPlanner(reward,s0,db.ID2grid[endpt],hp,M,obstacles)

#print("Reward per Distance: ", M.reward_to_dist, "\n")
#print("# of replans: ", M.wf_update, "\n")

#visualizeWaveFront(s0,db.ID2grid[endpt],wave_front,db,path)
visualizeRewardMap(s0,db.ID2grid[endpt],db,path)

