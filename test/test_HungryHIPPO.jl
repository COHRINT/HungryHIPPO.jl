include("../src/WaveFrontPlanner.jl")
include("visualize.jl")


# Testing/Debugging HungryHIPPO

reward = rand(120,120)

# List of ordered waypoints for the planner to visit
goal = [(12,13),(10, 17), (11,19),(13,19),(13,22)]
 
# Start node
s0 = (8,6)
 
#hyperparameters struct, hp
hp = weights(1.75,0.75)

obstacles = [(6,11),(7,11),(8,11),(9,11),(10,11),(11,11),(12,11),(13,11),(9,18),(10,18),(11,18),(12,18)]

reward = reward[1:25,1:25]


path = wavefrontPlanner(reward,s0,goal,hp,obstacles)
println("Path: ", path)
visualizeRewardMap(s0,goal,reward,path,obstacles)

