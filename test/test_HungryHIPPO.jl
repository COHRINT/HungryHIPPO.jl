include("../src/WaveFrontPlanner.jl")
include("../RINAO.jl/test/evaluateOperatorData.jl")

# Main
save_plot = false

# Save only one user input for end pos
endpt = inputs.user_points_p[30]
 
# Start node
s0 = (20, 85)
 
#direct = get_direct_path(s0,db.ID2grid[endpt])
hp = weights(3.8,0.8)

obstacles = [(27,72),(27,73),(27,74),(27,75),(27,76),(27,77),(26,72),(26,73),(26,74),(26,75),(26,76),(26,77)]
 
X = []
Y = []
 
#for i in 0.5:0.1:10

#    hp.w1 = i

#    waveFront, wave_front,M1 = sub_max_restrict(db,s0,db.ID2grid[endpt],hp,obstacles)

#    push!(X,hp.w1)
#    push!(Y,M1)

#end

#plotMetrics(X,Y)

path = wavefrontPlanner(db,s0,db.ID2grid[endpt],hp,obstacles)

#visualizeWaveFront(s0,db.ID2grid[endpt],wave_front,db,path)
#visualizeRewardMap(s0,db.ID2grid[endpt],db,path)

#plot_value_grid_pathing(db, inputs, grid_image, "Direct_Path_S2", points_p = endpt,path = direct, start_node=db.grid2ID[s0],save_plt=save_plot)
#plot_value_grid_pathing(db, inputs, grid_image, "Wave_Restricted_PathObstacles_S2", points_p = endpt,path = waveFront, start_node=db.grid2ID[s0],obstacles=obstacles,save_plt=save_plot)