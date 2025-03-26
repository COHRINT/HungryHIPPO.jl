include("../src/WaveFrontPlanner.jl")
include("visualize.jl")
include("../RINAO.jl/test/evaluateOperatorData.jl")

using Statistics

"""
This scripts purpose will be to run MC tests on the wavefront planner, varying both the start and the goal nodes randomly
for n MC runs. After n MC runs, the program will change the weight hyperparameter and run n MC runs again. This should
give us a good idea of how the wavefront planner performs under different conditions for a given weight.

This will be expanded to include different reward maps as well.

Obstacles??
"""

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

C1 = [(22,1),(15,1),(1,50),(54,34),(13,1),(46,35),(47,12),(20,1),(13,1),(22,1)]
C2 = [(55,51),(49,103),(50,103),(1,103),(55,64),(8,103),(1,75),(66,80),(43,103),(48,73)]
S = [S1,S2,S3,S4,S5,S6,S7,S8,S9,S10]
S_str = ["S1","S2","S3","S4","S5","S6","S7","S8","S9","S10"]

# Define the number of MC runs
n = 100
obstacles = []
hp = weights(0.5,0.8)

for k in 1:1:10

    print("Running MC test for ",S[k],"\n")
    include(S[k])
    db, vars, inputs = processJSONInputs(inputJSON, toPlot)
    reward = db.reward

    # Define the rectangles of possible start and goal nodes, with the center
    # being the location of the maximum reward
    max_reward, max_coords = findmax(reward)

    max_coords = tuple(max_coords[1],max_coords[2])

    print("Max Coords: ",max_coords,"\n")

    # Spread out 10 units from max_coords, until we hit the edge of the reward map
    mx, my = max_coords
    c1 = max_coords
    c2 = max_coords


    for i=1:1:10
        if mx+i > size(reward)[1] || my+i > size(reward)[2] 
            break
        end
        c1 = c1 .+ (1,1)  
    end

    for i=1:1:10
        if mx-i < 1 || my-i < 1 
            break
        end
        c2 = c2 .- (1,1)  
    end

    # Define the range of possible start and goal nodes
    if c1[1] < c2[1]
        x = collect(c1[1]:1:c2[1])
    else
        x = collect(c2[1]:1:c1[1])
    end

    if c1[2] < c2[2]
        y = collect(c1[2]:1:c2[2])
    else 
        y = collect(c2[2]:1:c1[2])
    end
    
    Coords = []

    for px in x
        for py in y
            push!(Coords,tuple(px,py))
        end
    end

    
    # Store results
    X = []
    meanRpD = []
    stdRpD = []

    # First loop will be where we vary the hyperparameter, hp.w
    for i in 0.5:0.1:10

        #hyperparameters struct, hp
        hp.w = i

        print("Weight: ",i,"\n")


        # List of r_per_d for each MC run
        RperD = []

        for j in 1:n # MC runs

            goal = rand(Coords)
            start = rand(Coords)

            while(goal == start)
                goal = rand(Coords)
            end

            M = Performance(0.0,0)
            path, wave_front, M = wavefrontPlanner(reward,start,goal,hp,M,obstacles)
            push!(RperD,M.reward_to_dist)

        end


        push!(X,i)
        push!(meanRpD,mean(RperD))
        push!(stdRpD,std(RperD))


    end
    title = "Data Set: ",S_str[k],", Reward per Distance for ",n," MC runs"
    
    plotMetrics(X,meanRpD,stdRpD,title)

    
end
