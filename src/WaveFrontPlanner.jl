
mutable struct weights
    # Struct to hold the weights for the reward function

    # w -> reward, incentive to go to high reward areas
    # threshold -> threshold for reward, if reward is below threshold, inflate wavefront value
    w::Float64
    threshold::Float64

end

mutable struct Performance
    # Struct to hold the performance metrics

    # reward_to_dist -> reward to distance ratio
    # wf_update -> number of times the wavefront has to be reset

    reward_to_dist::Float64
    wf_update::Int
    
end

function get_direct_path(start, finish) #For only one object, get the direct path between current and goal position
        
    # Essentially the same as Nolans function, just working for only one objective.
    path = []
    push!(path,start)
    

    while path[end] != finish
        curr = path[end]

        action = get_action(curr,finish)
        push!(path,tuple(curr[1] + action[1], curr[2] + action[2]))
    end

    return path
end

function get_action(start, finish)
    # only for a direct path from start -> goal
    # get the action to take to get to the finish node
    # get the difference between the start and finish nodes
    diff = (finish[1] - start[1], finish[2] - start[2])
    # get the action to take
    action = (0, 0)
    if diff[1] > 0
        action = (1, action[2])
    elseif diff[1] < 0
        action = (-1, action[2])
    end
    if diff[2] > 0
        action = (action[1], 1)
    elseif diff[2] < 0
        action = (action[1], -1)
    end
    return action
end

# Description
# Generates the wavefront between a start and goal node
# Used to define a distance metric to plan over
# Inputs:
# reward -> map_database
# start -> start node
# goal -> goal node
# xVec -> x vector to use
# yVec -> y vector to use
# Output:
# wave_front -> wavefront between start and goal
function get_wave(reward, start, goal, xVec, yVec, obstacles)

    sx, sy = start
    gx, gy = goal

    # Populate wave_front
    wave_front = ones(size(reward))*Inf
    wave_front[xVec, yVec] .= 0

    # Initialize goal 
    wave_front[gx, gy] = 2

    # Loop through and generate wavefront
    for i in xVec
        for j in yVec
            # Need to get next index, based on order of xVec and yVec
            if gx < sx
                # Previous index is + 1
                prior_i = i - 1
            else
                prior_i = i + 1
            end
            if gy < sy
                # Previous index is + 1
                prior_j = j - 1
            else
                prior_j = j + 1
            end

            if i == gx && j == gy
                continue
            elseif j == gy # along y edge, check prior x
                wave_front[i, j] = wave_front[prior_i, j] + 1
            elseif i == gx # along x edge, check prior y
                wave_front[i, j] = wave_front[i, prior_j] + 1 # move along edge
            else
                wave_front[i, j] = 1 + minimum([wave_front[prior_i, j], wave_front[i, prior_j], wave_front[prior_i, prior_j]])
            end
        end
    end

    # Add obstacles to the wavefront (if any)
    wave_front = addObstacle(wave_front, obstacles)
    
    return wave_front

end

function wavefrontPlanner(reward, start, goal,hp,M,obstacles)
    # Inputs: reward, start and goal (x, y) vector sets
    # Outputs: a path that restricts the grid to only the rectangle between start and goal, and maximizes reward

    dist = 0
    totalReward = 0

    sx, sy = start
    gx, gy = goal

    # Get orientation
    if gx > sx
        xVec = gx:-1:sx
    else
        xVec = gx:1:sx
    end

    if gy > sy
        yVec = gy:-1:sy
    else
        yVec = gy:1:sy
    end
    
    # Use wavefront planner but not restricted to follow all points
    wave_front = get_wave(reward, start, goal, xVec, yVec,obstacles)
    
    ## Normalize reward s.t all rewards are between 0 and 1
    reward = reward/maximum(reward)

    # Update wavefront so that it is more incentivized to go to high reward areas
    wave_front = RewardFxn(xVec,yVec,hp,wave_front,reward)
    
    # Now, to motion plan, start at start, and move to the minimum value at each step
    path = []
    # add first point to path
    push!(path, start)

    # Loop through and generate path until the goal is reacheed
    visited = []
    collectedRwrd = [start]

    while path[end] != goal

        # we want to store all visited grid cells so the planner never gets stuck in a loop
        push!(visited,path[end])

        # update curr
        curr = path[end]

        x,y = curr

        # check neighbors are in bounds to prevent errors
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1), (x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)]


        for neighbor in neighbors
            if inBounds(neighbor, wave_front)
                continue
            else
                deleteat!(neighbors, findall(x->x==neighbor,neighbors))
            end
        end

        # Get action should take
        action, M= action_wavefront(wave_front, neighbors,curr,visited,goal,reward,M)

        
        if !(tuple(curr[1] + action[1], curr[2] + action[2]) in collectedRwrd)

            totalReward += reward[path[end][1], path[end][2]]

        end

        # add the action to the path
        push!(path, tuple(curr[1] + action[1], curr[2] + action[2]))
        push!(collectedRwrd, tuple(curr[1] + action[1], curr[2] + action[2]))


    end
    
    dist = length(path)

    M.reward_to_dist = totalReward/dist

    return path, wave_front, M

end

## Description:
# This function is used to get the action to take based on the wavefront
# Inputs:  wave_front -> wavefront between start and goal
#          neighbors -> list of neighbors to the current node  
#          node -> current node
#          visited -> list of visited nodes
#          goal -> goal node
# Outputs: action -> action to take

function action_wavefront(wave_front, neighbors,node,visited,goal,reward,M)

    valid = false

    # Get the wavefront reward associated with the neighbors
    while !valid

        wave_vals = []

        if isempty(neighbors)

            # if all neighbors have been visited, return to the last visited node, clear visited list, and remake the wavefront

            sx, sy = node
            print("Reset - sx: ",sx," sy: ",sy,"\n")
            gx, gy = goal

            # Get orientation
            if gx > sx
                xVec = gx:-1:sx
            else
                xVec = gx:1:sx
            end
            if gy > sy
                yVec = gy:-1:sy
            else
                yVec = gy:1:sy
            end

            # Increment the wavefront update count
            M.wf_update += 1

            # Reset wavefront, visited, and neighbors
            wave_front = get_wave(reward, node, goal, xVec, yVec,obstacles)
            wave_front = addObstacle(wave_front, obstacles)

            reward = reward/maximum(reward[xVec, yVec])

            wave_front = RewardFxn(xVec,yVec,hp,wave_front,reward)

            visited = []
            neighbors = [(sx+1, sy), (sx-1, sy), (sx, sy+1), (sx, sy-1), (sx+1, sy+1), (sx-1, sy-1), (sx+1, sy-1), (sx-1, sy+1)]

            reset = true
            
            continue

        end

        for neighbor in neighbors

            if inBounds(neighbor, wave_front)
                push!(wave_vals, wave_front[neighbor[1], neighbor[2]])
            else
                push!(wave_vals, Inf)
            end

        end
        
        # Now get the minimum value
        min_index = argmin(wave_vals)
        action = (neighbors[min_index][1] - node[1], neighbors[min_index][2] - node[2])
        
        # Ensure the cell has not been visited
        chckNode = node .+ action

        if chckNode in visited
            # if node has been visited or is an obstacle, get a new action and remove that cell from neighbors
            deleteat!(neighbors, findall(x->x==neighbors[min_index],neighbors))
            continue
        else
            return action, M
        end

    end
    
end


function RewardFxn(xVec,yVec,hp,wave_front,reward)
    
    for  i in xVec
        for j in yVec

            # Potentially rethink how we weigh our rewards, reduce hyperparameters
            if reward[i,j] > hp.threshold
                wave_front[i,j] = wave_front[i,j] - (1+reward[i,j])^hp.w
            else
                # TEST - Inflate wave_front more when threshold is not met
                wave_front[i,j] = wave_front[i,j] + (1+reward[i,j])^hp.w
                
                # TEST - Inflate wave_front more when threshold is not met
                #wave_front[i,j] = Inf

                # Original Implementation
                #wave_front[i,j] = wave_front[i,j] - reward[i,j]^hp.w
            end

        end
    end 

    return wave_front
end

function addObstacle(waveFront, obstacles)
    # Add an obstacle to the database
    # obstacle is a list of tuples, where each tuple is a grid cell
    for obstacle in obstacles
        waveFront[obstacle[1], obstacle[2]] = Inf
    end

    return waveFront
end

function inBounds(neighbor, wave_front)

    # Check if the node is in the wavefront bounds

    if wave_front[neighbor[1], neighbor[2]] == Inf
        return false
    else    
        return true
    end

end
