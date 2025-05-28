
include("WaveFrontGen.jl")
include("smart_WaveFront.jl")

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


function wavefrontPlanner(reward, start, goals,hp,obstacles)
    # Inputs: reward, start and goal (x, y) vector sets
    # Outputs: a path that restricts the grid to only the rectangle between start and goal, and maximizes reward

    path = []

    if size(goals) == 0
        return path
    end

    ## Normalize reward s.t all rewards are between 0 and 1
    reward = reward/maximum(reward)

    for goal in goals

        if goal in obstacles
            continue
        end

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
    
        # Use wavefront planner to generate the wavefront between start and goal
        wave_front, reward, direct = get_wave(reward, start, goal, xVec, yVec,obstacles)
        

        if direct
            # If the path is direct, just get the direct path and append it to the path
            path_direct = get_direct_path(start, goal)
            append!(path,path_direct)
            start = goal
            continue
        end

        wave_front, xVec, yVec = expand_Wavefront(wave_front,reward,goal, s0,xVec, yVec)
        
        # Update wavefront so that it is more incentivized to go to high reward areas
        wave_front = RewardFxn(xVec,yVec,hp,wave_front,reward)
    
        # Now, to motion plan, start at start, and move to the minimum value at each step
        # add first point to path
        push!(path, start)

        # Loop through and generate path until the goal is reacheed
        visited = []

        while path[end] != goal

            # we want to store all visited grid cells so the planner never gets stuck in a loop
            push!(visited,path[end])

            # update curr
            curr = path[end]
            println("Current node: ", curr)
            x,y = curr

            # check neighbors are in bounds to prevent errors
            unchecked_neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1), (x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)]
            neighbors = []

            # Only keep neighbors that are in the flight region and not obstacles
            for neighbor in unchecked_neighbors
                if inBounds(neighbor, wave_front)
                    push!(neighbors, neighbor)
                else
                    continue
                end
            end

            # Get action should take
            action = action_wavefront(wave_front, neighbors,curr,visited,goal,reward,hp,obstacles)

            # action = (0,0) means that the planner replanned, and could not find a path to the goal, thus returning a direct path as a fail safe
            if action == (0,0)
                path_direct = get_direct_path(curr, goal)
                append!(path,path_direct)
                break
            end

            # make the action and move the drone to the new position
            push!(path, tuple(curr[1] + action[1], curr[2] + action[2]))

        end

        start = goal

    end
    
    # Need to add a third index to the path for altitude control in HIPPO
    temp_path = []
    for p in path
        push!(temp_path, (p[1], p[2], 0))
    end
    path = temp_path
    return path

end

## Description:
# This function is used to get the action to take based on the wavefront
# Inputs:  wave_front -> wavefront between start and goal
#          neighbors -> list of neighbors to the current node  
#          node -> current node
#          visited -> list of visited nodes
#          goal -> goal node
# Outputs: action -> action to take

function action_wavefront(wave_front, neighbors,node,visited,goal,reward, hp, obstacles)

    valid = false
    direct = false
    reset = false
    # Get the wavefront reward associated with the neighbors
    while !valid

        wave_vals = []

        # If the neighbors are empty, we need to reset the wavefront and neighbors -> replan
        if isempty(neighbors)

            # if all neighbors have been visited, return to the last visited node, clear visited list, and remake the wavefront
            if reset
                return (0,0)
            end
            reset = true
            visited = []
            wave_front, neighbors, direct = resetWave(reward, node, goal, hp, obstacles)
            continue

        end

        if direct
            return (0,0)
        end

        for neighbor in neighbors
            push!(wave_vals, wave_front[neighbor[1], neighbor[2]])
        end
        
        # Now get the minimum value
        min_index = argmin(wave_vals)
        action = (neighbors[min_index][1] - node[1], neighbors[min_index][2] - node[2])
        
        # Ensure the cell has not been visited
        chckNode = node .+ action

        if chckNode in visited
            # if node has been visited, get a new action and remove that cell from neighbors
            deleteat!(neighbors, findall(x->x==neighbors[min_index],neighbors))
            continue
        else
            return action
        end

    end
    
end

function resetWave(reward, node, goal, hp, obstacles)

    println("Resetting wavefront")
    # Reset the wavefront and neighbors

    sx, sy = node
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

    # Reset wavefront, visited, and neighbors
    wave_front, reward, direct = get_wave(reward, node, goal, xVec, yVec,obstacles)

    reward = reward/maximum(reward[xVec, yVec])

    wave_front = RewardFxn(xVec,yVec,hp,wave_front,reward)

    unchecked_neighbors = [(sx+1, sy), (sx-1, sy), (sx, sy+1), (sx, sy-1), (sx+1, sy+1), (sx-1, sy-1), (sx+1, sy-1), (sx-1, sy+1)]
    neighbors = []

    for neighbor in unchecked_neighbors

        if inBounds(neighbor, wave_front)
            push!(neighbors, neighbor)
        else
            continue
        end

    end
    
    return wave_front, neighbors, direct
end

function RewardFxn(xVec,yVec,hp,wave_front,reward)
    
    for  i in xVec
        for j in yVec

            if wave_front[i,j] == 1
                continue
            end
            
            # Potentially rethink how we weigh our rewards, reduce hyperparameters
            if reward[i,j] > hp.threshold
                wave_front[i,j] = wave_front[i,j] - (1+reward[i,j])^hp.w
            else
                #Inflate wave_front more when threshold is not met
                wave_front[i,j] = wave_front[i,j] + (1+reward[i,j])^hp.w
                
            end

        end
    end 

    return wave_front
end


function inBounds(neighbor, wave_front)

    # Check if the node is in the wavefront bounds
    if neighbor[1] < 1 || neighbor[1] > size(wave_front, 1) || neighbor[2] < 1 || neighbor[2] > size(wave_front, 2)
        return false
    end

    if wave_front[neighbor[1], neighbor[2]] == Inf || wave_front[neighbor[1], neighbor[2]] == 1
        # Check if the node is an obstacle or has a wavefront value of 0
        return false
    else
           
        return true
    end

end
