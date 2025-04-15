
#=
The purpose of this file is to house all the required functions to generate a wavefront which can then be used in the wavefront planner.

New wavefront implementation will allow the wavefront to dynamically resize when it gets stuck and connot get around an obstacle.

This will also allow me to test the new wavefront more directly and not have to worry about the planner itself.
=#


function get_wave(reward, start, goal, xVec, yVec, obstacles)

    sx, sy = start
    gx, gy = goal

    # Populate wave_front
    wave_front = ones(size(reward))*Inf
    wave_front[xVec, yVec] .= 0

    # Use BFS to generate wavefront
    wave_front = gen_wave_bfs(goal, wave_front)
    # Add obstacles to the wavefront (if any)
    wave_front = addObstacle(wave_front, obstacles)
    made = possiblePath(wave_front,start,goal)

    while !made

        # Expand the wavefront - get orientation to expand WF
        if gx > sx
            if gx > 2
                gx += 1
            end
            if sx < size(reward,1)
                sx -= 1
            end

            
            xVec = gx:-1:sx
        else
            if gx < size(reward,1)
                gx -= 1
            end
            if sx > 2
                sx += 1
            end
            
            xVec = gx:1:sx
        end
    
        if gy > sy

            if gy < size(reward,2)
                gy += 1
            end
            if sy > 2
                sy -= 1
            end
            
            yVec = gy:-1:sy
        else
            if gy >2
                gy -= 1
            end
            if sy < size(reward,2)
                sy += 1
            end
      
            yVec = gy:1:sy
        end
        
        # Populate wave_front
        wave_front = ones(size(reward))*Inf
     
        wave_front[xVec, yVec] .= 0

        wave_front = gen_wave_bfs(goal, wave_front)
        wave_front = addObstacle(wave_front, obstacles)
        # Check if a path is possible, keep expanding if not
        made = possiblePath(wave_front,start,goal)
        
    end

    
    return wave_front

end


function gen_wave_bfs(goal, waveFront)
    # BFS implementation of wavefront generation

    waveFront[goal[1], goal[2]] = 2
    queue = [goal]
    visited = []
    actions = [(0,1), (0,-1), (1,0), (-1,0),(1,1), (-1,1), (1,-1), (-1,-1)] 

    while !isempty(queue)

        cell = popfirst!(queue)

        if !BoundCheck(cell, waveFront)
            continue
        end

        for (dx, dy) in actions
            neighbor = (cell[1] + dx, cell[2] + dy)

            if isValid(cell,waveFront) && isFree(cell,waveFront) && !(neighbor in visited)
                
                push!(visited, neighbor)
                push!(queue, neighbor)

                if BoundCheck(neighbor, waveFront) && isValid(neighbor,waveFront) && isFree(neighbor,waveFront)
                    # Check if neighbor is a valid cell and not an obstacle
                    waveFront[neighbor[1], neighbor[2]] = waveFront[cell[1], cell[2]] + 1
                end

            end
        end
    end

    waveFront[goal[1], goal[2]] = 2

    return waveFront

end

function addObstacle(waveFront, obstacles)
    # Add an obstacle to the database
    # obstacle is a list of tuples, where each tuple is a grid cell
    for obstacle in obstacles
        waveFront[obstacle[1], obstacle[2]] = 1
    end

    return waveFront
end

function possiblePath(waveFront,start,goal)
    # Check if a path is possible between start and goal using a BFS

    queue = [start]
    visited = []
    made = false
    actions = [(0,1), (0,-1), (1,0), (-1,0),(1,1), (-1,1), (1,-1), (-1,-1)] 

    while !isempty(queue)

        cell = popfirst!(queue)

        if !BoundCheck(cell,waveFront)
            continue
        end

        if cell == goal
            made = true
            break
        end

        for (dx, dy) in actions
            neighbor = (cell[1] + dx, cell[2] + dy)

            if isValid(cell,waveFront) && isFree(cell,waveFront) && !(neighbor in visited)
                
                push!(visited, neighbor)
                push!(queue, neighbor)

                
            end
        end

    end

    return made

end

function isValid(cell,waveFront)
    return (waveFront[cell[1],cell[2]] != Inf)
end

function isFree(cell, waveFront)
    # Returns true if not an obstacle, false if it is
    return waveFront[cell[1],cell[2]] != 1
end

function BoundCheck(node,waveFront)
    
    if node[1] < 1 || node[1] > size(waveFront,1)  || node[2] < 1 || node[2] > size(waveFront,2)
        return false
    end

    return true
end

