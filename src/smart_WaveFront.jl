#=

The purpose of this file is to house all te functions required to intelligently expand the wavefront so that it can
encompass more of the environment and allow the agent to find a higher reward path to the goal.

=#

include("WaveFrontGen.jl")

function expand_Wavefront(wave_front,reward,goal, start,xVec, yVec)

    # Determine how much and the direction to expand the wavefront

    AR = get_AR(xVec, yVec)
    println("Aspect Ratio: ", AR)
    sx, sy = start
    gx, gy = goal

    while AR > 2.75 || AR < 0.6
        if AR > 2.75 # Wavefront is too long, expand vertically
            println("Expanding wavefront vertically ", AR)
            if gy > sy

                if gy < size(reward,2)
                    gy += 1
                end

                if sy > 2 
                    sy -= 1
                end
            
                yVec = gy:-1:sy
            else
                if gy > 2
                    gy -= 1
                end

                if sy < size(reward,2)
                    sy += 1
                end
      
                yVec = gy:1:sy

            end
        elseif AR < 0.6 # Wavefront is too tall, expand horizontally
            println("Expanding wavefront horizontally ", AR)
            if gx > sx

                if gx < size(reward,1)
                    gx += 1
                end

                if sx > 2
                    sx -= 1
                end

                xVec = gx:-1:sx

            else

                if gx > 2
                    gx -= 1
                end

                if sx < size(reward,1)
                    sx += 1
                end
            
                xVec = gx:1:sx
            end
        end

        AR = get_AR(xVec, yVec)

    end
    println("Final Aspect Ratio: ", AR)
    wave_front,~,~ = get_wave(reward, s0, goal, xVec, yVec, obstacles)

    return wave_front, xVec, yVec
end

function get_AR(xVec, yVec)
    # Get the aspect ratio of the wavefront

    # For large aspect ratios (>> 1), the wave_front is long and skinny, which is not ideal.
    # For small aspect ratios (<< 1), the wavefront is tall and narrow, which is also not ideal.
    # We want the aspect ratio to be close to 1, which means the wavefront is roughly square. 
    println("WV size: ", size(xVec,1), " ",size(yVec,1))
    return size(xVec,1) / size(yVec,1) # Width / Height

end