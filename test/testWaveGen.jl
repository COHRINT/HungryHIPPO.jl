include("visualize.jl")
#include("C:/Users/bijan/Documents/COHIRNT/RINAO.jl/test/evaluateOperatorData.jl")
include("../src/WaveFrontGen.jl")
include("../src/WaveFrontPlanner.jl")
include("../src/smart_WaveFront.jl")


goal = (80,23)
s0 = (100, 24)

reward = rand(120,120)

obstacles = []


#reward = db.reward

sx = s0[1]
sy = s0[2]
gx = goal[1]
gy = goal[2]


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

hp = weights(1.5,0.6)

waveFront,reward,direct = get_wave(reward, s0, goal, xVec, yVec, obstacles)
#println(waveFront)
waveFront, xVec, yVec = expand_Wavefront(waveFront,reward,goal, s0,xVec, yVec)
waveFront = RewardFxn(xVec,yVec,hp,waveFront,reward)
visualizeWaveFront(s0,goal,obstacles,waveFront)




