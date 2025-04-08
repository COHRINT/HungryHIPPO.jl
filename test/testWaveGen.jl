include("visualize.jl")
include("./../RINAO.jl/test/evaluateOperatorData.jl")
include("../src/WaveFrontGen.jl")

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

goal = (30,67)
s0 = (24, 100)


obstacles = []

for i in 20:1:36
    push!(obstacles, (i, 82))
    push!(obstacles, (i, 83))
    push!(obstacles, (i, 84))
end


reward = db.reward

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


waveFront = get_wave(reward, s0, goal, xVec, yVec, obstacles)
visualizeWaveFront(s0,goal,obstacles,waveFront)




