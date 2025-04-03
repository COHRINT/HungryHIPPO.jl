# HungryHIPPO

Path planning code for the RINAO/HIPPO projects. Computes a path between two cells using various approches. 
# Usage

## wavefrontPlanner
Computes a path using a wavefront approach, ensuring the drone navigates around obstacles (Keep-Out Zones).

Function:

`wavefrontPlanner(reward, start, waypoints,hp,obstacles) -> path`

Inputs:

`reward:Matrix{Float64}` grid value rewards. Each index corresponds to a cell in the reward map.

`start::Tuple{Int64, Int64}` Start node (coordinates) 

`waypoints::Vector{Tuple{Int64, Int64}}` Ordered list of waypoints for the planner to visit.

`hp:` [Struct](#hp-struct) containing hyperparameters for the path planner.

`M:` [Struct](#performance-struct) containing performance metrics that are updated throughout wavefrontPlanner.

`obstacles::Vector{Tuple{Int64, Int64}}` List containing obstacle coordinates that the planner must avoid.

Output:

`path::Vector{Any}` A list of coordinates representing the computed path. Typically each index in path is a tuple of integers, Tuple{Int64, Int64}.

`wave_front::Matrix{Float64}` Matrix with each index corresponding to a grid cell on the map. Outputted purely for plotting purposes.  

`M::Performance` Returns updated scruct of performance metrics.

## get_direct_path
Computes a direct path between the start and goal nodes. Does not consider any obstacles or the reward map.

Function:

`get_direct_path(start,finish) -> path`

Inputs:

`start::Tuple{Int64, Int64}` Start node (coordinates) 

`finish::Tuple{Int64, Int64}` End node (coordinates)

Outputs:

`path::Vector{Any}` A list of coordinates representing the computed path.Typically each index in path is a tuple of integers, Tuple{Int64, Int64}.


# weights Struct

A struct that holds tuning parameters for the wavefront path planner.

## Fields

`hp.w::Float64` : Determine how much extra weight to give rewards. A higher value increase the preference for high-reward paths.

`hp.threshold::Float64` : A threshold in the range [0,1] that defines what is considered a "large" reward. Values closer to 1 make the planner more selective on high-reward paths.

# Performance Struct

A struct that holds performance metrics to measure the performace of the planner algorithm. The struct should be initialized with all fields set to 0.

## Fields

`reward_to_dist::Float64` : A ratio of total collected reward per cell visited. Cells visited more than once do not count towards the reward total.

`wf_update::Int` : A count of how many times the planner was forced to recreate the wavefront. The wavefront is recreated each time the planner gets stuck.

