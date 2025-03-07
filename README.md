# HungryHIPPO

Path planning code for the RINAO/HIPPO projects. Computes a path between two cells using various approches.

# Usage

## wavefrontPlanner
Computes a path using a wavefront approach, ensuring the drone navigates around obstacles (Keep-Out Zones).

Function:

`wavefrontPlanner(db, start, goal,hp,obstacles) -> path`

Inputs:

`db:` map_database struct

`start::Tuple{Int64, Int64}` Start node (coordinates) 

`goal::Tuple{Int64, Int64}` End node (coordinates)

`hp:` [Struct](#hp-struct) containing hyperparameters for the path planner .

`obstacles::Vector{Tuple{Int64, Int64}}` List containing obstacle coordinates that the planner must avoid.

Output:

`path::Vector{Any}` A list of coordinates representing the computed path. Typically each index in path is a tuple of integers, Tuple{Int64, Int64}.


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
