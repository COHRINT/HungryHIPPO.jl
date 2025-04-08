using PlotlyJS

function visualizeWaveFront(start,goal,obstacles,waveFront)

    # Create a heatmap using PlotlyJS
    z=waveFront
    grid_size = size(waveFront)
    
    sc = 1

    X = collect(1:sc:grid_size[2])

    # Invert Y axis to match grid
    Y = -collect(1:sc:grid_size[1])
    
    # Create the heatmap trace
    heatmap_trace = PlotlyJS.heatmap(x=X, y=Y, z=z, colorscale="Viridis", opacity=0.6)

    # Create scatter traces for start and goal nodes
    start_trace = PlotlyJS.scatter(x=[X[start[2]]], y=[Y[start[1]]], mode="markers", marker_color="green", name="Start")
    
    # Create scatter trace for the goal
    yG = goal[1]
    xG = goal[2]

    goal_trace = PlotlyJS.scatter(x=xG, y=-yG, mode="markers", marker_color="red", name="Goal")
    
    # Obstacle Trace
    if length(obstacles) > 0
        xO = [obstacles[i][2] for i in eachindex(obstacles)]
        yO = [obstacles[i][1] for i in eachindex(obstacles)]

        obstacle_trace = PlotlyJS.scatter(x=xO, y=-yO, mode="markers", marker_color="black", name="Obstacle")
    else
        obstacle_trace = PlotlyJS.scatter(x=[], y=[], mode="markers", marker_color="black", name="Obstacle")
    end

    # Combine traces into a plot
    plot_data = [heatmap_trace, start_trace, obstacle_trace,goal_trace]

    # Create the layout
    layout = PlotlyJS.Layout(title="WaveFront Visualization", xaxis_title="X", yaxis_title="Y")

    # Create the plot
    plot = PlotlyJS.Plot(plot_data, layout)

    # Display the plot
    PlotlyJS.display(plot)

end


function visualizeRewardMap(start,goal,db,path,obstacles)
    
    # Create a heatmap using PlotlyJS
    z=db.reward
    grid_size = size(db.reward)
    
    sc = 1

    X = collect(1:sc:grid_size[2])
    Y = -collect(1:sc:grid_size[1])
    
    
    heatmap_trace = PlotlyJS.heatmap(x=X, y=Y, z=z, colorscale="Jet", opacity=0.6)

    # Create scatter traces for start and goal nodes
    start_trace = PlotlyJS.scatter(x=[X[start[2]]], y=[Y[start[1]]], mode="markers", marker_color="red", name="Start")
    
    # Create scatter trace for the goal
    yG = [goal[i][1] for i in eachindex(goal)]
    xG = [goal[i][2] for i in eachindex(goal)]

    goal_trace = PlotlyJS.scatter(x=xG, y=-yG, mode="markers", marker_color="green", name="Goal")

    xP = [path[i][1] for i in eachindex(path)]
    yP = [path[i][2] for i in eachindex(path)]

    path_trace = PlotlyJS.scatter(x=yP, y=-xP, mode="markers", marker_color="purple", name="Path")

    xO = [obstacles[i][2] for i in eachindex(obstacles)]
    yO = [obstacles[i][1] for i in eachindex(obstacles)]

    obstacle_trace = PlotlyJS.scatter(x=xO, y=-yO, mode="markers", marker_color="black", name="Obstacle")
    # Combine traces into a plot
    plot_data = [heatmap_trace, path_trace,start_trace, obstacle_trace, goal_trace]

    # Create the layout
    layout = PlotlyJS.Layout(title="Reward Map Visualization", xaxis_title="X", yaxis_title="Y")

    # Create the plot
    plot = PlotlyJS.Plot(plot_data, layout)

    # Display the plot
    PlotlyJS.display(plot)
    
    
end

function plotMetrics(X,Y,STD,title)

    # Create a scatter plot using PlotlyJS
    scatter_trace = PlotlyJS.scatter(x=X, y=Y, mode="lines+markers", error_y=attr(
        type="data",
        array=2 .* STD,  # 2σ bounds
        visible=true,
        color="rgb(100,100,100)",
        thickness=1,
        width=3
    ),marker_color="blue", name=title)

    # Combine traces into a plot
    plot_data = [scatter_trace]

    # Create the layout
    layout = PlotlyJS.Layout(title=title, xaxis_title="Weight", yaxis_title=title)

    # Create the plot
    plot = PlotlyJS.Plot(plot_data, layout)

    # Display the plot
    PlotlyJS.display(plot)
    
end