# Lab 6: Motion Planning

## Video Links and Explaination

## RRT

The RRT Planner works, but is highly suboptimal. The targetted point jumps around erratically making it difficult for the pure pursuit controller to perform well, as there is no path shortening or optimization ran on the determined route. Also, I am using a gap following approach to initial target selection (red marker in the video) as I wanted to aim for complete autonomous sytem. The planner would likely work even better if there was a human defined trajectory. Still, the planner always works in determining a collision free path. To respect the cars geometry during planning, the boundaries of objects that the car can collide with as determined by the lidar scan are eroded when formulating the occupancy grid (shown as the black and white mask in the vido). From the vanilla RRT formulation was implemented locally in the car's body frame.

[Handling Obstacles - RRT Demo: Drive Disabled](https://youtu.be/K-el0yRh-Es) NOTE: the path marker is laggy. That is why it looks like the target point (in green) is jumping around.
[Handling Obstacles - RRT Demo: Drive Enabled](https://youtu.be/4J6OYn2O3II)

## RRT*
[Handling Obstacles - RRT* Demo: Drive Disabled](https://youtu.be/sYkw6UtEe6g) 
[Handling Obstacles - RRT* Demo: Drive Enabled](https://youtu.be/qu1KqwrRdRk)

[RRT* around levine loop (no obstacles)](https://youtu.be/k1heeUMh4ks)
