# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: The command ```source /opt/ros/foxy/setup.bash``` ensure that the installed ROS2 Foxy tools and libraries in the path and available via terminal commands. This does not, however, include any custom local packages which have been built. To make these new packages available, the workspace within which the packages are housed must be sourced after building them (such as using 'colcon build'). To do this, the command ```source install/setup.bash``` is used.

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer: The queue_size argument controls the size of the message queue. In the case where messages are published to a topic faster than can be processed, the amount of messages specified by the queue_size argument will be maintained while any older messages will be dropped. Thus a small queue size can lead to lower memory utilization, with the potential risk of message loss.

### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: By default, ros2 launch will always search through the install directory of sourced workspaces. Changes to package files are not automatically reflected in the install directory. Thus, if the launch is called, it is required to colcon build again so that the launch file installed in this directory.