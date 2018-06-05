This is a repository that contains an example of a ROS 2 node that can have
a service and a subscription callback running in separate threads.  This
repository contains 3 packages:

1.  A `base_node` package which contains a node that publishes odometry messages.
1.  A `gtg_msgs` package which contains a custom GoToGoalPose service definition.
1.  A `nav_node` package which contains a node that subscribes to odometry
    messages, and which also has a GoToGoalPose service.

The idea is that the base node is running and publishing odometry messages.
The nav node is a subscriber to those messages.  At some point, an external
entity calls the GoToGoalPose service, and GoToGoalPose needs the data
from the odometry callbacks.  Thus, they need to be in separate threads.

To run the example, do the following:

1.  Build/install ros2 (https://github.com/ros2/ros2/wiki/Installation)
1.  Clone this repository.
1.  Build this package: `ament build --isolated --symlink-install`
1.  Open a terminal, source the base installation of ros2 and the workspace
    where this repository was built.
1.  Run: `ros2 run base_node base_node`.  This will start publishing odometry
    messages every 100ms.
1.  Open another terminal, source the base installation of ros2 and the workspace
    where this repository was built.
1.  Run: `ros2 run nav_node nav_node`.  This should start printing out that it
    got an odometry message, and which thread is handling the work.
1.  Open a third terminal, source the base installation of ros2 and the workspace
    where this repository was built.
1.  Run: `ros2 service call /go_to_goal_pose gtg_msgs/GoToGoalPose '{goal_x_m: 1.0, goal_y_m: 0.0, goal_theta_rad: 0.0, speed_m_per_s: 0.0}'`.  This will call the service in the nav node, which will print out "Starting goToGoalPose".  In the meantime, the odometry messages should continue to come in.
