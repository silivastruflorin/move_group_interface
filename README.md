In this tutorial we did:

- create bridges between ignition gazebo for joint states,clock,jointtrajectory controller and ros2
- launch the node: move_group and loaded the URDF,SRDF,Config files
Note:

	- move_group from moveit2 talks to ros2 ,in this example without ros2_control, through an action server(see package action_ros_server_for_move_it) created by me.
	- move_group creates an action client(for this client to be created we created a file called panda_ros_controller.yaml and here we specified a dummy controllername and the joints.Moveit takes this controllername and creates an action client wich has 4 topics:goal/result/feedback)
	- now that the action client was created by moveit we can start the action server wich has the name and type exactly as decribed in panda_ros_controller.yaml
panda_ros_controller.yaml


**Dependecies:**
	This package in order to work we need to download the `action_ros_server_for_moveit` from my repositories