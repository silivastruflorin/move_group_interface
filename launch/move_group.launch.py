import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # # planning_context
    # robot_description_config = xacro.process_file(
    #     os.path.join(
    #         get_package_share_directory("moveit_resources_panda_moveit_config"),
    #         "config",
    #         "panda.urdf.xacro",
    #     )
    # )
    # robot_description = {"robot_description": robot_description_config.toxml()}
    robot_description_config = load_file(
        "move_group_interface", "urdf/ur5_rg2.urdf.xml"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "move_group_interface", "config/ur5_rg2.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "move_group_interface", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "move_group_interface", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "move_group_interface", "config/panda_ros_controller.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

   
  
    # # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )


    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r ' + get_package_share_directory('move_group_interface') + '/model/ur_trajectory_pos_vel_cmd.sdf'
        }.items(),
    )

    # Bridge IGN -> ROS2
    bridge_Clock = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='parameter_bridge_block',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock','--ros-args'],
<<<<<<< Updated upstream
        parameters=[{'use_sim_time': use_sim_time}]
=======
>>>>>>> Stashed changes
         )



    # Bridge IGN -> ROS2
    bridge_JointState = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ign_bridge_joint_states',
        arguments=['/world/arm_robot_world/model/ur5_rg2/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',  #receive joint states from ignition
                   ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("/world/arm_robot_world/model/ur5_rg2/joint_state","/joint_states")] 
    )


    # Bridge ROS2 -> IGN
    bridge_arm_JointTrajectory = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ign_bridge_arm_joint_trajectory',
        arguments=['/model/ur5_rg2/arm_controller@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory'
                   ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("/model/ur5_rg2/arm_controller","/arm_joint_trajectory")]  
    )

    # Bridge ROS2 -> IGN
    bridge_gripper_JointTrajectory = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ign_bridge_gripper_joint_trajectory',
        arguments=['/model/ur5_rg2/gripper_controller@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory'
                   ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("/model/ur5_rg2/gripper_controller","/gripper_joint_trajectory")]  
    )

    # Bridge IGN -> ROS2
    bridge_arm_Joint_trajectory_progress= Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ign_bridge_arm_Joint_trajectory_progress',
        arguments=['/model/ur5_rg2/arm_controller_progress@std_msgs/msg/Float32[ignition.msgs.Float',  #receive joint states from ignition
                   ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("/model/ur5_rg2/arm_controller_progress","/arm_controller_progress")] 
    )

    # Bridge IGN -> ROS2
    bridge_gripper_Joint_trajectory_progress= Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ign_bridge_gripper_Joint_trajectory_progress',
        arguments=['/model/ur5_rg2/hand_controller_progress@std_msgs/msg/Float32[ignition.msgs.Float',  #receive joint states from ignition
                   ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("/model/ur5_rg2/hand_controller_progress","/hand_controller_progress")] 
    )

      # RViz
    rviz_config_file = (
        get_package_share_directory("move_group_interface") + "/launch/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world_link", "base_link"],
    )

     # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )

    ros_action_server_arm = Node(
        package="action_ros_server_for_moveit",
        executable="action_server_exe",
        name="ros_action_server_arm",
        output="screen",
        parameters=[{"action_name": "ur5_controller/follow_joint_trajectory"}], #set the name of the action server which will connect to moveit action client
        remappings=[("/joint_trajectory","/arm_joint_trajectory"),
                     ("/joint_trajectory_progress","/arm_controller_progress")
                    ],
    )

    ros_action_server_gripper = Node(
        package="action_ros_server_for_moveit",
        executable="action_server_exe",
        name="ros_action_server_gripper",
        output="screen",
        parameters=[{"action_name": "hand_controller/follow_joint_trajectory"}],
        remappings=[("/joint_trajectory","/gripper_joint_trajectory"),
                    ("/joint_trajectory_progress","/hand_controller_progress")
                   ], 
    )



    # Warehouse mongodb server
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
    )

    return LaunchDescription(
<<<<<<< Updated upstream
        [
            
=======
        [   
            ign_gazebo,
            robot_state_publisher,
>>>>>>> Stashed changes
            static_tf,
            robot_state_publisher,
            ign_gazebo,
            bridge_JointState,
            bridge_arm_JointTrajectory,
            bridge_gripper_JointTrajectory,
            bridge_arm_Joint_trajectory_progress,
            bridge_gripper_Joint_trajectory_progress,
            bridge_Clock,
            rviz_node,
            ros_action_server_arm,
            ros_action_server_gripper,
            run_move_group_node,
           
            #mongodb_server_node,
        ]
    )
