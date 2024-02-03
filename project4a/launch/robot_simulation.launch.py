from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from project4a.disc_robot import load_disc_robot




def generate_launch_description():
    # robot_name_arg = DeclareLaunchArgument('robot_name') ## argument to enter at the terminal for the robot name
    

# Using Command substitution to concatenate path and argument
    
    # robot_name_file_path = Command(["echo /home/lab2004/project_ws/src/project4a/project4a/", LaunchConfiguration('robot_name')])
    bag_in_arg = DeclareLaunchArgument('bag_in') ## argument to enter at the terminal for the input bag file name
    bag_out_arg = DeclareLaunchArgument('bag_out') ## argument to enter at the terminal for the output bag file name
       # Using Command substitution to concatenate path and argument
    bag_file_path = Command(["echo /home/lab2004/Fall_2023/CSCE_752/project4a/project4a-bags/", LaunchConfiguration('bag_in')])
    bag_record_file_path = Command(["echo /home/lab2004/Fall_2023/CSCE_752/project4a/project4a-bags/recordings/", LaunchConfiguration('bag_out')])
    robot_file = 'ideal.robot'
    # robot_file_arg = DeclareLaunchArgument('robot_name')
    # robot_file = LaunchConfiguration('robot_name')

    ep = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_path],
        shell=True,  # Added this to ensure that the command is executed in the shell
        name='ros2_bag_play'  # Given a name to the process for identification
    )
    
    ep2 = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','/clicked_point','/cmd_vel','/goal_pose','/initialpose','/joint_states','/parameter_events','/robot_description','/rosout','/tf','/tf_static','/vl','/vr','-o', bag_record_file_path],
        shell=True,  # Added this to ensure that the command is executed in the shell
        name='ros2_bag_record'  # Given a name to the process for identification
    )

    robot_name_file_path= f'/home/lab2004/project_ws/src/project4a/project4a/{robot_file}'
    # robot_name_file_path = Command(["echo /home/lab2004/project_ws/src/project4a/project4a/", robot_file])

    robot = load_disc_robot(robot_name_file_path)
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot['urdf']}],
        )
    velocity_translator_node = Node(
            package='project4a',
            executable='velocity_translator',
            name='velocity_translator',
            output='screen',
            parameters=[{'robot_name':robot_name_file_path}]
        )
    
    differential_drive_simulator_node = Node(
            package='project4a',
            executable='differential_drive_simulator',
            name='differential_drive_simulator',
            output='screen',
            parameters=[{'robot_name':robot_name_file_path}],
        )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        )
    return LaunchDescription([
        bag_in_arg,
        bag_out_arg,
        robot_state_publisher_node,
        velocity_translator_node,
        differential_drive_simulator_node,
        rviz_node,
        ep,
        ep2
        

    ])