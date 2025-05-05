import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    launch_description = LaunchDescription()
    
    num_agents = 3

    multi_node_param = os.path.join(
        get_package_share_directory('bern_traj'),
        'config',
        'multi_traj_params.yaml'
    )

    multi_node_exec_path = os.path.join(
        get_package_share_directory('bern_traj'),
        'lib', 'bern_traj', 'trajectory_manager_num_opt')
    
    # multi_node = Node(
    #         package='bern_traj',
    #         executable='trajectory_manager_num_opt',
    #         name='trajectory_manager_num_opt',
    #         output='screen',
    #         # parameters=[multi_node_param]
    #         )
    
    # launch_description.add_action(multi_node)

    for i in range(num_agents):
        agent_name = f"traj_manager_agent_{i+1}"
        robot_name = f"robot_{i + 1}"

        launch_description.add_action(
            Node(
                package='bern_traj',
                executable='trajectory_manager_num_opt',
                name=agent_name,
                output='screen',
                parameters=[multi_node_param,
                    {'robot_name': robot_name}]
            )
        )

        # launch_description.add_action(
        #     ExecuteProcess(
        #         cmd=[
        #             'tmux', 'new-session', '-d', '-s', f'agent_{i}',
        #             'ros2', 'run', 'bern_traj', 'trajectory_manager_num_opt',
        #             '--ros-args', '--param', f'agent_id:={i}'
        #         ],
        #         shell=True
        #         )
        #         )
    
    return launch_description