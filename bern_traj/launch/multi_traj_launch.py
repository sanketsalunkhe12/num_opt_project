import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    launch_description = LaunchDescription()
    
    multi_node_param = os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'multi_traj_params.yaml'
    ))

    # Get the number of agents from the yaml file
    yaml_path = os.path.abspath(multi_node_param)
    with open(yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)
    agent_keys = [k for k in yaml_data.keys() if k.startswith('traj_manager_agent_')]
    num_agents = len(agent_keys)

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