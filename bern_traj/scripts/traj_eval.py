import yaml
import random
import time
import subprocess
import csv
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import os
import json
import shutil
import threading

CONFIG_PATH = '../config/multi_traj_params.yaml'
OTHER_CONFIG_PATH = '../../install/bern_traj/share/bern_traj/config/multi_traj_params.yaml' # NOTE: This is not an ideal way to do this, but i couldn't get other ways of updating the yaml file to work without rebuilding
LOG_PATH = '../data'
NUM_TRIALS = 5 # number of trajectories to generate
SEED = 42 # makes it reproducible

# TODO: get data for convergence and create graphs for convergence rate
# TODO: incorporate comparison between ours and standard osqp

# ------------------- helper functions -----------------------

def random_vector(bounds, dim=3):
    return [random.uniform(*bounds) for _ in range(dim)]

def create_waypoints(num_points=3, bounds=(0, 15)):
    return [coord for _ in range(num_points) for coord in random_vector(bounds)]

def create_obstacles(num_obs=3, bounds=(0, 15)):
    return [coord for _ in range(num_obs) for coord in random_vector(bounds)]

def split_to_xyz(lst):
    return [lst[i:i+3] for i in range(0, len(lst), 3)]

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def check_min_distance_between_agents(agents_wps, min_dist):
    '''Check that no waypoints overlap.'''
    for i in range(len(agents_wps)):
        for j in range(i + 1, len(agents_wps)):
            for wp1, wp2 in zip(agents_wps[i], agents_wps[j]):
                if distance(wp1, wp2) < min_dist:
                    return False
    return True

def check_waypoint_obstacle_clearance(waypoints, obstacles, min_clearance):
    '''Check that waypoints and obstacles don't overlap.'''
    for wp in waypoints:
        for obs in obstacles:
            if distance(wp, obs) < min_clearance:
                return False
    return True

def ensure_nontrivial_path(waypoints_xyz):
    '''Make the path with curves (not just straight lines).'''
    for i in range(1, len(waypoints_xyz) - 1):
        v1 = np.array(waypoints_xyz[i]) - np.array(waypoints_xyz[i - 1])
        v2 = np.array(waypoints_xyz[i + 1]) - np.array(waypoints_xyz[i])
        angle = np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0))
        if angle > np.deg2rad(15):
            return True
    return False

def archive_yaml(trial_num, num_agents, num_obstacles, num_waypoints, source_path, archive_dir='./all_configs'):
    '''Save a copy of the config file for each trial.'''
    os.makedirs(archive_dir, exist_ok=True)
    filename = f"multi_traj_params_trial={trial_num}_agents={num_agents}_obs={num_obstacles}_pts={num_waypoints}.yaml"
    dest_path = os.path.join(archive_dir, filename)
    shutil.copy(source_path, dest_path)
    print(f"Saved config for trial {trial_num} to {dest_path}")

def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)

# ------------------- Create parameters file -----------------------

def generate_parameters(agent_count=3, obstacle_count=3, waypoint_count=3):
    data = {}
    all_waypoints = []
    obstacle_coords = create_obstacles(obstacle_count)

    for i in range(agent_count):
        while True:
            waypoints = create_waypoints(waypoint_count)
            waypoints_xyz = split_to_xyz(waypoints)
            if (check_min_distance_between_agents(all_waypoints + [waypoints_xyz], 1.5) and
                check_waypoint_obstacle_clearance(waypoints_xyz, split_to_xyz(obstacle_coords), 1.0) and
                ensure_nontrivial_path(waypoints_xyz)):
                break

        all_waypoints.append(waypoints_xyz)

        params = {
            "robot_name": f"robot_{i + 1}",
            "waypoints": [round(x, 2) for wp in waypoints_xyz for x in wp],
            "obstacles": obstacle_coords,
            "obstacle_distance": 1.0,
            "magic_fabian_constant": 6.0,
            "time_factor": 2.0,
            "minVel": [round(random.uniform(-5.0, -2.0), 2) for _ in range(3)],
            "maxVel": [round(random.uniform(2.0, 5.0), 2) for _ in range(3)],
            "minAcc": [round(random.uniform(-3.0, -1.0), 2) for _ in range(3)],
            "maxAcc": [round(random.uniform(1.0, 3.0), 2) for _ in range(3)],
        }

        data[f"traj_manager_agent_{i + 1}"] = {"ros__parameters": params}

    return data

def write_yaml(data, path):
    class NoAliasDumper(yaml.SafeDumper):
        def ignore_aliases(self, data):
            return True

        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow=True, indentless=indentless)
        
    with open(path, 'w') as f:
        yaml.dump(data, f, Dumper=NoAliasDumper, default_flow_style=None, sort_keys=False)

# ------------------- ROS Listener --------------------

class TrajResultListener(Node):
    def __init__(self, expected_robots):
        super().__init__('traj_result_listener')
        self.expected = set(expected_robots)
        self.results_by_robot = {}
        self.timestamps = {}
        self.lock = threading.Lock()
        self.subscription = self.create_subscription(
            String,
            '/opt_status',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            parsed = json.loads(msg.data)
            robot_name = parsed.get("robot")
            if robot_name in self.expected:
                with self.lock:
                    if robot_name not in self.results_by_robot:
                        self.results_by_robot[robot_name] = parsed
                        self.timestamps[robot_name] = time.time()
        except json.JSONDecodeError:
            pass

def wait_for_ros_messages(expected_robots, timeout_sec=20.0):
    rclpy.init()
    node = TrajResultListener(expected_robots)

    def spin():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    time.sleep(2.0)  # Ensure listener starts
    proc = subprocess.Popen(['ros2', 'launch', 'bern_traj', 'multi_traj_launch.py'])
    global_start = time.time()

    while (time.time() - global_start) < timeout_sec:
        with node.lock:
            if set(node.results_by_robot.keys()) == set(expected_robots):
                break
        time.sleep(0.1)

    proc.terminate()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)

    results = []
    with node.lock:
        for robot in expected_robots:
            if robot in node.results_by_robot:
                result = node.results_by_robot[robot]
                duration = node.timestamps[robot] - global_start
                results.append({
                    'robot': robot,
                    'feasible': result.get('feasible', False),
                    'obj_val': result.get('obj_val', float('nan')),
                    'time_sec': round(duration, 2)
                })
            else:
                results.append({
                    'robot': robot,
                    'feasible': 'Timed out',
                    'obj_val': float('nan'),
                    'time_sec': float('nan')
                })

    return results

# ------------------- Main loop for running trials -----------------------

def run_single_trial(trial, num_agents, num_obstacles, num_waypoints, log_writer):
    print(f"Running trial {trial} (agents={num_agents}, obs={num_obstacles}, wps={num_waypoints})")

    set_seed(SEED + trial)
    param_data = generate_parameters(agent_count=num_agents, obstacle_count=num_obstacles, waypoint_count=num_waypoints)
    write_yaml(param_data, CONFIG_PATH)
    write_yaml(param_data, OTHER_CONFIG_PATH)
    archive_yaml(trial, num_agents, num_obstacles, num_waypoints, CONFIG_PATH)

    expected_robots=[f"robot_{i+1}" for i in range(num_agents)]
    results = wait_for_ros_messages(
        expected_robots=expected_robots,
        timeout_sec=20.0
    )

    # Write results to csv
    for result in results:
        log_writer.writerow({
            'trial': trial,
            'time_sec': result.get('time_sec'),
            'robot_name': result['robot'],
            'feasible': result['feasible'],
            'obj_val': result['obj_val'],
            'num_agents': num_agents,
            'num_obstacles': num_obstacles,
            'num_waypoints': num_waypoints
        })

def run_trial_series(varied_param, values):
    """Run trials by varying one parameter (e.g. num of robots)."""
    os.makedirs(LOG_PATH, exist_ok=True)
    for val in values:
        log_filename = os.path.join(LOG_PATH, f"{varied_param}_{val}.csv")
        with open(log_filename, 'w', newline='') as csvfile:
            fieldnames = ['trial', 'time_sec', 'robot_name', 'feasible', 'obj_val',
                          'num_agents', 'num_obstacles', 'num_waypoints']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            for trial in range(1, NUM_TRIALS + 1):
                kwargs = { # Default values if not being varied
                    'num_agents': 4,
                    'num_obstacles': 3,
                    'num_waypoints': 3
                }
                kwargs[varied_param] = val
                run_single_trial(trial=trial, **kwargs, log_writer=writer)

def run_all():
    agent_counts = [1, 2, 4, 6, 8, 10]
    obstacle_counts = [1, 3, 5, 7, 10]
    waypoint_counts = [3, 5, 7, 9, 10]

    for param, vals in [('num_agents', agent_counts),
                        ('num_obstacles', obstacle_counts),
                        ('num_waypoints', waypoint_counts)]:
        run_trial_series(param, vals)

if __name__ == '__main__':
    run_all()
