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

CONFIG_PATH = '../config/multi_traj_params.yaml'
LOG_PATH = '../data/eval_results.csv'
NUM_TRIALS = 2 # number of trajectories to generate

# TODO: add min/max vel and accel as things read from params.yaml
# TODO: add code for more / fewer robots. change the expected_robots list to handle this
# TODO: create graphs for time, feasibility, and obj value as function of # robots, # obstacles, # waypoints
# TODO: create graphs for convergence rate
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

# ------------------- Create parameters file -----------------------

def generate_parameters(agent_count=3):
    data = {}
    all_waypoints = []
    obstacle_coords = create_obstacles()

    for i in range(agent_count):
        while True:
            waypoints = create_waypoints()
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
            "minVel": round(random.uniform(0.1, 0.5), 2),
            "maxVel": round(random.uniform(0.6, 1.5), 2),
            "minAcc": round(random.uniform(0.1, 0.5), 2),
            "maxAcc": round(random.uniform(0.6, 1.5), 2),
        }

        data[f"traj_manager_agent_{i + 1}"] = {"ros__parameters": params}

    return data

def write_yaml(data, path=CONFIG_PATH):
    with open(path, 'w') as f:
        yaml.dump(data, f, sort_keys=False)

# ------------------- ROS Listener --------------------

class TrajResultListener(Node):
    def __init__(self):
        super().__init__('traj_result_listener')
        self.subscription = self.create_subscription(
            String,
            '/opt_status',
            self.listener_callback,
            10)
        self.results = []  # List to store multiple robot messages

    def listener_callback(self, msg):
        self.get_logger().info(f"Received status: {msg.data}")
        try:
            parsed = json.loads(msg.data)
            if isinstance(parsed, dict) and "robot" in parsed:
                self.results.append(parsed)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Failed to parse JSON: {e}")

def wait_for_ros_messages(expected_robots=['robot_1', 'robot_2', 'robot_3'], timeout_sec=20.0):
    rclpy.init()
    node = TrajResultListener()
    start = time.time()

    received_robot_ids = set()
    while (time.time() - start) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        if expected_robots:
            for r in node.results:
                received_robot_ids.add(r["robot"])
            if set(expected_robots).issubset(received_robot_ids):
                break  # All expected robots reported

    results = node.results
    node.destroy_node()
    rclpy.shutdown()
    return results

# ------------------- Main loop for running trials -----------------------

def run_trials(num_trials=NUM_TRIALS):
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
    with open(LOG_PATH, 'w', newline='') as csvfile:
        fieldnames = ['trial', 'time_sec', 'robot_name', 'feasible', 'obj_val']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for trial in range(1, num_trials + 1):
            print(f"Running trial {trial}...")
            param_data = generate_parameters()
            write_yaml(param_data)

            start_time = time.time()
            proc = subprocess.Popen(['ros2', 'launch', 'bern_traj', 'multi_traj_launch.py'])
            results = wait_for_ros_messages(timeout_sec=20.0)
            proc.terminate()
            duration = time.time() - start_time

            if results:
                for result in results:
                    writer.writerow({
                        'trial': trial,
                        'time_sec': round(duration, 2),
                        'robot_name': result.get('robot', None),
                        'feasible': result.get('feasible', False),
                        'obj_val': result.get('obj_val', float('nan'))
                    })
            else:
                writer.writerow({
                    'trial': trial,
                    'time_sec': round(duration, 2),
                    'robot_name': result.get('robot', None),
                    'feasible': False,
                    'obj_val': float('nan')
                })

if __name__ == '__main__':
    run_trials()
