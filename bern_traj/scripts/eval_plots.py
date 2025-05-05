import matplotlib.pyplot as plt
import os
import csv
import numpy as np
import re

LOG_PATH = './data'
PLOTS_PATH = './plots'

def plot_from_logs(varied_param, values):
    """Read CSV logs and generate plots of time and objective value."""
    avg_times = []
    avg_objs = []

    for val in values:
        path = os.path.join(LOG_PATH, f"{varied_param}_{val}.csv")
        times = []
        objs = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row['feasible'] == 'True':
                    times.append(float(row['time_sec']))
                    try:
                        objs.append(float(row['obj_val']))
                    except ValueError:
                        continue
        avg_times.append(np.mean(times) if times else float('nan'))
        avg_objs.append(np.mean(objs) if objs else float('nan'))

    plt.figure(figsize=(12, 5))

    param_name = varied_param.replace("_", " ").title()
    plt.subplot(1, 2, 1)
    plt.plot(values, avg_times, marker='o')
    plt.title(f"Computation Time vs {param_name}")
    plt.xlabel(param_name)
    plt.ylabel("Time (sec)")

    plt.subplot(1, 2, 2)
    plt.plot(values, avg_objs, marker='o')
    plt.title(f"Objective Value vs {param_name}")
    plt.xlabel(param_name)
    plt.ylabel("Objective Value")

    plt.tight_layout()
    os.makedirs(PLOTS_PATH, exist_ok=True)
    plt.savefig(os.path.join(PLOTS_PATH, f"{varied_param}_results.png"))
    plt.close()

def get_param_and_values(log_path):
    """Determines which params were varied based on CSV logs."""
    pattern = re.compile(r'([a-zA-Z_]+)_(\d+)\.csv')
    param_values = {}

    for filename in os.listdir(log_path):
        if filename.endswith('.csv'):
            match = pattern.match(filename)
            if match:
                param = match.group(1)
                value = int(match.group(2))
                if param not in param_values:
                    param_values[param] = []
                param_values[param].append(value)

    # Sort values for each parameter
    for param in param_values:
        param_values[param] = sorted(param_values[param])

    return param_values

if __name__ == '__main__':
    params = get_param_and_values(LOG_PATH)

    for varied_param, values in params.items():
        print(f"Plotting for: {varied_param}, values: {values}")
        plot_from_logs(varied_param, values)
