# Trajectory Generation


## Setup OSQP:

Installation of OSQP lib:

```
git clone  --recursive https://github.com/osqp/osqp
sudo apt-get install libblas-dev liblapack-dev
cd osqp
git checkout 5bf713eb564629d233186e2fad8696195e4ea49f
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

## How to run:

`source /opt/ros/jazzy/setup.bash`

`cd /root/ros2_ws`

`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/install/bern_traj/lib/bern_traj`

`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug`

`source install/setup.bash`

`./build/bern_traj/trajectory_manager_num_opt`  or `ros2 run bern_traj trajectory_manager_num_opt`
`ros2 launch bern_traj multi_traj_launch.py`
For debugging: `ros2 run --prefix 'gdb -ex run --args' bern_traj trajectory_manager_num_opt`

Docker commands:
`docker stop ros2_container`
`docker start -ai ros2_container`

## Bernstein Trajectory: