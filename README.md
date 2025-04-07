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

## Bernstein Trajectory: