cmake_minimum_required(VERSION 3.10)
project(operator_splitting)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(operator_splitting
    src/main.cpp
    src/plot.cpp
)


# Setup for plotting trajectories
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

target_link_libraries(operator_splitting ${Python3_LIBRARIES})
include_directories(
    include
    /usr/include/python3.10  # Adjust to your Python version
    /usr/local/include/python3.10
    /usr/include/eigen3
    include/matplotlib-cpp
)
