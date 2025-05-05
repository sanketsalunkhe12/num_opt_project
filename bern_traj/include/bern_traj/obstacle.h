#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <Eigen/Dense>

/*
    * @file obstacle.h
    * @brief Obstacle class definition
    * 
    * All obstacles will be represented as rectangular prisms in 3D space.
    * The obstacle is defined by its position (center), length, width, and height.
*/

struct Obstacle
{
    Eigen::Vector3d position{0.0, 0.0, 0.0};
    float length{0.0}, width{0.0}, height{0.0};
    
};

#endif // OBSTACLE_H