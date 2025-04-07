#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include "eigen3/Eigen/Dense"

struct Waypoint
{
    Eigen::Vector3f position{0.0, 0.0, 0.0};
    Eigen::Vector3f velocity{0.0, 0.0, 0.0};
    Eigen::Vector3f acceleration{0.0, 0.0, 0.0};
    Eigen::Vector3f jerk{0.0, 0.0, 0.0};
    Eigen::Quaternion <float> orientation{1.0, 0.0, 0.0, 0.0};
    float duration{0.0};
    bool relative{false};

    double getConstraint(int &order, int &dim) const
    {
        switch(order)
        {
            case 0:
                return position(dim);
            case 1:
                return velocity(dim);
            case 2:
                return acceleration(dim);
            case 3:
                return jerk(dim);
            default:
                return 0.0;
        }
    }
};

#endif // WAYPOINT_HPP