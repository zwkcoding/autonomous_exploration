#ifndef _EXPLORATION_CONFIG_HPP_
#define _EXPLORATION_CONFIG_HPP_

namespace config {

    struct Weights {
        Weights() : explCells(1.0), angDist(1.0), robotGoalDist(1.0) {
        }

        Weights(double expl, double ang, double dist) : explCells(expl), angDist(ang), robotGoalDist(dist) {
        }

        double explCells; // Number of newly explored cells.
        double angDist; // Angle the robot has to be turned to face the goal.
        double robotGoalDist; // Distance between robot and goal.
    };

    struct Config {
        struct Weights weights;
         //  Goal within this distance to the robot are discarded.
        double min_goal_distance;
        double vehicle_length;
        double vehicle_width;
        double base2back;
    };

} // end namespace motion_planning_libraries

#endif
