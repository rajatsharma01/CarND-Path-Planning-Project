#ifndef _COST_H
#define _COST_H

#include "trajectory.h"

class CostFunction {
private: // Weights for different cost functions

    // Feasibility
    static const int COST_WT_COLLISION              = 100;
    static const int COST_WT_MAX_ACCELERATION       = 100;
    static const int COST_WT_MAX_JERK               = 100;

    // Safety
    static const int COST_WT_BUFFER                 = 40;
    static const int COST_WT_TIME_DIFF              = 40;
    static const int COST_WT_S_DIFF                 = 40;
    static const int COST_WT_D_DIFF                 = 40;

    // Legality
    static const int COST_WT_SPEED_LIMIT            = 10;
    static const int COST_WT_STAY_ON_ROAD           = 10;

    // Comfort
    static const int COST_WT_LANE_CENTER            = 5;
    static const int COST_WT_AVG_JERK               = 5;
    static const int COST_WT_AVG_ACCELERATION       = 5;

    // Efficiency
    static const int COST_WT_EFFICIENCY             = 1;
    static const int COST_WT_TARGET_LANE            = 1;

private: // Data members
    Car& _goal;                 // Target goal state for the trajectory
    double _T;                  // Expected time to reach goal state
    Trajectory& _trajectory;    // Trajectory whose cost needs to be determined
    Predictions& _predictions;  // other cars prediction to help determine cost

public: // C-tor
    CostFunction(Car& goal, double T, Trajectory& trajectory, Predictions& predictions)
        : _goal(goal), _T(T), _trajectory(trajectory), _predictions(predictions)
    { }

private: // helper functions
    // Returns values in range [-1, 1] for any value of x, used by below cost functions
    double logistic(double x) const;

private: // Various cost functions

    // Binary cost function which penalizes collisions.
    double collision_cost() const;

    // Binary cost function to penalize exceeding max acceleration
    double max_accel_cost() const;

    // Binary cost function to penalize exceeding max jerk
    double max_jerk_cost() const;

    // Penalizes getting close to other vehicles.
    double buffer_cost() const;

    // Penalizes trajectories that span a duration which is longer or shorter
    // than the duration requested.
    double time_diff_cost() const;

    // Penalizes trajectories whose s coordinate (and derivatives) differ
    // from the goal.
    double s_diff_cost() const;

    // Penalizes trajectories whose d coordinate (and derivatives) differ
    // from the goal.
    double d_diff_cost() const;

    // Penalizes exceeding speed limit
    double speed_limit_cost() const;

    // Penalizes going off the road
    double stay_on_road_cost() const;

    // Penalizes trajectories with goals which are far away from lane center
    double lane_center_cost() const;

    // Penalizes average jerk per second is higher than expected
    double total_jerk_cost() const;

    // Penalizes average acceleration per second is higher than expected
    double total_accel_cost() const;

    // Rewards high average speeds.
    double efficiency_cost() const;

    // Rewards trajectory ending in target lane
    double target_lane_cost() const;

public: // methods

    // Return total weighted cost of trajectory
    double operator()() const;
};

#endif  // _COST_H
