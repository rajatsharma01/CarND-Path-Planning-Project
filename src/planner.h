#ifndef _PLANNER_H
#define _PLANNER_H

#include <vector>
#include <map>

#include "cost.h"

class Planner {
private: // types
    enum State {
        STATE_KL,   // Keep Lane
        STATE_PLCL, // Prepare to Lane change left
        STATE_PLCR, // Prepare to change lane right
        STATE_LCL,  // Lane change left
        STATE_LCR,  // Lane change right
    };

    enum Lane {
        LANE_LEFT,
        LANE_MIDDLE,
        LANE_RIGHT,
    };

    // Next move this planner can come up with
    struct NextMove {
        Trajectory trajectory;  // Trajectory associated with this move
        double cost;            // Cost of the trajectory
        State state;            // State transition of the move

        NextMove() { }
    };

private: // Constants
    static const std::vector<std::string> StateName;

private: // Data
    State _state; // current state
    Lane _lane; // current driving lane
    double _max_s; // Max s value for whole track
    double _T; // Time horizon for planning
    const Car* _car_start; // Starting state of the ego car
    const Predictions* _predictions; // predictions for other cars on road

public: // C-tor
    Planner(double max_s, double T)
        : _state(STATE_KL), _lane(LANE_MIDDLE), _max_s(max_s), _T(T), _car_start(0), _predictions(0)
    { }

private: // helpers
    // Return list of possible next states
    std::vector<State> successor_states() const;

    // Return adjacent lane based on next_state
    Lane get_adjacent_lane(Planner::State next_state) const;

    // Returns lane from d value
    Lane d_to_lane(double d) const;

    // Returns d value of mid point from lane
    double lane_to_d(Lane lane) const;

    // Find next car ahead of us in desired lane
    // Returns true if a car is found and car is updated
    bool get_car_ahead(Lane lane, Car& car) const;

    // find next car behind us in desired lane
    // Returns true if a car is found and car is updated
    bool get_car_behind(Lane lane, Car& car) const;

    // Get kinematics of desired lane based on car in front
    Car get_lane_kinematics(Lane lane) const;

    // Below functions return a goal configuration for ego car in desired lane
    // with state transition to next_state. Returns true if a goal state is found
    bool keep_lane_goal(const State next_state, Car& goal) const;
    bool prepare_lane_change_goal(const State next_state, Car& goal) const;
    bool lane_change_goal(const State next_state, Car& goal) const;

    // Generate a low cost jerk minimizing trajectory for next_state
    bool generate_trajectory(const State next_state, NextMove& out_move) const;

    // Print debug info about trajectory selected by planner
    void print_trajectory(const NextMove& move, const std::vector<FrenetPt>& fpts) const;

public: // API

    // Plan next move for ego vehicle by generating a low cost jerk minimizing trajectory
    // 
    // Params:
    //  car_start   - starting state of ego car
    //  predictions - state predictions for other cars on the road
    //
    // returns path vector in frenet coordinates
    std::vector<FrenetPt> plan(const Car* car_start, const Predictions* predictions);
};

#endif  // _PLANNER_H
