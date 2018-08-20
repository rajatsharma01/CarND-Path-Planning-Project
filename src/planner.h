#ifndef _PLANNER_H
#define _PLANNER_H

#include <vector>
#include <map>

#include "cost.h"

class Planner {
private: // types
    enum State {
        STATE_CS,   // Constant Speed - initial default state
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
    }

    // A point in Frenet coordinates
    struct FrenetPt {
        double s;   // s distance
        double d;   // d distance
    }

private: // Constants
    static const double LANE_WIDTH = 4.0;
    static const double TIME_STEP = 0.02;

private: // Data
    State _state; // current state
    Lane _lane; // current driving lane
    double _max_s; // Max s value for whole track
    double _T; // Time horizon for planning
    Car& car_start; // Starting state of the ego car
    Predictions& _predictions; // predictions for other cars on road

public: // C-tor
    Planner(double max_s)
        : _state(STATE_CS), _lane(LANE_MIDDLE), _max_s(max_s)
    { }

private: // helpers
    // Return list of possible next states
    vector<State> successor_states() const;

    // Returns lane from d value
    Lane d_to_lane(double d);

    // Returns d value of mid point from lane
    double lane_to_d(Lane lane);

    // Find next car ahead of us in desired lane
    // Returns true if a car is found and car is updated
    bool get_car_ahead(Lane lane, Car& car);

    // find next car behind us in desired lane
    // Returns true if a car is found and car is updated
    bool get_car_behind(Lane lane, Car& car);

    // Get kinematics of desired lane based on car in front
    Car get_lane_kinematics(Lane lane);

    // Trajectory generation
    NextMove generate_trajectory(State next_state);
    NextMove constant_speed_trajectory(State next_state);
    NextMove keep_lane_trajectory(State next_state);
    NextMove prepare_lane_change_trajectory(State next_state);

public: // API

    // Plan next move for ego vehicle by generating a low cost jerk minimizing trajectory
    // 
    // Params:
    //  car_start   - starting state of ego car
    //  T           - Time horizon for the move
    //  predictions - state predictions for other cars on the road
    //
    // returns path vector in frenet coordinates
    vector<Frenet> plan(Car& car_start, double T, Predictions& predictions);
};

#endif  // _PLANNER_H
