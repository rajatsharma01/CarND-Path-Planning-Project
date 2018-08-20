#include <cassert>
#include "planner.h"

vector<State>
Planner::successor_states() {
	vector<State> next_states;
	// Its always possible to keep lane
	next_states.push_back(STATE_KL);
	if (_state == STATE_KL) {
		next_states.push_back(STATE_PLCL);
		next_states.push_back(STATE_PLCR);
	} else if (_state == STATE_PLCL && _lane != LANE_LEFT) {
		next_states.push_back(STATE_PLCL);
		next_states.push_back(STATE_LCL);
	} else if (_state == STATE_PLCR && _lane != LANE_RIGHT) {
		next_states.push_back(STATE_PLCR);
		next_states.push_back(STATE_LCR);
	}
	return next_states;
}

Lane
Planner::d_to_lane(double d) {
    assert(d > 0 && d < 3 * LANE_WIDTH);
    if (d < LANE_WIDTH) {
        return LANE_LEFT;
    }
    if (d < 2*LANE_WIDTH) {
        return LANE_MIDDLE;
    }
    return LANE_RIGHT;
}

double
Planner::lane_to_d(Lane lane) {
    double d;
    switch (lane) {
        case LANE_LEFT:     d = 0.5 * LANE_WIDTH; break;
        case LANE_MIDDLE:   d = 1.5 * LANE_WIDTH; break;
        case LANE_RIGHT:    d = 2.5 * LANE_WIDTH; break;
    }
    return d;
}

bool
Planner::get_car_ahead(Lane lane, Car& out_car) {
    double min_s = _max_s;
    bool found = false;
    Car& car_ahead;
    for (Predictions::iterator it = _predictions.begin(); it != _predictions.end(); ++it) {
        car_ahead = it->second;
        if (d_to_lane(car_ahead.get_d()) == lane &&
            car_ahead.get_s() > _car_start.get_s() &&
            car_ahead.get_s() < min_s) {
            min_s = car_ahead.get_s();
            out_car = car_ahead;
            found = true;
        }
    }
    return found;
}

bool
Planner::get_car_behind(Lane lane, Car& out_car) {
    double max_s = -1;
    bool found = false;
    Car& car_behind;
    for (Predictions::iterator it = _predictions.begin(); it != _predictions.end(); ++it) {
        car_behind = it->second;
        if (d_to_lane(car_behind.get_d()) == lane &&
            car_behind.get_s() < _car_start.get_s() &&
            car_behind.get_s() > max_s) {
            max_s = car_behind.get_s();
            out_car = car_behind;
            found = true;
        }
    }
    return found;
}

Car
Planner::get_lane_kinematics(Lane lane) {
    double s;
    double v = std::min(Car::MAX_SPEED, _car_start.get_s_dot() + Car::MAX_ACCELERATION * _T);
    double a;
    double T2 = _T*_T;
    Car car_ahead;

    // Follow the car ahead
    if (get_car_ahead(lane, car_ahead)) {
        v = std::min(car_ahead.get_s_dot(), v);
    }
    a = (v - _car_start.get_s_dot())/_T;        // a = (v-u)/t
    s = _car_start.get_s() + v*_T + 0.5*a*T2;   // s = ut + 1/2at^2 

    return Car(s, v, a, lane_to_d(lane), 0, 0);
}

bool
Planner::constant_speed_goal(State next_state, Car& goal) {
    return Car(
}

bool
Planner::keep_lane_goal(State next_state, Car& goal) {
}

bool
Planner::prepare_lane_change_goal(State next_state, Car& goal) {
}

bool
Planner::lane_change_goal(State next_state, Car& goal) {
}

bool
Planner::generate_trajectory(State next_state, NextMove& out_move) {
    Car goal;
    bool status = false;
    switch (next_state) {
        case STATE_CS:
            status = constant_speed_trajectory(next_state, goal);
            break;
        case STATE_KL:
            status = keep_lane_trajectory(next_state, goal);
            break;
        case STATE_PLCL:
        case STATE_PLCR:
            status = prepare_lane_change_trajectory(next_state, goal);
            break;
        case STATE_LCL:
        case STATE_LCR:
            status = lane_change_trajectory(next_state, goal);
            break;
    }

    // If transition is possible, generate a minimum cost jerk minimum trajectory
    // for this possible transition
    if (status) {
        double min_cost = std::numeric_limits<double>::max();
        vector<Trajectory> trajectories =
            Trajectory(_car_start, goal, _T).get_perturb_trajectories();
        for (vector<Trajectory>::iterator it = trajectories; it != trajectories.end(); ++it) {
            double cost = CostFunction(*it, _predictions)();
            if (cost < min_cost) {
                min_cost = cost;
                out_move.trajectory = *it;
                out_move.cost = cost;
                out_move.state = next_state;
            }
        }
    }

    return status;
}

vector<FrenetPts>
Planner::plan(Car& car_start, double T, Predictions& predictions) {
    _car_start = car_start;
    _T = T;
    _predictions = predictions;
    NextMove move = find_best_move();
    for (auto next_state : successor_states()) {
        boolove move = generate_trajectory(next_state, move);



    }
    
}
