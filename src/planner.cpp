#include <cassert>
#include "constants.h"
#include "planner.h"

std::vector<Planner::State>
Planner::successor_states() const {
    std::vector<Planner::State> next_states;
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

Planner::Lane
Planner::get_adjacent_lane(Planner::State next_state) const {
    Planner::Lane lane;
    switch (next_state) {
        case STATE_KL:
            lane = _lane;
            break;
        case STATE_PLCL:
        case STATE_LCL:
            lane = (_lane == LANE_RIGHT) ? LANE_MIDDLE : LANE_LEFT;
            break;
        case STATE_PLCR:
        case STATE_LCR:
            lane = (_lane == LANE_LEFT) ? LANE_MIDDLE : LANE_RIGHT;
            break;
    }
    return lane;
}

Planner::Lane
Planner::d_to_lane(double d) const {
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
Planner::lane_to_d(Lane lane) const {
    double d;
    switch (lane) {
        case LANE_LEFT:     d = 0.5 * LANE_WIDTH; break;
        case LANE_MIDDLE:   d = 1.5 * LANE_WIDTH; break;
        case LANE_RIGHT:    d = 2.5 * LANE_WIDTH; break;
    }
    return d;
}

bool
Planner::get_car_ahead(Lane lane, Car& out_car) const {
    double min_s = _max_s;
    bool found = false;
    for (Predictions::iterator it = _predictions->begin(); it != _predictions->end(); ++it) {
        Car& car_ahead = it->second;
        if (d_to_lane(car_ahead.get_d()) == lane &&
            car_ahead.get_s() > _car_start->get_s() &&
            car_ahead.get_s() < min_s) {
            min_s = car_ahead.get_s();
            out_car = car_ahead;
            found = true;
        }
    }
    return found;
}

bool
Planner::get_car_behind(Lane lane, Car& out_car) const {
    double max_s = -1;
    bool found = false;
    for (Predictions::iterator it = _predictions->begin(); it != _predictions->end(); ++it) {
        Car& car_behind = it->second;
        if (d_to_lane(car_behind.get_d()) == lane &&
            car_behind.get_s() < _car_start->get_s() &&
            car_behind.get_s() > max_s) {
            max_s = car_behind.get_s();
            out_car = car_behind;
            found = true;
        }
    }
    return found;
}

Car
Planner::get_lane_kinematics(Lane lane) const {
    double s;
    double v = std::min(MAX_SPEED, _car_start->get_s_dot() + Car::MAX_ACCELERATION * _T);
    double a;
    double T2 = _T*_T;
    Car car_ahead;

    // Follow the car ahead
    if (get_car_ahead(lane, car_ahead)) {
        v = std::min(car_ahead.get_s_dot(), v);
    }
    a = (v - _car_start->get_s_dot())/_T;        // a = (v-u)/t
    s = _car_start->get_s() + v*_T + 0.5*a*T2;   // s = ut + 1/2at^2 

    return Car(s, v, a, lane_to_d(lane), 0, 0);
}

bool
Planner::keep_lane_goal(Planner::State next_state, Car& goal) const {
    goal = get_lane_kinematics(_lane);
    return true;
}

bool
Planner::prepare_lane_change_goal(Planner::State next_state, Car& goal) const {
    goal = get_lane_kinematics(_lane);

    // We can not slow down if there is car behind us
    Car car_behind;
    if (get_car_behind(_lane, car_behind) == false) {
        Lane next_lane = get_adjacent_lane(next_state);
        Car next_goal = get_lane_kinematics(next_lane);
        if (next_goal.get_s_dot() < goal.get_s_dot()) {
            goal = next_goal; 
        }
    }

    return true;
}

bool
Planner::lane_change_goal(Planner::State next_state, Car& goal) const {
    Lane next_lane = get_adjacent_lane(next_state);
    for (Predictions::iterator it = _predictions->begin(); it != _predictions->end(); ++it) {
        Car other_car = it->second;
        if (d_to_lane(other_car.get_d()) == _lane && other_car.overlaps(*_car_start)) {
            // Can not perform a lane change if there is a car in adjacent lane
            return false;
        }
    }
    goal = get_lane_kinematics(next_lane);
    return true;
}

bool
Planner::generate_trajectory(Planner::State next_state, NextMove& out_move) const {
    Car goal;
    bool status = false;
    switch (next_state) {
        case STATE_KL:
            status = keep_lane_goal(next_state, goal);
            break;
        case STATE_PLCL:
        case STATE_PLCR:
            status = prepare_lane_change_goal(next_state, goal);
            break;
        case STATE_LCL:
        case STATE_LCR:
            status = lane_change_goal(next_state, goal);
            break;
    }

    // If no transition is possible, return error
    if (!status) return status;

    // Generate a minimum cost jerk minimum trajectory for this possible transition
    double min_cost = std::numeric_limits<double>::max();
    std::vector<Trajectory> trajectories =
        Trajectory(*_car_start, goal, _T).get_perturb_trajectories();
    for (std::vector<Trajectory>::iterator it = trajectories; it != trajectories.end(); ++it) {
        double cost = CostFunction(goal, _T, *it, *_predictions)();
        if (cost < min_cost) {
            min_cost = cost;
            out_move.trajectory = *it;
            out_move.cost = cost;
            out_move.state = next_state;
        }
    }

    return status;
}

void
Planner::print_trajectory(NextMove& move, std::vector<FrenetPt>& fpts) const {
    std::cout << "State: " << StateName[move.state] << ", Cost: " << move.cost << std::endl;
    std::cout << "Trajectory: [ ";
    for (int i = 0; i < fpts.size(); i++) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << "(" << fpts[i].s << "," << fpts[i].d << ")";
    }
    std::cout << " ]" << std::endl;
}

std::vector<FrenetPt>
Planner::plan(Car* car_start, double T, Predictions* predictions) {
    _car_start = car_start;
    _T = T;
    _predictions = predictions;
    _lane = d_to_lane(_car_start->get_d());

    double min_cost = std::numeric_limits<double>::max();
    NextMove move;
    bool status = false;
    for (auto next_state : successor_states()) {
        NextMove state_move;
        status = generate_trajectory(next_state, temp_move);
        if (status && state_move.cost < min_cost) {
            min_cost = state_move.cost;
            move = state_move;
        }
    }
    assert(status);
    std::vector<FrenetPt> fpts = move.trajectory.get_frenet_points();
    print_trajectory(move, fpts);

    _state = move.state;
     
    return fpts;
}
