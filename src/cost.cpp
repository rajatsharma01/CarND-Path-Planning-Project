#include <cmath>

#include "cost.h"
#include "constants.h"

double
CostFunction::logistic(double x) const {
    return 2.0 / (1 + exp(-x)) - 1.0;
}

double
CostFunction::collision_cost() const {
    for (Predictions::const_iterator it = _predictions->begin(); it != _predictions->end(); ++it) {
        const Car& temp = it->second;
        Car other = temp.at(_trajectory->_T);
        double s = _trajectory->_jmt_s.get_value(_trajectory->_T);
        double d = _trajectory->_jmt_d.get_value(_trajectory->_T);
        if (other.overlaps(s, d)) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::buffer_cost() const {
    double closest = std::numeric_limits<double>::max();
    for (Predictions::const_iterator it = _predictions->begin(); it != _predictions->end(); ++it) {
        const Car& temp = it->second;
        Car other = temp.at(_trajectory->_T);
        double s_diff = _trajectory->_jmt_s.get_value(_trajectory->_T) - other.get_s();
        double d_diff = _trajectory->_jmt_d.get_value(_trajectory->_T) - other.get_d();
        double distance = sqrt(s_diff * s_diff + d_diff * d_diff);
        if (distance < closest) {
            closest = distance;
        }
    }
    return logistic(PREFERRED_GAP/closest);
}

double
CostFunction::max_accel_cost() const {
    for (double t = 0; t < _T; t += _trajectory->TIME_STEP) {
        if (fabs(_trajectory->_jmt_s.get_second_derivative(t)) > MAX_ACCELERATION) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::total_accel_cost() const {
    double accel = 0;
    for (double t = 0; t < _T; t += _trajectory->TIME_STEP) {
        accel += fabs(_trajectory->_jmt_s.get_second_derivative(t));
    }
    accel /= _T;
    return logistic(accel/TOTAL_ACCELERATION);
}

double
CostFunction::max_jerk_cost() const {
    for (double t = 0; t < _T; t += _trajectory->TIME_STEP) {
        if (fabs(_trajectory->_jmt_s.get_third_derivative(t)) > MAX_JERK) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::total_jerk_cost() const {
    double jerk = 0;
    for (double t = 0; t < _T; t += _trajectory->TIME_STEP) {
        jerk += fabs(_trajectory->_jmt_s.get_third_derivative(t));
    }
    jerk /= _T;
    return logistic(jerk/TOTAL_JERK);
}

double
CostFunction::speed_limit_cost() const {
    for (double t = 0; t < _T; t += _trajectory->TIME_STEP) {
        if (fabs(_trajectory->_jmt_s.get_first_derivative(t)) > MAX_SPEED) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::stay_on_road_cost() const {
    for (double t = 0; t < _T; t += _trajectory->TIME_STEP) {
        double d = _trajectory->_jmt_d.get_value(t);
        if (d < 0 || d > MAX_d) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::lane_center_cost() const {
    double t = _trajectory->_T;
    double d = std::fmod(_trajectory->_jmt_d.get_value(t), LANE_WIDTH);
    return logistic(fabs(d-LANE_CENTER)/LANE_CENTER);
}

double
CostFunction::target_lane_cost() const {
    double t = _trajectory->_T;
    double reached_lane = floor(_trajectory->_jmt_d.get_value(t) / LANE_WIDTH);
    double target_lane = floor(_goal.get_d() / LANE_WIDTH);
    return (target_lane != reached_lane) ? 1 : 0;
}

double
CostFunction::time_diff_cost() const {
    return logistic(fabs(_T - _trajectory->_T)/_trajectory->SIGMA_T);
}

double
CostFunction::s_diff_cost() const {
    double cost = 0;
    std::vector<double> goal_svec = _goal.get_s_vector();
    std::vector<double> traj_svec = _trajectory->_jmt_s.get_vector(_T);
    for (size_t i = 0; i < goal_svec.size(); i++) {
        cost += logistic(fabs(goal_svec[i] - traj_svec[i]) / _trajectory->SIGMA_S[i]);
    }
    return cost;
}

double
CostFunction::d_diff_cost() const {
    double cost = 0;
    std::vector<double> goal_dvec = _goal.get_d_vector();
    std::vector<double> traj_dvec = _trajectory->_jmt_d.get_vector(_T);
    for (size_t i = 0; i < goal_dvec.size(); i++) {
        cost += logistic(fabs(goal_dvec[i] - traj_dvec[i]) / _trajectory->SIGMA_D[i]);
    }
    return cost;
}

double
CostFunction::efficiency_cost() const {
    double t = _trajectory->_T;
    double avg_v = _trajectory->_jmt_s.get_value(t) / t;
    double target_v = _trajectory->_car_start.at(t).get_s() / t;
    return logistic(2*(target_v - avg_v)/avg_v);
}

double
CostFunction::operator()() const {
	return  (COST_WT_COLLISION * collision_cost() +
			 COST_WT_MAX_ACCELERATION * max_accel_cost() +
			 COST_WT_MAX_JERK * max_jerk_cost() +
			 COST_WT_BUFFER * buffer_cost() +
			 COST_WT_TIME_DIFF * time_diff_cost() +
			 COST_WT_S_DIFF * s_diff_cost() +
			 COST_WT_D_DIFF * d_diff_cost() +
			 COST_WT_SPEED_LIMIT * speed_limit_cost() +
			 COST_WT_STAY_ON_ROAD * stay_on_road_cost() +
			 COST_WT_LANE_CENTER * lane_center_cost() +
			 COST_WT_AVG_JERK * total_jerk_cost() +
			 COST_WT_AVG_ACCELERATION * total_accel_cost() +
			 COST_WT_EFFICIENCY * efficiency_cost() +
             COST_WT_TARGET_LANE * target_lane_cost()) /
			(COST_WT_COLLISION +
			 COST_WT_MAX_ACCELERATION +
			 COST_WT_MAX_JERK +
			 COST_WT_BUFFER +
			 COST_WT_TIME_DIFF +
			 COST_WT_S_DIFF +
			 COST_WT_D_DIFF +
			 COST_WT_SPEED_LIMIT +
			 COST_WT_STAY_ON_ROAD +
			 COST_WT_LANE_CENTER +
			 COST_WT_AVG_JERK +
			 COST_WT_AVG_ACCELERATION +
			 COST_WT_EFFICIENCY +
             COST_WT_TARGET_LANE);
}
