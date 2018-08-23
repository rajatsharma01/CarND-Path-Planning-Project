#include <math.h>

#include "cost.h"
#include "constants.h"

double
CostFunction::logistic(double x) {
    return 2.0 / (1 + exp(-x)) - 1.0;
}

double
CostFunction::collision_cost() {
    for (Predictions::iterator it = _predictions.begin(); it != _predictions.end(); _it) {
        Car other = it->at(_trajectory._T);
        double s = _trajectory._jmt_s.get_value(_trajectory._T);
        double d = _trajectory._jmt_d.get_value(_trajectory._T);
        if (other.overlaps(s, d)) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::buffer_cost() {
    double closest = std::numeric_limit<double>max();
    for (Predictions::iterator it = _predictions.begin(); it != _predictions.end(); _it) {
        Car other = it->at(_trajectory._T);
        double s_diff = _trajectory._jmt_s.get_value(_trajectory._T) - other.get_s();
        double d_diff = _trajectory._jmt_d.get_value(_trajectory._T) - other.get_d();
        distance = sqrt(s_diff * s_diff + d_diff * d_diff);
        if (distance < closest) {
            closest = distance;
        }
    }
    return logistic(PREFERRED_GAP/closest);
}

double
CostFunction::max_accel_cost() {
    for (double t = 0; t < _T; t += TIME_STEP) {
        if (fabs(_trajectory._jmt_s.get_second_derivative(t)) > MAX_ACCELERATION) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::total_accel_cost() {
    double accel = 0;
    for (double t = 0; t < _T; t += TIME_STEP) {
        accel += fabs(_trajectory._jmt_s.get_second_derivative(t));
    }
    accel /= _T;
    return logistic(accel/TOTAL_ACCELERATION);
}

double
CostFunction::max_jerk_cost() {
    for (double t = 0; t < _T; t += TIME_STEP) {
        if (fabs(_trajectory._jmt_s.get_third_derivative(t)) > MAX_JERK) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::total_jerk_cost() {
    double jerk = 0;
    for (double t = 0; t < _T; t += TIME_STEP) {
        jerk += fabs(_trajectory._jmt_s.get_third_derivative(t));
    }
    jerk /= _T;
    return logistic(jerk/TOTAL_JERK);
}

double
CostFunction::speed_limit_cost() {
    for (double t = 0; t < _T; t += TIME_STEP) {
        if (fabs(_trajectory._jmt_s.get_first_derivative(t)) > MAX_SPEED) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::stay_on_road_cost() {
    for (double t = 0; t < _T; t += TIME_STEP) {
        double d = _trajectory._jmt_d.get_value(t);
        if (d < 0 || d > MAX_d) {
            return 1;
        }
    }
    return 0;
}

double
CostFunction::lane_center_cost() {
    double d = std::fmod(_trajectory._jmt_d.get_value(_T), LANE_WIDTH);
    return logistic(fabs(d-LANE_CENTER)/LANE_CENTER);
}

double
CostFunction::time_diff_cost() {
    return logistic(fabs(_T - trajectory._T)/trajectory.SIGMA_T);
}

double
CostFunction::s_diff_cost() {
    double cost = 0;
    vector<double> goal_svec = _goal.get_s_vector();
    vector<double> traj_svec = trajectory._jmt_s.get_vector();
    for (size_t i = 0; i < goal_svec.size(); i++) {
        cost += logistic(fabs(goal_svec[i] - traj_svec[i]) / trajectory.SIGMA_S[i]);
    }
    return cost;
}

double
CostFunction::d_diff_cost() {
    double cost = 0;
    vector<double> goal_dvec = _goal.get_d_vector();
    vector<double> traj_dvec = trajectory._jmt_d.get_vector();
    for (size_t i = 0; i < goal_svec.size(); i++) {
        cost += logistic(fabs(goal_dvec[i] - traj_dvec[i]) / trajectory.SIGMA_D[i]);
    }
    return cost;
}

double
CostFunction::operator()() {
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
