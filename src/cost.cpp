#include <math.h>

#include "cost.h"

double
CostFunction::logistic(double x) {
    return 2.0 / (1 + exp(-x)) - 1.0;
}

double
CostFunction::collision_cost() {
}

double
CostFunction::max_accel_cost() {
}

double
CostFunction::max_jerk_cost() {
}

double
CostFunction::buffer_cost() {
}

double
CostFunction::match_traffic_speed_cost() {
}

double
CostFunction::time_diff_cost() {
}

double
CostFunction::s_diff_cost() {
}

double
CostFunction::d_diff_cost() {
}

double
CostFunction::speed_limit_cost() {
}

double
CostFunction::stay_on_road_cost() {
}

double
CostFunction::lane_center_cost() {
}

double
CostFunction::total_jerk_cost() {
}

double
CostFunction::total_accel_cost() {
}

double
CostFunction::operator()() {
	return  (COST_WT_COLLISION * __collision_cost() +
			 COST_WT_MAX_ACCELERATION * __max_accel_cost() +
			 COST_WT_MAX_JERK * __max_jerk_cost() +
			 COST_WT_BUFFER * __buffer_cost() +
			 COST_WT_MATCH_TRAFFIC_SPEED * __match_traffic_speed_cost() +
			 COST_WT_TIME_DIFF * __time_diff_cost() +
			 COST_WT_S_DIFF * __s_diff_cost() +
			 COST_WT_D_DIFF * __d_diff_cost() +
			 COST_WT_SPEED_LIMIT * __speed_limit_cost() +
			 COST_WT_STAY_ON_ROAD * __stay_on_road_cost() +
			 COST_WT_LANE_CENTER * __lane_center_cost() +
			 COST_WT_AVG_JERK * __total_jerk_cost() +
			 COST_WT_AVG_ACCELERATION * __total_accel_cost() +
			 COST_WT_EFFICIENCY * __efficiency_cost() +
			 COST_WT_TARGET_LANE * __target_lane_cost()) /
			(COST_WT_COLLISION +
			 COST_WT_MAX_ACCELERATION +
			 COST_WT_MAX_JERK +
			 COST_WT_BUFFER +
			 COST_WT_MATCH_TRAFFIC_SPEED +
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
