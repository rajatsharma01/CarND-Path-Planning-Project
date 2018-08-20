#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include <vector>
#include "car.h"
#include "jmt.h"

class Trajectory {
private: // Constants
    static const vector<double> SIGMA_S = {10.0, 4.0, 2.0};
    static const vector<double> SIGMA_D = {1.0, 1.0, 1.0};
    static const double SIGMA_T = 2.0;
    static const int N_SAMPLES = 10;

private: // Data members
    Car _car_start; // Starting state of trajectory
    Car _car_end; // End state of trajectory
    double _T;  // expected time to reach desired goal state
    JMT _jmt_s; // JMT for s
    JMT _jmt_d; // JMT for d

public: // C-tor
    Trajectory(Car car_start, Car car_end, double T);

public: // Methods

    // Return vector of trajectories including this original trajectory, with goals
    // perturbed in time, s and d
    vector<Trajectory> get_perturb_trajectories() const;
};

#endif  // _TRAJECTORY_H
