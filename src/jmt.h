#ifndef _JMT_H
#define _JMT_H

#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
   This class calculates the Jerk Minimizing Trajectory that connects the
   initial state to the final state in time T.
 */
class JMT {
public: // Ctor
    // start - initial state vector containing
    //         <value, first order derivative, second order derivative>
    // w.r.t. dimension of T e.g. <distance, velocity, acceleration>
    // end   - end state vector similar to start
    // T     - The duration, in seconds, over which this maneuver should occur.
    JMT(std::vector<double> start, std::vector <double> end, double T);

public: // methods

    // Return value at t
    //  s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    double get_value(double t) const;

    // Return first order derivative at t
    //  s'(t) = a1 + 2 * a2 * t + 3 * a3 * t^2 + 4 * a4 * t^3 + 5 * a5 * t^4
    double get_first_derivative(double t) const;

    // Return second order derivative at t
    //  s''(t) = 2 * a2 + 6 * a3 * t + 12 * a4 * t^2 + 20 * a5 * t^3
    double get_second_derivative(double t) const;

    // Return third order derivative at t
    //  s'''(t) = 6 * a3 + 24 * a4 * t + 60 * a5 * t^2
    double get_third_derivative(double t) const;

    // Returns vector containing above 4 values at t
    // { s(t), s'(t), s''(t), s'''(t) }
    vector<double> get_vector(double t) const;

private: // Data members
    VectorXd _coeffs;
};

#endif  // _JMT_H
