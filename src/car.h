#ifndef _CAR_H
#define _CAR_H

#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include "constants.h"

// Car class can represent a logical position of a car on same side of the road
// as ego car. This class can either tell facts about current state as pulled
// from sensor fusion data or can be used as a point on trajectory predicted
// for this car.
struct Car {
private: // Data members
    double _s;          // car's s position in frenet coordinates
    double _s_dot;      // first order derivative os s w.r.t. time (longitudanal acceleration)
    double _s_dot_dot;  // second order derivative os s w.r.t. time (longitudanal jerk)
    double _d;          // car's d position in frenet coordinates
    double _d_dot;      // first order derivative of d w.r.t. time (horizontal acceleration)
    double _d_dot_dot;  // second order derivative of d w.r.t. time (horizontal jerk)

public: // C-tor
    Car() { }

    // Derived state values
    Car(double s, double d, double s_dot, double d_dot,
        double s_dot_dot = 0.0, double d_dot_dot = 0.0)
        : _s(s), _d(d), _s_dot(s_dot), _d_dot(d_dot),
          _s_dot_dot(s_dot_dot), _d_dot_dot(d_dot_dot)
    { }

public: // methods
    // Return new state of this car at time t
    Car at(double t) const {
        // return new position assuming constant velocity
        double t2 = t*t;
        return Car(_s + _s_dot * t + _s_dot_dot * t2 * 0.5,
                   _s_dot + _s_dot_dot * t,
                   _s_dot_dot,
                   _d + _d_dot * t + _d_dot_dot * t2 * 0.5,
                   _d_dot + _d_dot_dot * t,
                   _d_dot_dot);
    }

    bool overlaps(double s, double d) const {
        return (fabs(s - _s) < S_OVERLAP) && (fabs(d - _d) < D_OVERLAP);
    }

    // Return true if this car's s distance overlaps with other_car
    bool overlaps(const Car& other_car) const {
        overlaps(other_car._s, other_car._d);
    }

    // Accessors
    double get_s() const { return _s; }
    double get_d() const { return _d; }
    double get_s_dot() const { return _s_dot; }
    double get_d_dot() const { return _d_dot; }
    double get_s_dot_dot() const { return _s_dot_dot; }
    double get_d_dot_dot() const { return _d_dot_dot; }

    std::vector<double> get_s_vector() const {
        std::vector<double> svec = {_s, _s_dot, _s_dot_dot};
        return svec;
    }

    std::vector<double> get_d_vector() const {
        std::vector<double> dvec = {_d, _d_dot, _d_dot_dot};
        return dvec;
    }

    void print() const {
        std::vector<std::vector<double>> sd_vec = { get_s_vector(), get_d_vector() };
        std::cout << "Car: " << std::endl;
        for (size_t i = 0; i < sd_vec.size(); i++) {
            std::cout << ((i == 0) ? "s" : "d") << " vector: [ ";
            for (size_t ii = 0; ii < sd_vec[i].size(); ii++) {
                if (ii > 0) {
                    std::cout << ", ";
                }
                std::cout << sd_vec[i][ii];
            }
            std::cout << " ]" << std::endl;
        }
    }
};

// Predictions of all cars state on the road
typedef std::map<int, Car> Predictions;

#endif // _CAR_H
