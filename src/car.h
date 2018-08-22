#ifndef _CAR_H
#define _CAR_H

#include <vector>
#include <map>

// Car class can represent a logical position of a car on same side of the road
// as ego car. This class can either tell facts about current state as pulled
// from sensor fusion data or can be used as a point on trajectory predicted
// for this car.
struct Car {
private: // Constants
    static const double MAX_SPEED = 22.35; // maximum allowed speed in m/s (= 50 MPH)
    static const double MAX_ACCELERATION = 10; // maximum acceleration of this car in m/s^2
    static const double MAX_JERK = 10; // maximum jerk in m/s^3
    static const double PREFERRED_GAP = 15; // preffered gap in m to maintain from front car 
    static const double OVERLAP_DISTANCE = 15; // two cars below overlap distance can collide

private: // Data members
    double _s;          // car's s position in frenet coordinates
    double _s_dot;      // first order derivative os s w.r.t. time (longitudanal acceleration)
    double _s_dot_dot;  // second order derivative os s w.r.t. time (longitudanal jerk)
    double _d;          // car's d position in frenet coordinates
    double _d_dot;      // first order derivative of d w.r.t. time (horizontal acceleration)
    double _d_dot_dot;  // second order derivative of d w.r.t. time (horizontal jerk)

public: // C-tor/D-tor
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

    // Return true if this car's s distance overlaps with other_car
    bool overlaps(Car& other_car) const {
        return fabs(this->s - other_car) < OVERLAP_DISTANCE;
    }

    // Accessors
    double get_s() const { return _s; }
    double get_d() const { return _d; }
    double get_s_dot() const { return _s_dot; }
    double get_d_dot() const { return _d_dot; }
    double get_s_dot_dot() const { return _s_dot_dot; }
    double get_d_dot_dot() const { return _d_dot_dot; }

    std::vector<double> get_s_vector() const {
        vector<double> svec = {_s, _s_dot, _s_dot_dot};
        return svec;
    }

    std::vector<double> get_d_vector() const {
        vector<double> dvec = {_d, _d_dot, _d_dot_dot};
        return dvec;
    }
};

// Predictions of all cars state on the road
typedef map<int, Car> Predictions;

#endif // _CAR_H
