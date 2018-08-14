#ifndef _CAR_H
#define _CAR_H

// Car class can represent a logical position of a car on same side of the road
// as ego car. This class can either tell facts about current state as pulled
// from sensor fusion data or can be used as a point on trajectory predicted
// for this car.
class Car {
private: // Constants

private: // Data members

    // car's state usually pulled from sensor fusion data
    int _id; // Unique id of this vehicle
    double _x; // car's x position in map coordinates
    double _y; // car's y position in map coordinates
    double _x_dot; // car's x velocity in m/s
    double _y_dot; // car's y velocity in m/s
    double _s; // car's s position in frenet coordinates
    double _d; // car's d position in frenet coordinates

    // Derived/predicted state variables
    int _lane; // current lane id of car

public: // C-tor/D-tor
    // Derived state values
    Car(int id, double x, double x_dot, double y_dot, double s, double d)
        : _id(id), _x(x), _y(y), _x_dot(x_dot), _y_dot(y_dot), _s(s), _d(d)
    { } 
};

#endif // _CAR_H
