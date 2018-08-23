#ifndef _CONSTANTS_H
#define _CONSTANTS_H

static const double LANE_WIDTH = 4.0;       // width of the lane
static const double LANE_CENTER = LANE_WIDTH/2; // center of lane from from lane boundary
static const int TOTAL_LANES = 3;           // number of lanes on highway
static const double MAX_d = LANE_WIDTH * TOTAL_LANES; // Max value for d

static const double MAX_SPEED = 22.35;      // maximum allowed speed in m/s (= 50 MPH)
static const double MAX_ACCELERATION = 10;  // maximum acceleration of this car in m/s^2
static const double TOTAL_ACCELERATION = 2; // total acceleration per second in m/s
static const double MAX_JERK = 10;          // maximum jerk in m/s^3
static const double TOTAL_JERK = 1;         // total jerk per second in m/s^2

static const double PREFERRED_GAP = 15;     // preffered gap in m to maintain from front car
static const double S_OVERLAP = 10;         // Overlap s distance between two cars for detecting collision 
static const double D_OVERLAP = 4;          // Overlap d distance between two cars for detecting collision 

#endif  // _CONSTANTS_H
