#include <cassert>
#include "jmt.h"

JMT::JMT(std::vector<double> start, std::vector<double> end, double T) : _coeffs(6) {
    double T_2 = T * T;
    double T_3 = T_2 * T;
    double T_4 = T_3 * T;
    double T_5 = T_4 * T;

    MatrixXd time_mat(3, 3);
    time_mat <<  T_3,    T_4,    T_5,
                 3*T_2,  4*T_3,  5*T_4,
                 6*T,    12*T_2, 20*T_3;

    VectorXd svec(3);
    svec << end[0] - (start[0] + start[1]*T + start[2]*T_2/2),
            end[1] - (start[1] + start[2]*T),
            end[2] - start[2];

    VectorXd alpha = time_mat.inverse() * svec;

    _coeffs << start[0], start[1], start[2]/2, alpha;
}

double
JMT::get_value(double t) const {
    double t2 = t*t;
    double t3 = t2*t;
    double t4 = t3*t;
    double t5 = t4*t;

    VectorXd tv(6);
    tv << 1, t, t2, t3, t4, t5;

    return tv.dot(_coeffs);
}

double
JMT::get_first_derivative(double t) const {
    double t2 = t*t;
    double t3 = t2*t;
    double t4 = t3*t;

    VectorXd tv(5);
    tv << 1, 2 * t, 3 * t2, 4 * t3, 5 * t4;

    return tv.dot(_coeffs.tail(5));
}

double
JMT::get_second_derivative(double t) const {
    double t2 = t*t;
    double t3 = t2*t;

    VectorXd tv(4);
    tv << 2, 6 * t, 12 * t2, 20 * t3;

    return tv.dot(_coeffs.tail(4));
}

double
JMT::get_third_derivative(double t) const {
    double t2 = t*t;

    VectorXd tv(3);
    tv << 6, 24 * t, 60 * t2;

    return tv.dot(_coeffs.tail(3));
}

std::vector<double>
JMT::get_vector(double t) const {
    return { get_value(t),
             get_first_derivative(t),
             get_second_derivative(t),
             get_third_derivative(t) };
}

std::ostream& operator<<(std::ostream& os, const JMT& jmt) {
    os << "JMT: [ " << jmt._coeffs.transpose() << " ]";
    return os;
}
