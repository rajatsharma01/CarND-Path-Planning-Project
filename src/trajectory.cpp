#include "trajectory.h"

const std::vector<double> Trajectory::SIGMA_S = {10.0, 4.0, 2.0};
const std::vector<double> Trajectory::SIGMA_D = {1.0, 1.0, 1.0};

Trajectory::Trajectory(const Car& car_start, const Car& car_end, double T)
    : _car_start(car_start), _car_end(car_end), _T(T),
    _jmt_s(car_start.get_s_vector(), car_end.get_s_vector(), T),
    _jmt_d(car_start.get_d_vector(), car_end.get_d_vector(), T)
{ }

std::vector<Trajectory>
Trajectory::get_perturb_trajectories() const {
    std::vector<Trajectory> perturb_trajs;
    perturb_trajs.push_back(*this);

    std::vector<double> svec = _car_end.get_s_vector();
    std::vector<double> dvec = _car_end.get_d_vector();

    std::default_random_engine generator;
    std::normal_distribution<double> T_distrib(_T, SIGMA_T);
    std::vector<std::function<double(std::default_random_engine&)>> S_distrib;
    std::vector<std::function<double(std::default_random_engine&)>> D_distrib;

    for (size_t i = 0; i < SIGMA_S.size(); i++) {
        S_distrib.push_back(std::normal_distribution<double>(svec[i], SIGMA_S[i]));
        D_distrib.push_back(std::normal_distribution<double>(dvec[i], SIGMA_D[i]));
    }

    for (int s = 0; s < N_SAMPLES; s++) {
        double perturb_T = T_distrib(generator);
        Car perturb_end = Car(S_distrib[0](generator),
                              S_distrib[1](generator),
                              S_distrib[2](generator),
                              D_distrib[0](generator),
                              D_distrib[1](generator),
                              D_distrib[2](generator));
        assert(perturb_end != _car_start);
        perturb_trajs.push_back(Trajectory(_car_start, perturb_end, perturb_T));
    }

    return perturb_trajs;
}

std::vector<FrenetPt>
Trajectory::get_frenet_points() const {
    std::vector<FrenetPt> fpts;
    for (double t = 0; t < _T; t += TIME_STEP) {
        double s = _jmt_s.get_value(t);
        double d = _jmt_d.get_value(t);
        if (s > _car_end.get_s()) {
            break;
        }
        fpts.push_back(FrenetPt(s, d));
    }
    return fpts;
}

std::ostream&
operator<<(std::ostream& os, const Trajectory& trajectory) {
    os << "Trajectory: " << std::endl;
    os << "Start " << trajectory._car_start << std::endl;
    os << "End " << trajectory._car_end << std::endl;
    os << "T: " << trajectory._T << std::endl;
    os << "s " << trajectory._jmt_s << std::endl;
    os << "d " << trajectory._jmt_d << std::endl;
    return os;
}
