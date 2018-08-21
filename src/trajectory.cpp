#include "trajectory.h"

Trajectory::Trajectory(Car car_start, Car car_end, double T)
    : _car_start(car_start), _car_end(car_end), _T(T),
    _jmt_s(car_start.get_s_vector(), car_end.get_s_vector(), T),
    _jmt_d(car_start.get_d_vector(), car_end.get_d_vector(), T)
{ }

vector<Trajectory>
Trajectory::get_perturb_trajectories() {
    vector<Trajectory> perturb_trajs;
    perturb_trajs.push_back(*this);

    vector<double> svec = car_end.get_s_vector();
    vector<double> dvec = car_end.get_d_vector();

    std::default_random_engine generator;
    std::normal_distribution<double> T_distrib(_T, SIGMA_T);
    std::vector<std::normal_distribution<double>> S_distrib(SIGMA_S.size());
    std::vector<std::normal_distribution<double>> D_distrib(SIGMA_D.size());

    for (size_t i = 0; i < S_distrib.size(); i++) {
        S_distrib.push_back(std::normal_distribution<double>(svec[i], SIGMA_S[i]));
        D_distrib.push_back(std::normal_distribution<double>(dvec[i], SIGMA_D[i]));
    }

    for (int s = 0; s < N_SAMPLES; s++) {
        double perturb_T = T_distrib(generator);
        Car perturb_end = Car(S_distrib[0](generator),
                              D_distrib[0](generator),
                              S_distrib[1](generator),
                              D_distrib[1](generator),
                              S_distrib[2](generator),
                              D_distrib[2](generator));
        perturb_trajs.push_back(Trajectory(_car_start, perturb_end, perturb_T));
    }

    return perturb_trajs;
}

vector<FrenetPt>
Trajectory::get_frenet_points() {
    vector<FrenetPt> fpts;
    for (double t = 0; t < _T; t += TIME_STEP) {
        fpts.push_back(FrenetPt(_jmt_s.get_value(t), _jmt_d.get_value(t)));
    }
    return fpts;
}
