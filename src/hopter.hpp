#ifndef __hopter_hpp__
#define __hopter_hpp__

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 80, 1> Vectord;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 18, 1> Vector18d;
typedef Matrix<double, 36, 1> Vector36d;
typedef Matrix<double, 3, 6> Matrix3x6d;
typedef Matrix<double, 6, 6> Matrix6d;

typedef struct Parameters
{
    double time_step     = 0.01;
    double time_tau      = 0.01;
    double k_omega       = 50.0;
    double k_quat        = 10.0;
    double alpha_quat    = 10.0;
    Matrix3d I           = MatrixXd::Identity(3, 3);
    Matrix6d Gamma_omega = MatrixXd::Zero(6, 6);
    Matrix6d Gamma_theta = MatrixXd::Zero(6, 6);
    Quaterniond q_D      = Quaterniond(1.0, 0.0, 0.0, 0.0);
    Vector3d omega_D     = VectorXd::Zero(3, 1);
} Param;

typedef Vectord (*ode)(const double &t, const Vectord &system_states, const Param &para);

void ode45_rk(ode system_dynamics, double &t, Vectord &system_states, const Param &para);

Vectord system_dynamics(const double &t, const Vectord &system_states, const Param &para);

void write_data_to_file(const double &t, const Vectord &system_states, ofstream &file_stream);

Vector3d quaternion_to_euler(Quaterniond &q);

template <typename T> T deg2rad(T deg)
{
    return deg * M_PI / 180.0;
}

template <typename T> T rad2deg(T rad)
{
    return rad * 180.0 / M_PI;
}
#endif