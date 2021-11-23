#include "hopter.hpp"
#include <cmath>
#include <fstream>

int main()
{
    const string file_path = "../sim_result.dat"; // save path for simulation result
    ofstream file_stream(file_path);              // corresponding file stream

    const double time_init  = 0.0;  // initial simulation time
    const double time_final = 5.0;  // terminal simulation time
    const double time_step  = 0.01; // simulation time step

    const Vector3d I_vec           = {0.1, 0.1, 0.5};                               // (3x1) nominal moment of inertia
    const Vector6d Gamma_omega_vec = {0.2, 0.2, 0.2, 0.0, 0.0, 0.0};                // (6x1) vectorized adaptive gain for Y'omega_a
    const Vector6d Gamma_theta_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                // (6x1) vectorized adaptive gain for tilde_theta
    const double k_omega           = 50.0;                                          // (1x1) feedback gain for omega_a
    const double k_quat            = 10.0;                                          // (1x1) feedback gain for q_e
    const double alpha_quat        = 10.0;                                          // (1x1) sliding surface gain for omega_a
    const double time_tau          = 0.01;                                          // (1x1) low pass filter time constant
    const Vector3d euler_D         = {deg2rad(20.0), deg2rad(-10.0), deg2rad(0.0)}; // (3x1) desired euler angle, pitch-roll-yaw (Y-X-Z)
    const Quaterniond q_D          = AngleAxisd(euler_D(0), Vector3d::UnitY()) * AngleAxisd(euler_D(1), Vector3d::UnitX()) * AngleAxisd(euler_D(2), Vector3d::UnitZ());
    const Vector3d omega_D         = {0.0, 0.0, 0.0};
    const Vector6d theta_star      = {1 / I_vec(0) - 1.0, 1 / I_vec(1) - 1.0, 1 / I_vec(1) - 1.0, (I_vec(1) - I_vec(2)) / I_vec(0), (I_vec(2) - I_vec(0)) / I_vec(1), (I_vec(1) - I_vec(0)) / I_vec(2)};
    Param para;
    para.time_step   = time_step;
    para.I           = I_vec.asDiagonal();
    para.time_tau    = time_tau;
    para.k_omega     = k_omega;
    para.alpha_quat  = alpha_quat;
    para.Gamma_omega = Gamma_omega_vec.asDiagonal();
    para.Gamma_theta = Gamma_theta_vec.asDiagonal();
    para.q_D         = q_D;
    para.omega_D     = omega_D;

    const Quaterniond q_B0        = Quaterniond(1.0, 0.0, 0.0, 0.0); // (4x1) initial quaternion of body-fixed frame
    const Vector3d omega_B0       = Vector3d(0.0, 0.0, 6 * M_PI);    // (3x1) initial angular velocity of body-fixed frame
    const Quaterniond q_Q0        = q_B0;                            // (4x1) initial quaternion of quasi-body-fixed frame
    // const Vector6d theta_init     = theta_star;                      // (6x1) initial estimation values
    const Vector6d theta_init     = {5.0, 5.0, 5.0, -2.0, 2.0, 1.0}; // (6x1) initial estimation values
    const Vector3d xi_init        = Vector3d(0.0, 0.0, 0.0);         // (3x1) initial values of xi
    const Vector18d YF_vec_init   = VectorXd::Zero(18, 1);           // (18x1) initial values of vectorized YF
    const Vector36d YFYF_vec_init = VectorXd::Zero(36, 1);           // (36x1) initial values of vectorized YFYF
    const Vector6d YFYFtheta_init = VectorXd::Zero(6, 1);            // (6x1) initial values of YFYFtheta

    Vectord system_states;
    system_states << q_B0.w(), q_B0.vec(), omega_B0, q_Q0.w(), q_Q0.vec(), theta_init, xi_init, YF_vec_init, YFYF_vec_init, YFYFtheta_init;
    double t = time_init;

    cout.precision(6); // config the terminal output format
    cout.flags(ios::fixed);
    cout.setf(ios::right);

    while (t <= time_final)
    {
        write_data_to_file(t, system_states, file_stream);

        ode45_rk(system_dynamics, t, system_states, para);
    }

    file_stream.close();
    return 0;
}