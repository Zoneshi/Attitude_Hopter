#include "hopter.hpp"

Vectord system_dynamics(const double &t, const Vectord &system_states, const Param &para)
{
    //|----------------------------------------------------------------------|
    //|                           Parsing variables                          |
    //|----------------------------------------------------------------------|
    //| 0:3 |   4:6   | 7:10 | 11:16 | 17:19 |  20:37 |   38:73  |   74:79   |
    //|----------------------------------------------------------------------|
    //| q_B | omega_B |  q_Q | theta |   xi  | YF_vec | YFYF_vec | YFYFtheta |
    //|----------------------------------------------------------------------|
    Quaterniond q_B    = Quaterniond(system_states(0), system_states(1), system_states(2), system_states(3)).normalized();  // current quaternion for body-fixed frame
    Vector3d omega_B   = Vector3d(system_states(4), system_states(5), system_states(6));                                    // current angular velocity for body-fixed frame
    Quaterniond q_Q    = Quaterniond(system_states(7), system_states(8), system_states(9), system_states(10)).normalized(); // current quaternion for quasi-body-fixed frame
    Vector6d theta     = {system_states(11), system_states(12), system_states(13), system_states(14), system_states(15), system_states(16)};
    Vector3d xi        = Vector3d(system_states(17), system_states(18), system_states(19));
    Vector18d YF_vec   = system_states(seq(20, 37));
    Vector36d YFYF_vec = system_states(seq(38, 73));
    Vector6d YFYFtheta = system_states(seq(74, 79));

    //------------------------------------------------------------------------
    //|                           Augmented variables                        |
    //------------------------------------------------------------------------
    Quaterniond bar_omega_B = Quaterniond(0.0, 0.5 * omega_B.x(), 0.5 * omega_B.y(), 0.5 * omega_B.z()); // current augmented angular velocity of body-fixed frame
    Vector3d euler_B        = quaternion_to_euler(q_B);                                                  // convert current quaternion to euler angles
    Vector3d omega_Q        = {omega_B.x() * cos(euler_B.z()) - omega_B.y() * sin(euler_B.z()), omega_B.x() * sin(euler_B.z()) + omega_B.y() * cos(euler_B.z()), -(omega_B.x() * sin(euler_B.z()) + omega_B.y() * cos(euler_B.z())) * tan(euler_B.y())};
    Quaterniond bar_omega_Q = Quaterniond(0.0, 0.5 * omega_Q.x(), 0.5 * omega_Q.y(), 0.5 * omega_Q.z()); // current augmented angular velocity of quasi-body-fixed frame
    Vector3d theta1         = theta(seq(0, 2));
    Vector3d theta2         = theta(seq(3, 5));
    Vector3d Y2_vec         = {omega_Q.y() * omega_Q.z(), omega_Q.x() * omega_Q.z(), omega_Q.x() * omega_Q.y()}; // diagnal elements of Y2
    Matrix3d Y2             = Y2_vec.asDiagonal();
    Matrix3d Theta          = theta1.asDiagonal();
    Matrix3d I              = MatrixXd::Identity(3, 3);
    Matrix3d ITheta         = I + Theta;
    Matrix3x6d YF           = YF_vec.reshaped(3, 6);
    Matrix6d YFYF           = YFYF_vec.reshaped(6, 6);

    //------------------------------------------------------------------------
    //|                           Consturct Controller                       |
    //------------------------------------------------------------------------
    Quaterniond q_e      = para.q_D.inverse() * q_Q;                                    // error quaternion
    Vector3d omega_e     = omega_Q - q_e.toRotationMatrix().transpose() * para.omega_D; // error angular velocity in quasi-body-fixed frame
    Vector3d omega_a     = omega_e + para.alpha_quat * q_e.vec();
    Vector3d dot_omega_r = -omega_e.cross(q_e.toRotationMatrix().transpose() * para.omega_D) - 0.5 * q_e.w() * omega_e - 0.5 * q_e.vec().cross(omega_e);
    Vector3d M_Q         = ITheta.inverse() * (-para.k_omega * omega_a - para.k_quat * q_e.vec() + dot_omega_r - Y2 * theta2);
    Matrix3d R_QB{
        { cos(euler_B.z()), sin(euler_B.z()), 0.0},
        {-sin(euler_B.z()), cos(euler_B.z()), 0.0},
        {              0.0,              0.0, 1.0}
    };
    Vector3d M_B = R_QB * M_Q;

    Matrix3d Y1 = M_Q.asDiagonal();

    //------------------------------------------------------------------------
    //|                                  ODES                                |
    //------------------------------------------------------------------------
    Matrix3x6d Y;
    Y << Y1, Y2;
    Vector3d w = para.k_quat * q_e.vec() + Y * theta;

    Quaterniond dot_q_B    = q_B * bar_omega_B;
    Vector3d dot_omega_B   = para.I.inverse() * (M_B - omega_B.cross(para.I * omega_B));
    Quaterniond dot_q_Q    = q_Q * bar_omega_Q;
    Vector6d dot_theta     = para.Gamma_omega * Y.transpose() * omega_a - para.Gamma_theta * (YFYF * theta - YFYFtheta);
    Vector3d dot_xi        = (w - (1 / para.time_tau - para.k_omega) * omega_a - xi) / para.time_tau;
    Matrix3x6d dot_YF      = (Y - YF) / para.time_tau;
    Matrix6d dot_YFYF      = YF.transpose() * YF;
    Vector6d dot_YFYFtheta = YF.transpose() * (xi + omega_a / para.time_tau);

    Vector18d dot_YF_vec   = dot_YF.reshaped(18, 1);
    Vector36d dot_YFYF_vec = dot_YFYF.reshaped(36, 1);

    Vectord dot_system_states;
    dot_system_states << dot_q_B.w(), dot_q_B.vec(), dot_omega_B, dot_q_Q.w(), dot_q_Q.vec(), dot_theta, dot_xi, dot_YF_vec, dot_YFYF_vec, dot_YFYFtheta;
    return dot_system_states;
}

void ode45_rk(ode system_dynamics, double &t, Vectord &system_states, const Param &para)
{
    Vectord temp_states = system_states;
    double temp_t       = t;
    Vectord k1          = system_dynamics(temp_t, temp_states, para);

    temp_t      = t + 0.5 * para.time_step;
    temp_states = system_states + 0.5 * para.time_step * k1;
    Vectord k2  = system_dynamics(temp_t, temp_states, para);

    temp_states = system_states + 0.5 * para.time_step * k2;
    Vectord k3  = system_dynamics(temp_t, temp_states, para);

    temp_t      = t + para.time_step;
    temp_states = system_states + 0.5 * para.time_step * k3;
    Vectord k4  = system_dynamics(temp_t, temp_states, para);

    t             = t + para.time_step;
    system_states = system_states + para.time_step * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
}

void write_data_to_file(const double &t, const Vectord &system_states, ofstream &file_stream)
{
    file_stream << t << '\t';
    for (auto data : system_states)
    {
        file_stream << data << '\t';
    }
    file_stream << endl;
}
Vector3d quaternion_to_euler(Quaterniond &q)
{
    Matrix3d dcm = q.toRotationMatrix().transpose();
    double theta = atan2(dcm(2, 0), dcm(2, 2));
    double phi   = -asin(dcm(2, 1));
    double psi   = atan2(dcm(0, 1), dcm(1, 1));
    return Vector3d(theta, phi, psi);
}