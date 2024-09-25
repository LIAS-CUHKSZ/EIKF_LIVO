#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>
#include <Eigen/Core>
#include <opencv4/opencv2/opencv.hpp>
#include <cmath>

// #include <common_lib.h>

#define SKEW_SYM_MATRIX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
#define mathEPS 0.000001

inline Eigen::Matrix<double, 3, 3> vec_to_hat(Eigen::Matrix<double, 3, 1> &omega)
{
    Eigen::Matrix<double, 3, 3> res_mat_33;
    res_mat_33.setZero();
    res_mat_33(0, 1) = -omega(2);
    res_mat_33(1, 0) = omega(2);
    res_mat_33(0, 2) = omega(1);
    res_mat_33(2, 0) = -omega(1);
    res_mat_33(1, 2) = -omega(0);
    res_mat_33(2, 1) = omega(0);
    return res_mat_33;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang)
{
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRIX(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRIX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRIX(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template <typename T>
Eigen::Matrix<T, 3, 1> SO3_LOG(const Eigen::Matrix<T, 3, 3> &R)
{
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if (!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    }
    else
    {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

// Barfoot, Timothy D, State estimation for robotics. Page 232-237
template <typename T = double>
inline Eigen::Matrix<T, 3, 3> left_jacobian_of_rotation_matrix(const Eigen::Matrix<T, 3, 1> &omega)
{
    Eigen::Matrix<T, 3, 3> res_mat_33;

    T theta = omega.norm();
    if (std::isnan(theta) || theta == 0)
        return Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 1> a = omega / theta;
    Eigen::Matrix<T, 3, 3> hat_a = vec_to_hat(a);
    res_mat_33 = sin(theta) / theta * Eigen::Matrix<T, 3, 3>::Identity() + (1 - (sin(theta) / theta)) * a * a.transpose() + ((1 - cos(theta)) / theta) * hat_a;
    return res_mat_33;
}

inline Eigen::Matrix<double, 6, 6> left_jacobian_of_rotation_matrix(Eigen::Matrix<double, 3, 1> &phi, Eigen::Matrix<double, 3, 1> &lou)
{
    Eigen::Matrix<double, 6, 6> res_mat_66; // res
    res_mat_66.setIdentity();

    Eigen::Matrix<double, 3, 3> J_sub_l; // J_l
    Eigen::Matrix<double, 3, 3> Q_l;     // Q

    double theta = phi.norm();
    if (std::isnan(theta) || theta == 0)
        return Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 3, 1> a = phi / theta;
    Eigen::Matrix<double, 3, 3> hat_a = vec_to_hat(a);
    J_sub_l = sin(theta) / theta * Eigen::Matrix<double, 3, 3>::Identity() + (1 - (sin(theta) / theta)) * a * a.transpose() + ((1 - cos(theta)) / theta) * hat_a;

    Eigen::Matrix<double, 3, 3> s_lou = vec_to_hat(lou);     // skew_lou
    Eigen::Matrix<double, 3, 3> s_phi = vec_to_hat(phi);     // skew_phi
    Eigen::Matrix<double, 3, 3> s_phi_2 = s_phi_2 * s_phi_2; // s_phi^2
    Eigen::Matrix<double, 3, 3> s_phi_lou = s_phi * s_lou;   // s_phi * s_lou
    Eigen::Matrix<double, 3, 3> s_lou_phi = s_lou * s_phi;   // s_lou * s_phi


    Q_l = 0.5 * s_lou + ((theta - sin(theta)) / pow(theta, 3)) * (s_phi_lou + s_lou_phi + s_phi * s_lou_phi) +
          ((theta * theta + 2 * cos(theta) - 2) / 2 * pow(theta, 4)) * (s_phi_2 * s_lou + s_lou_phi * s_phi - 3 * s_phi_lou * s_phi) +
          ((2 * theta - 3 * sin(theta) + theta * cos(theta)) / 2 * pow(theta, 5)) * (s_phi_lou * s_phi_2 + s_phi_2 * s_lou_phi);

    res_mat_66.block<3, 3>(0, 0) = J_sub_l;
    res_mat_66.block<3, 3>(3, 3) = J_sub_l;
    res_mat_66.block<3, 3>(3, 0) = Q_l;
    return res_mat_66;
}

inline Eigen::Matrix<double, 9, 9> left_jacobian_of_rotation_matrix(Eigen::Matrix<double, 3, 1> &phi, Eigen::Matrix<double, 3, 1> &lou, Eigen::Matrix<double, 3, 1> &lou1)
{
    Eigen::Matrix<double, 9, 9> res_mat_99; // res
    res_mat_99.setIdentity();

    Eigen::Matrix<double, 3, 3> J_sub_l; // J_l
    Eigen::Matrix<double, 3, 3> Q_l;     // Q

    double theta = phi.norm();
    if (std::isnan(theta) || theta <= mathEPS)
        return Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 3, 1> a = phi / theta;
    Eigen::Matrix<double, 3, 3> hat_a = vec_to_hat(a);
    J_sub_l = sin(theta) / theta * Eigen::Matrix<double, 3, 3>::Identity() + (((theta-sin(theta)) / theta)) * a * a.transpose() + ((1 - cos(theta)) / theta) * hat_a;

    Eigen::Matrix<double, 3, 3> s_lou = vec_to_hat(lou);     // skew_lou
    Eigen::Matrix<double, 3, 3> s_phi = vec_to_hat(phi);     // skew_phi
    Eigen::Matrix<double, 3, 3> s_phi_2 = s_phi_2 * s_phi_2; // s_phi^2
    Eigen::Matrix<double, 3, 3> s_phi_lou = s_phi * s_lou;   // s_phi * s_lou
    Eigen::Matrix<double, 3, 3> s_lou_phi = s_lou * s_phi;   // s_lou * s_phi


    // Q_l = 0.5 * s_lou + ((theta - sin(theta)) / pow(theta, 3)) * (s_phi_lou + s_lou_phi + s_phi * s_lou_phi) +
    //       ((theta * theta + 2 * cos(theta) - 2) / 2 * pow(theta, 4)) * (s_phi_2 * s_lou + s_lou_phi * s_phi - 3 * s_phi_lou * s_phi) +
    //       ((2 * theta - 3 * sin(theta) + theta * cos(theta)) / 2 * pow(theta, 5)) * (s_phi_lou * s_phi_2 + s_phi_2 * s_lou_phi);
    Q_l = 0.5 * s_lou+ ((theta - sin(theta)) / pow(theta, 3)) * (s_phi_lou + s_lou_phi + s_phi * s_lou_phi);

    res_mat_99.block<3, 3>(0, 0) = J_sub_l;
    res_mat_99.block<3, 3>(3, 3) = J_sub_l;
    res_mat_99.block<3, 3>(3, 0) = Q_l;
    res_mat_99.block<3, 3>(6, 6) = J_sub_l;
    res_mat_99.block<3, 3>(6, 0) = 0.5*vec_to_hat(lou1);


    return res_mat_99;
}


template <typename T = double>
inline Eigen::Matrix<T, 3, 3> left_jacobian_of_rotation_matrix(const T &v1, const T &v2, const T &v3)
{
    Eigen::Matrix<T, 3, 1> omega;
    omega << v1, v2, v3;

    Eigen::Matrix<T, 3, 3> res_mat_33;

    T theta = omega.norm();
    if (std::isnan(theta) || theta == 0)
        return Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 1> a = omega / theta;
    Eigen::Matrix<T, 3, 3> hat_a = vec_to_hat(a);
    res_mat_33 = sin(theta) / theta * Eigen::Matrix<T, 3, 3>::Identity() + (1 - (sin(theta) / theta)) * a * a.transpose() + ((1 - cos(theta)) / theta) * hat_a;
    return res_mat_33;
}

template <typename T = double>
Eigen::Matrix<T, 3, 3> inverse_left_jacobian_of_rotation_matrix(const Eigen::Matrix<T, 3, 1> &omega)
{
    Eigen::Matrix<T, 3, 3> res_mat_33;

    T theta = omega.norm();
    if (std::isnan(theta) || theta == 0)
        return Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 1> a = omega / theta;
    Eigen::Matrix<T, 3, 3> hat_a = vec_to_hat(a);
    res_mat_33 = (theta / 2) * (cot(theta / 2)) * Eigen::Matrix<T, 3, 3>::Identity() + (1 - (theta / 2) * (cot(theta / 2))) * a * a.transpose() + (theta / 2) * hat_a;
    return res_mat_33;
}

template <typename T = double>
T cot(const T theta)
{
    return 1.0 / std::tan(theta);
}


#endif
