#include "utility/math.h"
#include "utility/vectors.h"
#include <iostream>
#include <numbers>

namespace AIS4104::utility {

    // from Assignment 2
    double cot(double x)
    {
        return 1 /(std::sin(x) / std::cos(x));
    }
    //Equation R(⍺,β,𝛄) = I * Rot(ẑ,⍺) * Rot(ŷ,β) * Rot(x̂,𝛄)
    //page 577 MR pre-print 2019


Eigen::Vector3d euler_zyx_from_rotation_matrix(const Eigen::Matrix3d &r)
    {
        Eigen::Vector3d euler_zyx;
        double alpha, beta, gamma;

        beta  = atan2(-r(2,0),sqrt(std::pow(r(0,0),2.0)+std::pow(r(1,0),2.0)));
        alpha = atan2(r(1, 0), r(0, 0));
        gamma = atan2(-r(2, 1), r(2, 2));

        euler_zyx << alpha,  beta,  gamma;

        return euler_zyx;
}

    // Formula (3.30) Page 75 MR pre-print 2019
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d skew;
    skew <<
                0,      -v.z(),  v.y(),
             v.z(),      0,     -v.x(),
            -v.y(),  v.x(),      0;
    return skew;
}
    // Formula (3.30) Page 75 MR pre-print 2019
    //check
Eigen::Vector3d from_skew_symmetric(const Eigen::Matrix3d &m)
{
        Eigen::Vector3d v;
        v << (m(2,1), m(0,2), m(1,0));
        return v;
}
    // Definition 3.20 on page 98, MR pre-print 2019
Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
       Eigen::Matrix3d p_hat = skew_symmetric(p);
        Eigen::MatrixXd AdT(6,6);
        AdT.setZero();
        AdT.block<3,3>(0,0) = r;
        AdT.block<3,3>(3,0) = p_hat * r;
        AdT.block<3,3>(3,3) = r;


        return AdT;
}
    // Definition 3.20 on page 98, MR pre-print 2019
Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf)
{
    Eigen::Matrix3d R = tf.block<3,3>(0,0);  //could have used T.TopLeftCorner instead of T.block
    Eigen::Vector3d p = tf.block<3,1>(0,3);

    Eigen::Matrix3d p_hat = skew_symmetric(p);

    Eigen::MatrixXd AdT(6,6);
    AdT.setZero();
    AdT.block<3,3>(0,0) = R;
    AdT.block<3,3>(3,0) = p_hat * R;
    AdT.block<3,3>(3,3) = R;


    return AdT;
}
    // Definition 3.20 on page 98, MR pre-print 2019

Eigen::VectorXd adjoint_map(const Eigen::VectorXd &twist, const Eigen::Matrix4d &tf)
{
        Eigen::Matrix3d R = tf.block<3,3>(0,0);
        Eigen::Vector3d p = tf.block<3,1>(0,3);
        Eigen::Matrix3d p_hat = skew_symmetric(p);

        Eigen::MatrixXd AdT(6,6);
        AdT.setZero();
        AdT.block<3,3>(0,0) = R;
        AdT.block<3,3>(3,0) = p_hat * R;
        AdT.block<3,3>(3,3) = R;

        return AdT * twist;
}
    // Equation 3.70 page 96, MR pre-print 2019
Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::VectorXd V(6);
    V << w, v;
    return V;
}
//// Un named Equation page 101, MR pre-print 2019
///Equation 3.71 page 96, MR pre-print 2019
Eigen::VectorXd twist(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
        Eigen::VectorXd twist(6);
        Eigen::Vector3d w = s;
        Eigen::Vector3d v = -w.cross(q) + h * w;
        twist.head<3>() = w * angular_velocity;
        twist.tail<3>() = v * angular_velocity;
        return twist;
}
    ///Equation 3.71 page 101, MR pre-print 2019
Eigen::Matrix4d twist_matrix(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
        Eigen::Matrix4d twist_hat = Eigen::Matrix4d::Zero();
        Eigen::Matrix3d w_hat = skew_symmetric(w);
        twist_hat.block<3,3>(0,0) = w_hat;
        twist_hat.block<3,1>(0,3) = v;
        return twist_hat;
}
    ///Equation 3.71 page 101, MR pre-print 2019
Eigen::Matrix4d twist_matrix(const Eigen::VectorXd &twist)
{
        Eigen::Matrix4d twist_hat = Eigen::Matrix4d::Zero();
        Eigen::Vector3d w = twist.head<3>();
        Eigen::Vector3d v = twist.tail<3>();

        Eigen::Matrix3d w_hat = skew_symmetric(w);


        twist_hat.block<3,3>(0,0) = w_hat;
        twist_hat.block<3,1>(0,3) = v;
        return twist_hat;
}
    // Definition 3.24 page 102 MR pre-print 2019 Si=(w,v)
Eigen::VectorXd screw_axis(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::VectorXd S(6);
        S.head<3>() = w;
        S.tail<3>() = v;
        return S;
}
    // Equation on page 101, MR pre-print 2019
Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    Eigen::Vector3d v = -s.cross(q) + h * s;
    Eigen::VectorXd S(6);
    S << s, v;
    return S;
}
    // Equation 3.51 page 82, MR pre-print 2019
Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta)
{

    Eigen::Vector3d u = w.normalized();
    Eigen::Matrix3d u_hat = skew_symmetric(u);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
         + std::sin(theta) * u_hat
         + (1.0 - std::cos(theta)) * (u_hat * u_hat);
    return R;
}
    // Proposition 3.25 on page 103, MR pre-print 2019

Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta)
{
        /*
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector3d u = w.normalized();
    Eigen::Matrix3d u_hat = skew_symmetric(u);
    Eigen::Matrix3d R = matrix_exponential(u, theta);
    Eigen::Matrix3d G = Eigen::Matrix3d::Identity()*theta
                      + (1.0 - std::cos(theta)) * u_hat
                      + (theta - std::sin(theta)) * (u_hat * u_hat);
                      */
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        if (w.norm()< 1e-6) {
            T.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
            T.block<3,1>(0,3) = v * theta;
        }
        else{
            Eigen::Vector3d u = w.normalized();
            Eigen::Matrix3d u_hat = skew_symmetric(u);
            Eigen::Matrix3d R =matrix_exponential(w,theta);
            Eigen::Matrix3d G = Eigen::Matrix3d::Identity() * theta
            + (1.0-std::cos(theta)) * u_hat + (theta-std::sin(theta))*(u_hat * u_hat);
            T.block<3,3>(0,0) = R;
            T.block<3,1>(0,3) = G * v;
        }

    return T;
}
    // Proposition 3.25 on page 103-104, MR pre-print 2019
Eigen::Matrix4d matrix_exponential(const Eigen::VectorXd &screw, double theta)
{

        Eigen::Vector3d w = screw.head<3>();
        Eigen::Vector3d v = screw.tail<3>();
        Eigen::Matrix4d T = matrix_exponential(w, v, theta);
    return T;
}
    // Equations 3.58 - 3.61, pages 85 - 86, MR pre-print 2019

std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r)
{
    double theta;
    Eigen::Vector3d w;

    if (r == Eigen::Matrix3d::Identity()) {
        theta = 0;
    }
    else {
        double trace_r = r.trace();
        if (trace_r ==-1){
            theta = std::numbers::pi;
            w= (1/sqrt(2*(1+r(2,2))))*Eigen::Vector3d (r(0,2), r(1,2),1+ r(2,2));
        }
        else {
            theta = acos(0.5*(trace_r-1));
            double w_n = 1/2*std::sin(theta);
            double w_1 = w_n * (r(2,1)-r(1,2));
            double w_2 = w_n * (r(0,2)-r(2,0));
            double w_3 = w_n * (r(1,0)-r(0,1));
            w = Eigen::Vector3d(w_1,w_2,w_3);
        }
    }
    return std::pair<Eigen::Vector3d, double>(w, theta);
}
    // Algorithm on page 104, MR pre-print 2019
std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
    {
        Eigen::Vector3d w;
        Eigen::Vector3d v;
        double theta;

        if (r.isApprox(Eigen::Matrix3d::Identity())) {
            w = Eigen::Vector3d::Zero();
            theta = p.norm();
            v = p / theta;

        }else {
            std::pair<Eigen::Vector3d, double> m_log = matrix_logarithm(r);
            w = m_log.first;
            theta = m_log.second;

            Eigen::Matrix3d w_hat = skew_symmetric(w);
            double cot_half_theta = cot(theta / 2.0);

            Eigen::Matrix3d G_inv = (1.0 / theta) * Eigen::Matrix3d::Identity()
                                    - 0.5 * w_hat
                                    + ((1.0 / theta) - 0.5 * cot_half_theta) * (w_hat * w_hat);
            v = G_inv * p;
        }
        Eigen::VectorXd S(6);
        S << w, v;

        return std::pair<Eigen::VectorXd, double>(S, theta);

}
    // Algorithm on page 104, MR pre-print 2019

std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &tf)
    {
        const Eigen::Matrix3d R = tf.topLeftCorner<3,3>();
        const Eigen::Vector3d p = tf.topRightCorner<3,1>();

        Eigen::Vector3d w;
        Eigen::Vector3d v;
        double theta;

        if ((R - Eigen::Matrix3d::Identity()).norm() < 1e-12) {
            w = Eigen::Vector3d::Zero();
            v = p / p.norm();
            theta = p.norm();
        } else {
            std::pair<Eigen::Vector3d, double> m_log = matrix_logarithm(R);
            w = m_log.first;
            theta = m_log.second;

            const Eigen::Matrix3d skew_w = skew_symmetric(w);
            double cot_half_theta = cot(theta / 2.0);

            Eigen::Matrix3d G_inv = (1.0 / theta) * Eigen::Matrix3d::Identity()
                                  - 0.5 * skew_w
                                  + ((1.0 / theta) - 0.5 * cot_half_theta) * skew_w * skew_w;

            v = G_inv * p;
        }

        Eigen::VectorXd S(6);
        S << w, v;
        return std::pair<Eigen::VectorXd, double>(S, theta);
}
    // Algorithm on page 72, MR pre-print 2019
Eigen::Matrix3d rotate_x(double radians)
    {Eigen::Matrix3d m;
        m << 1,        0,         0,
             0,  std::cos(radians), -std::sin(radians),
             0,  std::sin(radians),  std::cos(radians);
        return m;
}
    // Algorithm on page 72, MR pre-print 2019
Eigen::Matrix3d rotate_y(double radians)
    {
        Eigen::Matrix3d m;
        m <<  std::cos(radians), 0, std::sin(radians),
                     0, 1,       0,
             -std::sin(radians), 0, std::cos(radians);
        return m;
}
    // Algorithm on page 72, MR pre-print 2019
Eigen::Matrix3d rotate_z(double radians)
    {
        Eigen::Matrix3d m;
        m << std::cos(radians), -std::sin(radians), 0,
             std::sin(radians),  std::cos(radians), 0,
                   0,         0, 1;
        return m;
}
    //Using equation (3.16) R= [ˆxb ˆyb ˆzb] Page 65 MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
    {
        Eigen::Matrix3d m;
        m.col(0) = x.normalized();
        m.col(1) = y.normalized();
        m.col(2) = z.normalized();
        return m;
    }

//using Equation R(⍺,β,𝛄) = Rot(ẑ,⍺) * Rot(ŷ,β) * Rot(x̂,𝛄) page 577 MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
    double alpha = e(0);
    double beta  = e(1);
    double gamma = e(2);

    // Use previously defined functions
    Eigen::Matrix3d Rz = rotate_z(alpha);
    Eigen::Matrix3d Ry = rotate_y(beta);
    Eigen::Matrix3d Rx = rotate_x(gamma);

    // Apply in ZYX order
    Eigen::Matrix3d R = Rz * Ry * Rx;

    return R;
}
//Page 82 MR pre-print 2019
//using equation (3.51) Rot(ˆ ω,θ) = e[ˆ ω] θ= I+ sin θ[ˆ ω] + (1−cos θ)[ˆ ω]2 ∈SO(3)
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    Eigen::Vector3d w = axis.normalized();

    Eigen::Matrix3d w_hat = skew_symmetric(w);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
                        + std::sin(radians) * w_hat
                        + (1 - std::cos(radians)) * (w_hat * w_hat);
    return R;
}

    //Equation (3.62) Page 87 MR pre-print 2019
Eigen::Matrix3d rotation_matrix(const Eigen::Matrix4d &tf)
{
        Eigen::Matrix3d R = tf.block<3,3>(0,0);
    return R;
}

//Equation (3.62) Page 87 MR pre-print 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Vector3d &p)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,1>(0,3) = p;
    return T;
}

// Equation (3.62) Page 87 MR pre-print 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r)
{Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = r;
    return T;
}
//Equation (3.62) Page 87 MR pre-print 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = r;
    T.block<3,1>(0,3) = p;

    return T;
}

}