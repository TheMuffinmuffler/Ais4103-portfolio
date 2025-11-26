#include "app/robotwrapper.h"

#include <utility/math.h>
#include <utility/vectors.h>

using namespace AIS4104;

RobotWrapper::RobotWrapper(std::shared_ptr<threepp::Robot> robot, std::shared_ptr<Simulation::KinematicsSolver> solver)
    : m_tool_transform(Eigen::Matrix4d::Identity())
    , m_robot(std::move(robot))
    , m_solver(std::move(solver))
{
}

threepp::Robot& RobotWrapper::threepp_robot()
{
    return *m_robot;
}

std::shared_ptr<threepp::Robot> RobotWrapper::threepp_robot_ptr()
{
    return m_robot;
}

const threepp::Robot& RobotWrapper::threepp_robot() const
{
    return *m_robot;
}

uint8_t RobotWrapper::joint_count() const
{
    return m_solver->joint_count();
}

//TASK: Implement the function to calculate the joint positions for the desired tool pose
// a) Use m_tool_transform to calculate the flange pose required by m_solver.ik_solve()
// b) Use the m_solver.ik_solve() overload with the solution selector lambda to choose the most desirable IK solution.
Eigen::VectorXd RobotWrapper::ik_solve_pose(const Eigen::Matrix4d &eef_pose, const Eigen::VectorXd &j0) const
{
    //a
    Eigen::Matrix4d flange_pose = eef_pose * m_tool_transform.inverse();
    return ik_solve_flange_pose(flange_pose,j0);
}
//b
/*
Eigen::VectorXd RobotWrapper::ik_solve_flange_pose(const Eigen::Matrix4d &flange_pose, const Eigen::VectorXd &j0) const {

    return m_solver->ik_solve(flange_pose,j0,[&](const std::vector<Eigen::VectorXd> & candidates) -> uint_fast32_t {

        if (candidates.empty()) return 0;

        uint32_t best_inx = 0;
        double min_dist_squared = (candidates[0]-j0).squaredNorm();

        for (uint32_t i = 1; i < candidates.size(); ++i) {
            double dist_squared = (candidates[i]-j0).squaredNorm();
            if (dist_squared < min_dist_squared) {
                min_dist_squared = dist_squared;
                best_inx = i;
            }
        }
        return best_inx;
    });
}
*/

//TASK: Implement the function to calculate the joint positions for the desired flange pose
// a) Use m_tool_transform to calculate the flange pose required by m_solver.ik_solve()
// b) Use the m_solver.ik_solve() overload with the solution selector lambda to choose the most desirable IK solution.
Eigen::VectorXd RobotWrapper::ik_solve_flange_pose(const Eigen::Matrix4d &flange_pose, const Eigen::VectorXd &j0) const {

    return ik_solve_flange_pose(flange_pose,j0);
}
/*
    Eigen::Matrix4d tool_pose = flange_pose * m_tool_transform;
    Eigen::VectorXd solution = m_solver->ik_solve(tool_pose, j0, [&](const std::vector<Eigen::VectorXd> & candidates) -> uint_fast32_t {
        if (candidates.empty()) return 0u;

        uint32_t best_guess = 0;
        double best_dist = (candidates[0]-j0).norm();

        for (uint32_t i = 1; i < candidates.size(); ++i) {
            double dist = (candidates[i]-j0).norm();
            if (dist < best_dist) {
                best_dist = dist;
                best_guess = i;
            }
        }
        return best_guess;

    }
    );
    return solution;
}
*/

Eigen::Matrix4d RobotWrapper::tool_transform() const
{
    return m_tool_transform;
}

void RobotWrapper::set_tool_transform(Eigen::Matrix4d transform)
{
    m_tool_transform = std::move(transform);
}

//TASK: Calculate the pose of the end effector using forward kinematics;
// Relevant variables are m_solver and m_tool_transform.
Eigen::Matrix4d RobotWrapper::current_pose() const
{
    return Eigen::Matrix4d::Identity();
}

//TASK: Calculate the position of the end effector using forward kinematics.
// Relevant variables are m_solver and m_tool_transform (or possibly another function of RobotWrapper?).
Eigen::Vector3d RobotWrapper::current_position() const
{
    return Eigen::Vector3d::Zero();
}

//TASK: Calculate the orientation of the end effector using forward kinematics and m_solver (or rely on another function of RobotWrapper?).
Eigen::Vector3d RobotWrapper::current_orientation_zyx() const
{
    return Eigen::Vector3d::Zero();
}

//TASK: Calculate the pose of the end effector using forward kinematics and m_solver.
Eigen::Matrix4d RobotWrapper::current_flange_pose() const
{
    Eigen::VectorXd current_joints = joint_positions();
    Eigen::Matrix4d flange_pose = m_solver->fk_solve(current_joints);
    return flange_pose;
}

//TASK: Based on the flange pose, return its linear position.
Eigen::Vector3d RobotWrapper::current_flange_position() const
{
   Eigen::Matrix4d T = current_flange_pose();
    Eigen::Vector3d p = T.block<3,1>(0,3);

    return p;
}

//TASK: Based on the flange pose, return its orientation in the Euler ZYX representation.
Eigen::Vector3d RobotWrapper::current_flange_orientation_zyx() const
{
    Eigen::Matrix4d T = current_flange_pose();
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d euler_ZYX = utility::euler_zyx_from_rotation_matrix(R);
    return euler_ZYX;
}

const Simulation::JointLimits& RobotWrapper::joint_limits() const
{
    return m_solver->joint_limits();
}

Eigen::VectorXd RobotWrapper::joint_positions() const
{
    return utility::to_eigen_vectord(m_robot->jointValues());
}

void RobotWrapper::set_joint_positions(const Eigen::VectorXd &joint_positions)
{
    m_robot->setJointValues(utility::to_std_vectorf(joint_positions));
}
