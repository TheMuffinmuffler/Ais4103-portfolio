//
// Created by William Haugland on 27/11/2025.
//

#ifndef AIS4104_PORTFOLIO_PTP_CUBIC_TRAJECTORY_GENERATOR_H
#define AIS4104_PORTFOLIO_PTP_CUBIC_TRAJECTORY_GENERATOR_H

#include <simulation/trajectorygenerator.h>
#include <simulation/robot.h>
#include <Eigen/Dense>
#include <simulation/trajectorygenerator.h>

    namespace AIS4104 {

        class PTPCubicTrajectoryGenerator : public Simulation::PointToPointTrajectoryGenerator
        {
        public:
            using PointToPointTrajectoryGenerator::PointToPointTrajectoryGenerator;

            void stop() override;
            bool has_reached_endpoint() const override;

            bool plan_trajectory(const Simulation::JointLimits &limits, double velocity_factor) override;

            Eigen::VectorXd joint_positions(std::chrono::nanoseconds delta_t) override;

        private:
            std::atomic<bool> m_stopped;
            std::chrono::nanoseconds m_total;
            std::chrono::nanoseconds m_current;
        };

    }

#endif
