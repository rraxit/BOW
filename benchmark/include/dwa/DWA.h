//
// Created by redwan on 3/8/25.
//

#ifndef BOW_DWA_H
#define BOW_DWA_H
#include <vector>
#include <array>
#include <memory>
#include <chrono>
#include <iostream>
#include <cmath>
#include <functional>
#include <Eigen/Dense>
#include "bow/CollisionChecker.h"
#include "bow/ParamManager.h"

namespace dwa{
    using State = std::array<double, 5>;    // x, y, yaw, velocity, yaw_rate
    using Control = std::array<double, 2>;  // velocity, yaw_rate
    using Point = std::array<double, 2>;    // x, y
    using Traj = std::vector<State>;
    using Window = std::array<double, 4>;   // min_v, max_v, min_yaw, max_yaw
    using Obstacle = std::vector<Point>;
    using ObstaclePtr = std::shared_ptr<Obstacle>;
    class DynamicWindowApproach {
    public:
        DynamicWindowApproach(const State& x, const Point& goal, bow::CCPtr cc, ParamPtr pm);
        //  Public interface
        std::pair<bool, Traj> solve(double time, bool verbose=true);
    protected:
        std::vector<Eigen::VectorXd> rolloutTrajectories();
        double computeDistanceToGoalCost(std::vector<Eigen::VectorXd> trajectory);
        double computeDistanceToObstacleCost(std::vector<Eigen::VectorXd> trajectory);
        double computeVelocityCost(std::vector<Eigen::VectorXd> trajectory);
        void predictState(Eigen::Vector2d &control, Eigen::VectorXd &state);
        std::vector<Eigen::VectorXd> calculateTrajectory(Eigen::Vector2d &control);
        void dynamicWindow();

    private:
        double pi_ = 3.14159;
        double maxLinearVelocity_ = 1;
        double minLinearVelocity_ = -0.5;
        double maxAngularVelocity_ = 40 * pi_/180;
        double maxLinearAcceleration_ = 0.2;
        double maxAngularAcceleration_ = 40 * pi_/180;
        double dt_ = 0.1;
        double windowTime_ = 4;
        double velocityResolution_ = 0.01;
        double angularVelocityResolution_ = 0.1 * pi_/180;
        double goalCostFactor_ = 3;
        double globalpathCostFactor_ = 1;
        double obstacleCostFactor_ = 1;
        double robotRadius_ = 1;
        double goalRadius_ = 1;
        int stateSize_ = 5;
        Eigen::MatrixXd transferFunction_;
        Eigen::VectorXd state_;
        Eigen::VectorXd currentState_;
        Eigen::Vector2d control_;
        std::vector<std::string> stateNames;
        std::vector<std::vector<float>> obstacles_;
        std::vector<double> currentWindow_;
        Eigen::Vector2d goal_;
        std::vector<double> origin_;
        bow::CCPtr cc_;

    };
}

#endif //BOW_DWA_H
