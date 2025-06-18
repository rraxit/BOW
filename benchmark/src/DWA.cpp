//
// Created by redwan on 3/8/25.
//

#include "../include/dwa/DWA.h"
namespace dwa{

    DynamicWindowApproach::DynamicWindowApproach(const State &x, const Point &goal, bow::CCPtr cc, ParamPtr pm)
    : cc_(cc){
        maxLinearVelocity_ = pm->get_param<double>("max_speed");
        minLinearVelocity_ = pm->get_param<double>("min_speed");
        maxAngularVelocity_ = pm->get_param<double>("max_yawrate");
        maxLinearAcceleration_ = pm->get_param<double>("max_accel");
        maxAngularAcceleration_ = pm->get_param<double>("max_dyawrate");
        dt_ = pm->get_param<double>("dt");
        windowTime_ = pm->get_param<double>("predict_time");
        velocityResolution_ = pm->get_param<double>("v_reso");
        angularVelocityResolution_ = pm->get_param<double>("yawrate_reso");
        goalCostFactor_ = pm->get_param<double>("to_goal_cost_gain");
        globalpathCostFactor_ = pm->get_param<double>("speed_cost_gain");
        obstacleCostFactor_ = pm->get_param<double>("to_obstacle_cost_gain");
        robotRadius_ = pm->get_param<double>("robot_radius");
        goalRadius_ = pm->get_param<double>("goal_radius");
        origin_ = pm->get_param<std::vector<double>>("origin");

        // make all the coordinates relative to origin and positive values
        // Initial State
        currentState_.resize(stateSize_);
        currentState_<< x[0] - origin_[0], x[1] - origin_[1], x[2], 0, 0;
        // Goal
        goal_<< goal[0] - origin_[0], goal[1] - origin_[1];

    }


    void DynamicWindowApproach::predictState(Eigen::Vector2d &control, Eigen::VectorXd &state){
        state(2) += control(1) * dt_;
        state(0) += control(0) * std::cos(state(2)) * dt_;
        state(1) += control(0) * std::sin(state(2)) * dt_;
        state(3) = control(0);
        state(4) = control(1);
    }

    void DynamicWindowApproach::dynamicWindow(){
        currentWindow_.clear();
        double currLinearVelocity = currentState_(3);
        double currTheta = currentState_(4);

        // Velocity and Theta Increment with maximum Linear and angular Acceleration
        double linearVelocityIncrement = maxLinearAcceleration_ * dt_;
        double angularVelocityIncrement = maxAngularAcceleration_ * dt_;

        // Below linear velocity range will contain all possible linear velocities produced by diffrent linear accelerations constrained by maximum linear velocity and acceleration
        // Min possible velocity in dt
        currentWindow_.push_back(std::max(currLinearVelocity - linearVelocityIncrement, minLinearVelocity_));
        // Max possible velocity in dt
        currentWindow_.push_back(std::min(currLinearVelocity + linearVelocityIncrement, maxLinearVelocity_));


        // Below angular velocity range will contain all possible angular velocities produced by diffrent angular accelerations constrained by maximum angular velocity and acceleration
        // Min possible theta in dt
        currentWindow_.push_back(std::max(currTheta - angularVelocityIncrement, -maxAngularVelocity_));
        // Max possible theta in dt
        currentWindow_.push_back(std::min(currTheta + angularVelocityIncrement, maxAngularVelocity_));
    }

    std::vector<Eigen::VectorXd> DynamicWindowApproach::calculateTrajectory(Eigen::Vector2d &control){
        std::vector<Eigen::VectorXd> trajectory;
        double time = 0;
        int i =0;
        Eigen::VectorXd state = currentState_;
        trajectory.push_back(state);
        while(time <= windowTime_){
            predictState(control, state);
            trajectory.push_back(state);

            time = time + dt_;
            i++;
        }
        return trajectory;
    }

    std::vector<Eigen::VectorXd> DynamicWindowApproach::rolloutTrajectories(){
        double cost = 0;
        double minCost = INT_MAX;
        std::vector<Eigen::VectorXd> bestTrajectory;
        // Find all trajectories with sampled input in dynamic window
        for(double i = currentWindow_[0]; i<=currentWindow_[1]; i = i + velocityResolution_){
            for(double j = currentWindow_[2]; j<=currentWindow_[3]; j = j + angularVelocityResolution_){
                Eigen::Vector2d control(i, j);
                std::vector<Eigen::VectorXd> trajectory;
                trajectory = calculateTrajectory(control);

                // Compute cost for the trajectory
                cost = computeDistanceToGoalCost(trajectory) + computeDistanceToObstacleCost(trajectory) + computeVelocityCost(trajectory);

                if(cost < minCost){
                    minCost = cost;
                    bestTrajectory = trajectory;
                }
            }
        }
        return bestTrajectory;
    }


    double DynamicWindowApproach::computeDistanceToGoalCost(std::vector<Eigen::VectorXd> trajectory){
        double goalCost;

        // Cost: Distance to goal (using error Angle)
        double startGoalVector = std::sqrt(goal_(0)*goal_(0) + goal_(1)*goal_(1));
        double startCurrentVector = std::sqrt(std::pow(trajectory.back()(0), 2) + std::pow(trajectory.back()(1), 2));
        double distance = std::sqrt(std::pow(trajectory.back()(0)-goal_(0),2) + std::pow(trajectory.back()(1)-goal_(1),2));
        double dotProduct = (goal_(0) * trajectory.back()(0))+ (goal_(1) * trajectory.back()(1));
        double cosTheta = dotProduct / (startGoalVector * startCurrentVector);
        double theta = std::acos(cosTheta);
        goalCost = goalCostFactor_ * (distance*0.1 + theta);

        return goalCost;
    }

    double DynamicWindowApproach::computeDistanceToObstacleCost(std::vector<Eigen::VectorXd> trajectory){
        double obstacleCost = INT_MAX;
        double minimumSeparation = INT_MAX;

        // Cost: Distance to Obstacle
        std::vector<Eigen::Matrix<double, 5, 1>> ccTrajectory;
        for (int i=0; i<trajectory.size(); i++){
            Eigen::Matrix<double, 5, 1> state;
            state << trajectory[i](0) + origin_[0], trajectory[i](1) + origin_[1], trajectory[i](2), trajectory[i](3), trajectory[i](4);
            ccTrajectory.push_back(state);
        }

        minimumSeparation = cc_->minDist(ccTrajectory);
        if (minimumSeparation <= robotRadius_){
            return obstacleCost;
        }
        obstacleCost = 1 / minimumSeparation;
        obstacleCost = obstacleCost * obstacleCostFactor_;

        return obstacleCost;
    }

    double DynamicWindowApproach::computeVelocityCost(std::vector<Eigen::VectorXd> trajectory){
        double velocityCost = maxLinearVelocity_ - trajectory.back()[3];
        return velocityCost;
    }



    std::pair<bool, Traj> DynamicWindowApproach::solve(double time, bool verbose) {
        bool goalNotReached = true;
        std::vector<Eigen::VectorXd> bestTrajectory;
        auto start_time = std::chrono::high_resolution_clock::now();
        Traj final_result;

        final_result.push_back({currentState_(0) + origin_[0], currentState_(1) + origin_[1], currentState_(2), currentState_(3), currentState_(4)});
        long elapsed_time = 0;

        while(goalNotReached){
            // find dynamic window
            dynamicWindow();

            // find Trajectories as well as find best of all those based on cost functions
            bestTrajectory = rolloutTrajectories();
            // Send the control commond to robot that leads to bestTrajectory for time dt
            control_(0) = bestTrajectory[1](3);
            control_(1) = bestTrajectory[1](4);
            predictState(control_, currentState_);
            // update trajectory
            final_result.push_back({currentState_(0) + origin_[0], currentState_(1) + origin_[1], currentState_(2), currentState_(3), currentState_(4)});

            // update current state using state transition function
            auto end_time = std::chrono::high_resolution_clock::now();
            // Calculate duration
             elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            // Check if robot is within goal threshold radius
            if(std::sqrt(std::pow(currentState_(0) - goal_(0), 2) + std::pow(currentState_(1) - goal_(1), 2) <= goalRadius_)){
                goalNotReached = true;
                break;
            }
            else if (double(elapsed_time / 10.0e3) > time){
                goalNotReached = false;
                break;
            }
        }
        if(verbose)
            std::cout << "[DWA]: Solution found in time: " << elapsed_time << " ms" << std::endl;

        return std::make_pair(goalNotReached, final_result);
    }
}
