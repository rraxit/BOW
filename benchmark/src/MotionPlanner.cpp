//
// Created by airlab on 10/28/24.
//

#include "../include/ompl/MotionPlanner.h"
namespace ompl{
    MotionPlanner::MotionPlanner(const CCPtr &cc, const ParamPtr &param, double lower, double upper, bool verbose) :
            _cc(cc), _param(param), _verbose(verbose)
    {
        // Construct the robot state space in which we're planning. We're
        _space = std::make_shared<ob::SE2StateSpace>();
        // Construct a space information instance for this state space
        ob::RealVectorBounds bounds(2);
        bounds.setLow(lower);
        bounds.setHigh(upper);
        _space->setBounds(bounds);
    }

    TRAJ MotionPlanner::plan(std::vector<float> start_pos, std::vector<float> goal_pos, double timeout) {
        ob::ScopedState<> start(_space);

        // Set our robot's starting state
        start->as<ob::SE2StateSpace::StateType>()->setX(start_pos[0]);
        start->as<ob::SE2StateSpace::StateType>()->setY(start_pos[1]);
        start->as<ob::SE2StateSpace::StateType>()->setYaw(M_PI_2);

        // Set our robot's goal state
        ob::ScopedState<> goal(_space);
        goal->as<ob::SE2StateSpace::StateType>()->setX(goal_pos[0]);
        goal->as<ob::SE2StateSpace::StateType>()->setY(goal_pos[1]);
        goal->as<ob::SE2StateSpace::StateType>()->setYaw(M_PI_2);

        // create a control space
        _cspace = std::make_shared<UnicycleControlSpace>(_space);

        // set the bounds for the control space
        // since u, w have two different domains, control will be generated in normalized domain [0, 1]
        // we then map it to u and w domains
        ob::RealVectorBounds cbounds(2);
        cbounds.setLow(0.0);
        cbounds.setHigh(1.0);

        _cspace->setBounds(cbounds);

        // define a simple setup class
        oc::SimpleSetup ss(_cspace);
        auto si = ss.getSpaceInformation();
        ob::PlannerPtr planner(new oc::RRT(si));
        ss.setPlanner(planner);

        // set state validity checking for this space
        ss.setStateValidityChecker(
                [&](const ob::State *state) { return this->isStateValid(si.get(), state); });

        // Setting the propagation routine for this space:
        // KinematicCarModel does NOT use ODESolver
        ss.setStatePropagator(std::make_shared<KinematicCarModel>(ss.getSpaceInformation(), _param));
        ss.setStartAndGoalStates(start, goal, _param->get_param<double>("goal_radius"));
        ss.setup();

        return m_findSolution(ss, timeout);

    }

    // TRAJ MotionPlanner::m_findSolution(oc::SimpleSetup &ss, double timeout) const {
    //     ob::PlannerStatus solved = ss.solve(timeout);

    //     TRAJ traj;

    //     if (solved)
    //     {
    //         if(_verbose)
    //         {
    //             std::cout << "Found solution:" << std::endl;
    //             ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    //         }
    //         ss.getSolutionPath().interpolate();
    //         const auto &pathStates = ss.getSolutionPath().getStates();
    //         for (size_t i = 0; i < pathStates.size(); ++i)
    //         {
    //             // Get the state
    //             const ob::State* state = pathStates[i];
    //             // Convert the state to a vector of doubles
    //             std::vector<double> stateVector;
    //             _space->copyToReals(stateVector, state);
    //             // Add the state to the trajectory
    //             traj.push_back(stateVector);
    //         }
    //     }
    //     return traj;
    // }

    TRAJ MotionPlanner::m_findSolution(oc::SimpleSetup &ss, double timeout) const {
        ob::PlannerStatus solved = ss.solve(timeout);
    
        TRAJ traj;
    
        if (solved)
        {
            if (_verbose)
            {
                std::cout << "Found solution:" << std::endl;
                ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
            }
    
            // Downcast to PathControl to access controls and durations
            const auto *pathControl = dynamic_cast<const oc::PathControl*>(&ss.getSolutionPath());
            if (!pathControl)
            {
                std::cerr << "Error: Solution path is not of type PathControl!" << std::endl;
                return traj;
            }
    
            for (std::size_t i = 0; i < pathControl->getStateCount(); ++i)
            {
                const ob::State* state = pathControl->getState(i);
                std::vector<double> stateVector;
                _space->copyToReals(stateVector, state);
    
                std::vector<double> controlVector;
                double duration = 0.0;
    
                if (i < pathControl->getControlCount())
                {
                    const oc::Control* control = pathControl->getControl(i);
                    const auto* rvc = static_cast<const oc::RealVectorControlSpace::ControlType*>(control);
                    controlVector.assign(rvc->values, rvc->values + _cspace->getDimension());
                    duration = pathControl->getControlDuration(i);
                }
    
                // Merge state + control + duration into a single vector
                std::vector<double> stepData = stateVector;
                stepData.insert(stepData.end(), controlVector.begin(), controlVector.end());
                stepData.push_back(duration);
    
                traj.push_back(stepData);
            }
        }
        return traj;
    }
    
    

    void MotionPlanner::save_results(const std::stringstream &ss) const {
        std::ofstream outFile("output.csv");
        if (outFile.is_open()) {
            // Write the stringstream contents to the file
            outFile << ss.str();
            // Close the file
            outFile.close();
            std::cout << "Stringstream contents have been written to output.csv" << std::endl;
        } else {
            std::cerr << "Unable to open file for writing" << std::endl;
        }
    }

    bool MotionPlanner::isStateValid(const oc::SpaceInformation *si, const ob::State *state) const {
        float x = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getX());
        float y = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getY());
        float yaw = static_cast<float>(state->as<ob::SE2StateSpace::StateType>()->getYaw());
        Eigen::Matrix<double, 5, 1> state_vec;
        state_vec << x, y, yaw, 0.0, 0.0;
        std::vector<Eigen::Matrix<double, 5, 1>> trajectory = {state_vec};
        return !_cc->isCollision(trajectory);
    }

}
