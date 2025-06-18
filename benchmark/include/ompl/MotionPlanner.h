//
// Created by airlab on 10/28/24.
//

#ifndef OMPL_BOW_MOTIONPLANNER_H
#define OMPL_BOW_MOTIONPLANNER_H
#include "pch.h"
#include "bow/CollisionChecker.h"
#include "bow/ParamManager.h"
#include "model.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


namespace ompl{
    using TRAJ = std::vector<std::vector<double>>;
    class MotionPlanner {
    using CCPtr = std::shared_ptr<bow::CollisionChecker>;
    public:
        MotionPlanner(const CCPtr &cc, const ParamPtr &param, double lower, double upper, bool verbose=true);
        TRAJ plan(std::vector<float> start_pos, std::vector<float> goal_pos, double timeout=5.0);

    protected:
        void save_results(const std::stringstream& ss) const;
        bool isStateValid(const oc::SpaceInformation *si, const ob::State *state) const;
    private:
        CCPtr _cc;
        ParamPtr _param;
        bool _verbose;
        std::shared_ptr<ob::SE2StateSpace> _space;
        std::shared_ptr<oc::RealVectorControlSpace> _cspace;

        TRAJ m_findSolution(oc::SimpleSetup &ss, double timeout) const;
    };

}

#endif //OMPL_BOW_MOTIONPLANNER_H
