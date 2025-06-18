//
// Created by redwan on 11/9/24.
//

#ifndef MCTS_BOW_COLLISIONCHECKER_H
#define MCTS_BOW_COLLISIONCHECKER_H
#include <vector>
#include <array>
#include <memory>
#include <fcl/fcl.h>
#include "ParamManager.h"

namespace bow{

    class CollisionChecker : public std::enable_shared_from_this<CollisionChecker> {
    public:
        using CCPtr = std::shared_ptr<CollisionChecker>;
        CollisionChecker(const std::vector<std::vector<float>>& obsList, float robotRadius, float obsLen);
        CCPtr getSharedPtr();
        ~CollisionChecker();
        bool isCollision(const std::vector<Eigen::Matrix<double, 5, 1>>&trajectory) const;
        float minDist(const std::vector<Eigen::Matrix<double, 5, 1>>& trajectory) const;

        void setTriangles(const std::vector<Triangle2D>& triangles);
    private:
        float _robotRadius;
        std::vector<fcl::CollisionObject<float>*> _obs_list;
        std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> _manager;
    };
    using CCPtr = std::shared_ptr<CollisionChecker>;

}

#endif //MCTS_BOW_COLLISIONCHECKER_H