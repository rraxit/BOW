#include "bow/CollisionChecker.h"
#include <Eigen/Geometry>
// #define DEBUG(x) std::cout << x << std::endl
// #define DEBUG(x) do { } while (0) // Disable debug output

namespace bow{
    CollisionChecker::CollisionChecker(const std::vector<std::vector<float>>& obsList,
                                       float robotRadius, float obsLen)
            : _robotRadius(robotRadius)
    {
        // Initialize the collision manager first
        _manager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();

        if (!obsList.empty()) {
            auto geom = std::make_shared<fcl::Box<float>>(obsLen, obsLen, obsLen);
            _obs_list.reserve(obsList.size());

            for (const auto& o : obsList) {
                fcl::Transform3f pose = fcl::Transform3f::Identity();
                pose.linear() = Eigen::Quaternionf::Identity().matrix();
                pose.translation() = Eigen::Vector3f(float(o.at(0)), float(o.at(1)), 0.5f);
                _obs_list.emplace_back(new fcl::CollisionObject<float>(geom, pose));
            }

            _manager->registerObjects(_obs_list);
            _manager->setup();
        } else {
            // DEBUG("Warning: No obstacles provided to CollisionChecker.");
        }
    }

    CollisionChecker::CCPtr CollisionChecker::getSharedPtr()
    {
        return shared_from_this();
    }

    bool CollisionChecker::isCollision(const std::vector<Eigen::Matrix<double, 5, 1>> &trajectory) const {
        // Check if we have any obstacles registered
        if (_obs_list.empty()) {
            // DEBUG("Warning: No obstacles registered for collision checking");
            return false;
        }

        auto geom = std::make_shared<fcl::Sphere<float>>(_robotRadius);
        std::vector<fcl::CollisionObject<float>*> trajectory_objects;
        trajectory_objects.reserve(trajectory.size());

        for (const auto& state : trajectory) {
            fcl::Transform3f pose = fcl::Transform3f::Identity();
            pose.linear() = Eigen::Quaternionf::Identity().matrix();
            pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), 0.0f); // Changed to 0.0f for 2D
            trajectory_objects.emplace_back(new fcl::CollisionObject<float>(geom, pose));
        }

        fcl::DynamicAABBTreeCollisionManager<float> trajectory_manager;
        trajectory_manager.registerObjects(trajectory_objects);
        trajectory_manager.setup();

        fcl::DefaultCollisionData<float> collision_data;
        trajectory_manager.collide(_manager.get(), &collision_data, fcl::DefaultCollisionFunction);

        bool has_collision = collision_data.result.isCollision();

        // Clean up trajectory objects
        for (auto* obj : trajectory_objects) {
            delete obj;
        }

        return has_collision;
    }

    float CollisionChecker::minDist(const std::vector<Eigen::Matrix<double, 5, 1>> &trajectory) const {
        // Check if we have any obstacles registered
        if (_obs_list.empty()) {
            // DEBUG("Warning: No obstacles registered for distance checking");
            return std::numeric_limits<float>::max();
        }

        auto geom = std::make_shared<fcl::Sphere<float>>(_robotRadius);
        std::vector<fcl::CollisionObject<float>*> trajectory_objects;
        trajectory_objects.reserve(trajectory.size());

        for (const auto& state : trajectory) {
            fcl::Transform3f pose = fcl::Transform3f::Identity();
            pose.linear() = Eigen::Quaternionf::Identity().matrix();
            pose.translation() = Eigen::Vector3f(float(state(0)), float(state(1)), 0.0f); // Changed to 0.0f for 2D
            trajectory_objects.emplace_back(new fcl::CollisionObject<float>(geom, pose));
        }

        fcl::DynamicAABBTreeCollisionManager<float> trajectory_manager;
        trajectory_manager.registerObjects(trajectory_objects);
        trajectory_manager.setup();

        fcl::DefaultDistanceData<float> distance_data;
        trajectory_manager.distance(_manager.get(), &distance_data, fcl::DefaultDistanceFunction);

        float min_distance = distance_data.result.min_distance;

        // Clean up trajectory objects
        for (auto* obj : trajectory_objects) {
            delete obj;
        }

        return min_distance;
    }

    void CollisionChecker::setTriangles(const std::vector<Triangle2D> &triangles) {
        // DEBUG("Setting triangles in CollisionChecker " << triangles.size() << " triangles");

        // Clear existing objects from manager
        _manager->clear();

        // Clean up existing collision objects
        for (auto* obj : _obs_list) {
            delete obj;
        }
        _obs_list.clear();

        // Reserve space for efficiency
        _obs_list.reserve(triangles.size());

        for (const auto& triangle : triangles) {
            // Create a BVH model for the triangle mesh
            auto geom = std::make_shared<fcl::BVHModel<fcl::OBBRSS<float>>>();

            // Begin model construction
            geom->beginModel();

            // Use triangles in their original coordinates (no centering needed)
            fcl::Vector3<float> v0(triangle.vertices[0][0], triangle.vertices[0][1], 0.0f);
            fcl::Vector3<float> v1(triangle.vertices[1][0], triangle.vertices[1][1], 0.0f);
            fcl::Vector3<float> v2(triangle.vertices[2][0], triangle.vertices[2][1], 0.0f);

            // Add the triangle using the vertex positions
            geom->addTriangle(v0, v1, v2);

            // End model construction
            geom->endModel();

            // Create identity transform (no translation needed since we use original coordinates)
            fcl::Transform3<float> pose = fcl::Transform3<float>::Identity();

            // Create collision object and add to list
            _obs_list.emplace_back(new fcl::CollisionObject<float>(geom, pose));
        }

        // Register objects with the collision manager
        if (_obs_list.empty()) {
            // DEBUG("Warning: No triangles provided to CollisionChecker.");
        } else {
            _manager->registerObjects(_obs_list);
            _manager->setup();
            // DEBUG("Registered " + std::to_string(triangles.size()) + " triangles with collision manager.");
        }
    }

    // Add destructor to clean up dynamically allocated objects
    CollisionChecker::~CollisionChecker() {
        for (auto* obj : _obs_list) {
            delete obj;
        }
    }
}