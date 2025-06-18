#include <iostream>
#include <unordered_map>
#include <chrono>
#include "bow/BOW.h"

template <typename T>
void save_trajectory(const std::vector<T>& traj)
{
    // save trajectory in a csv file
    std::ofstream file("trajectory.csv");
    if (file.is_open()) {
        for (const auto& state : traj) {
            file << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << state(4) << "\n";
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}

// Templated path length computation with potential performance improvements
template <typename T>
[[nodiscard]] double compute_path_length(const std::vector<T>& traj) noexcept {
    if (traj.size() < 2) return 0.0;

    // Use std::transform and std::adjacent_difference for potentially better optimization
    double path_length = 0.0;
    for (size_t i = 0; i < traj.size() - 1; ++i) {
        const double dx = traj[i][0] - traj[i + 1][0];
        const double dy = traj[i][1] - traj[i + 1][1];
        path_length += std::hypot(dx, dy);  // More numerically stable than sqrt()
    }
    return path_length;
}

std::unordered_map<std::string, double> solve(const std::string& config_path) {
    auto pm = std::make_shared<param_manager>(config_path);
    // Pre-load configuration data
    std::vector<std::vector<float>> obsList;
    pm->get_obstacles(obsList);



    float robotRadius = pm->get_param<float>("robot_radius");
    float obsLen = pm->get_param<float>("obstacle_length");
    auto cc = std::make_shared<bow::CollisionChecker>(obsList, robotRadius, obsLen);

    if(obsList.empty()) {
        auto triangles = pm->get_triangles();
        cc->setTriangles(triangles);
    }

    auto start = pm->get_param<std::vector<double>>("start");
    auto goal = pm->get_param<std::vector<double>>("goal");

    bow::State s0;
    s0 << start[0], start[1], start[2], 0.0, 0.0;
    bow::Point g;
    g << goal[0], goal[1];

    auto start_time = std::chrono::high_resolution_clock::now();
    bow::BOPlanner planner(s0, g, cc->getSharedPtr(), pm->getSharedPtr());
    auto [sol, traj] = planner.solve(1.0, false);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    save_trajectory(traj);

    return {{"path_length", compute_path_length(traj)}, {"num_nodes", traj.size()}, {"elapsed_time", elapsed.count()}};
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_path>" << std::endl;
        return 1;
    }
    std::string config_path = argv[1];
    auto results = solve(config_path);
    std::cout << "Path Length: " << results["path_length"] << std::endl;
    std::cout << "Number of Nodes: " << results["num_nodes"] << std::endl;
    std::cout << "Elapsed Time: " << results["elapsed_time"] << " seconds" << std::endl;

    return 0;
}
