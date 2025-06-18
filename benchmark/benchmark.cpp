#include <benchmark/benchmark.h>
#include "bow/BOW.h"
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <algorithm>
#include <numeric>
#include "hrvo/HRVO.h"
#include "ompl/MotionPlanner.h"
#include "dwa/DWA.h"
#include <mutex>

std::mutex mu;

// Templated path length computation with potential performance improvements
template <typename T>
[[nodiscard]] double compute_path_length(const std::vector<T>& traj) noexcept {
    if (traj.size() < 2) return 0.0;

    // Use std::transform and std::adjacent_difference for potentially better optimization
    double path_length = 0.0;
    for (size_t i = 0; i < traj.size() - 1; ++i) {
        if (traj[i].size() < 2 || traj[i + 1].size() < 2) {
            continue; // Skip if the trajectory points are invalid
        }
        const double dx = traj[i][0] - traj[i + 1][0];
        const double dy = traj[i][1] - traj[i + 1][1];
        path_length += std::hypot(dx, dy);  // More numerically stable than sqrt()
    }
    return path_length;
}

template <typename T>
[[nodiscard]] double compute_avg_velocity(const std::vector<T>& traj) noexcept {
    if (traj.size() < 2) return 0.0;

    // Use std::transform and std::adjacent_difference for potentially better optimization
    double total_velocity = 0.0;
    for (size_t i = 0; i < traj.size(); ++i) {
      total_velocity += std::hypot(traj[i][3], traj[i][4]); // Assuming traj[i][3] is vx and traj[i][4] is vy
    }
    return total_velocity / traj.size();
}

template <typename T>
[[nodiscard]] double compute_avg_jerk(const std::vector<T>& traj) noexcept {
    if (traj.size() < 3) return 0.0; // Need at least 3 points for 2 acceleration steps

    double total_jerk = 0.0;
    int valid_count = 0;

    for (size_t i = 0; i < traj.size() - 2; ++i) {
        if (traj[i].size() < 5 || traj[i+1].size() < 5 || traj[i+2].size() < 5)
            continue;

        // Backward-difference accelerations
        const double ax_i   = traj[i][3]     - traj[i+1][3];
        const double ay_i   = traj[i][4]     - traj[i+1][4];

        const double ax_ip1 = traj[i+1][3]   - traj[i+2][3];
        const double ay_ip1 = traj[i+1][4]   - traj[i+2][4];

        // Jerk = Δacceleration
        const double jx = ax_i - ax_ip1;
        const double jy = ay_i - ay_ip1;

        total_jerk += std::hypot(jx, jy);
        ++valid_count;
    }

    return valid_count > 0 ? total_jerk / valid_count : 0.0;
}


// Define benchmark function for RRT
static void BM_RRT_Planner(benchmark::State& state, const std::string& config_path) {
    auto pm = std::make_shared<param_manager>(config_path);
    // Pre-load configuration data
    std::vector<std::vector<float>> obsList;
    pm->get_obstacles(obsList);
    float robotRadius = pm->get_param<float>("robot_radius");
    float goalRadius = pm->get_param<float>("goal_radius");
    float obsLen = pm->get_param<float>("obstacle_length");
    auto cc = std::make_shared<bow::CollisionChecker>(obsList, robotRadius, obsLen);

    auto start = pm->get_param<std::vector<float>>("start");
    auto goal = pm->get_param<std::vector<float>>("goal");

    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    std::vector<double> boundary;

    for (const auto& s : start) {
        boundary.push_back(s);
    }
    for (const auto& g : goal) {
        boundary.push_back(g);
    }

    auto lower = *std::min_element(boundary.begin(), boundary.end()) - 1.0;
    auto upper = *std::max_element(boundary.begin(), boundary.end()) + 1.0;
    for (auto _ : state) {
        ompl::MotionPlanner planner(cc, pm, lower, upper, false);
        auto traj = planner.plan(start, goal);
        state.counters["path_length"] = compute_path_length(traj);
        state.counters["num_nodes"] = traj.size();
        state.counters["avg_velocity"] = compute_avg_velocity(traj);
        state.counters["avg_jerk"] = compute_avg_jerk(traj);
    }

}

// Define benchmark function for HRVO
static void BM_HRVO_Planner(benchmark::State& state, const std::string& config_path)
{
    auto pm = std::make_shared<param_manager>(config_path);
    // Pre-load configuration data
    std::vector<std::vector<float>> obsList;
    pm->get_obstacles(obsList);
    float robotRadius = pm->get_param<float>("robot_radius");
    float goalRadius = pm->get_param<float>("goal_radius");
    float obsLen = pm->get_param<float>("obstacle_length");
    auto cc = std::make_shared<bow::CollisionChecker>(obsList, robotRadius, obsLen);

    auto start = pm->get_param<std::vector<double>>("start");
    auto goal = pm->get_param<std::vector<double>>("goal");
    float dt = pm->get_param<float>("dt");



    // Benchmark loop
    for (auto _ : state) {
        hrvo::Simulator simulator;
        simulator.setTimeStep(dt);

        simulator.setAgentDefaults(float(obsList.size() + 1), obsList.size() + 1,  (2 * obsLen), goalRadius, 0.50F, 1.0F);

        const hrvo::Vector2 start_position = hrvo::Vector2(start[0], start[1]);
        const hrvo::Vector2 goal_position = hrvo::Vector2(goal[0], goal[1]);
        simulator.addAgent(start_position, simulator.addGoal(goal_position));

        // add obstacles
        for (const auto& ob : obsList) {
            simulator.addAgent(hrvo::Vector2(ob[0], ob[1]), simulator.addGoal(hrvo::Vector2(ob[0], ob[1])));
        }
        // Only execution code should be inside the loop
        std::vector<std::array<double, 5>> traj;
        do {
            auto pos = simulator.getAgentPosition(0);
            auto vel = simulator.getAgentVelocity(0);
            traj.push_back(std::array<double, 5>{pos.getX(), pos.getY(), 0, vel.getX(), vel.getY()});
            simulator.doStep();
        } while (!simulator.haveReachedGoals());
        state.counters["path_length"] = compute_path_length(traj);
        state.counters["num_nodes"] = traj.size();
        state.counters["avg_velocity"] = compute_avg_velocity(traj);
        state.counters["avg_jerk"] = compute_avg_jerk(traj);
    }
}

// Define benchmark function for bow
static void BM_BOW_Planner(benchmark::State& state, const std::string& config_path) {
    // Create resources outside the benchmark loop
    auto pm = std::make_shared<param_manager>(config_path);

    // Pre-load configuration data
    std::vector<std::vector<float>> obsList;
    pm->get_obstacles(obsList);
    float robotRadius = pm->get_param<float>("robot_radius");
    float obsLen = pm->get_param<float>("obstacle_length");
    auto cc = std::make_shared<bow::CollisionChecker>(obsList, robotRadius, obsLen);

    auto start = pm->get_param<std::vector<double>>("start");
    auto goal = pm->get_param<std::vector<double>>("goal");


    // Benchmark loop
    for (auto _ : state) {
        // Only execution code should be inside the loop
        bow::State s0;
        s0 << start[0], start[1], start[2], 0.0, 0.0;
        bow::Point g;
        g << goal[0], goal[1];
        bow::BOPlanner planner(s0, g, cc->getSharedPtr(), pm->getSharedPtr());
        auto [sol, traj] = planner.solve(1.0, false);
        state.counters["path_length"] = compute_path_length(traj);
        state.counters["num_nodes"] = traj.size();
        state.counters["avg_velocity"] = compute_avg_velocity(traj);
        state.counters["avg_jerk"] = compute_avg_jerk(traj);
    }
}

// Define benchmark function for DWA

static void BM_DWA_Planner(benchmark::State& state, const std::string& config_path) {
    // Create resources outside the benchmark loop
    auto pm = std::make_shared<param_manager>(config_path);

    // Pre-load configuration data
    std::vector<std::vector<float>> obsList;
    pm->get_obstacles(obsList);
    float robotRadius = pm->get_param<float>("robot_radius");
    float obsLen = pm->get_param<float>("obstacle_length");

    auto start = pm->get_param<std::vector<double>>("start");
    auto goal = pm->get_param<std::vector<double>>("goal");

    // Benchmark loop
    for (auto _ : state) {
        auto cc = std::make_shared<bow::CollisionChecker>(obsList, robotRadius, obsLen);
        dwa::State s0{start[0], start[1], start[2], 0.0, 0.0};
        dwa::Point g{goal[0], goal[1]};
        // Only execution code should be inside the loop
        dwa::DynamicWindowApproach planner(s0, g, cc->getSharedPtr(), pm->getSharedPtr());
        auto [sol, traj] = planner.solve(5.0, false);
        state.counters["path_length"] = compute_path_length(traj);
        state.counters["num_nodes"] = traj.size();
        state.counters["avg_velocity"] = compute_avg_velocity(traj);
        state.counters["avg_jerk"] = compute_avg_jerk(traj);
    }
}


// Efficient benchmark registration
void RegisterBenchmarks() {
    std::ifstream file("../../test/env_list.txt");
    std::vector<std::string> env_paths;

    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            env_paths.push_back(std::move(line));
        }
    }
    auto benchmark_types = {
            std::make_pair("DWA", BM_DWA_Planner),
            std::make_pair("RRT", BM_RRT_Planner),
            std::make_pair("BOW", BM_BOW_Planner),
            std::make_pair("HRVO", BM_HRVO_Planner),
    };

    for (size_t i = 0; i < env_paths.size(); ++i) {
        for (const auto& [type, benchmark_func] : benchmark_types) {
            std::string name =  std::string(type); + "_Planner/ENV" + std::to_string(i + 1);
            benchmark::RegisterBenchmark(
                    name.c_str(),
                    [benchmark_func, path = env_paths[i]](benchmark::State& state) {
                        benchmark_func(state, path);
                    })
                    ->Arg(i + 1)
                    ->Unit(benchmark::kMillisecond);
        }
    }
}

int main(int argc, char** argv) {
    RegisterBenchmarks();
    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}