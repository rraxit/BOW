include(FetchContent)
# --------------------------
# Google Benchmark
# --------------------------
FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG main  # Or a stable release like v1.8.3
)

# Google Benchmark has a dependency on GoogleTest
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "Disable benchmark tests" FORCE)
set(BENCHMARK_DOWNLOAD_DEPENDENCIES ON)

FetchContent_MakeAvailable(benchmark)

# --------------------------
# HRVO
# --------------------------
FetchContent_Declare(
        hrvo
        GIT_REPOSITORY https://github.com/snape/HRVO.git
        GIT_TAG main  # Or pin to a commit or release
)

FetchContent_MakeAvailable(hrvo)

include_directories(
        include
        #add OMPL
        /opt/local/include
        /usr/local/include/ompl-1.6
        /usr/include/ompl-1.5
)

set(BENCHMARK_PLANNERS_SRC
        include/dwa/DWA.h
        include/ompl/pch.h
        include/ompl/MotionPlanner.h
        src/DWA.cpp
        src/MotionPlanner.cpp
)

link_directories(
        ${CMAKE_BINARY_DIR}
        "/usr/local/lib"
)
add_executable(BowBenchmark benchmark.cpp
        ${BENCHMARK_PLANNERS_SRC}
)

add_dependencies(BowBenchmark bow HRVO)

target_link_libraries(BowBenchmark PRIVATE
        -lbow
        -lompl
        yaml-cpp::yaml-cpp
        Eigen3::Eigen
        benchmark::benchmark
        HRVO
)