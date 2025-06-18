cmake_minimum_required(VERSION 3.27)
project(BowPlannerRAL)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()

option(BUILD_BENCHMARK "Google benchmark for evaluating BOW." ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find required packages
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(FCL REQUIRED)
find_package(CCD REQUIRED)

include_directories(
    include
    include/limbo
    ${EIGEN3_INCLUDE_DIR}
)

set(SRC_FILES
        include/bow/bow_param.h
        include/bow/BOW.h
        include/bow/CollisionChecker.h
        src/BOW.cpp
        src/CollisionChecker.cpp
        src/ParamManager.cpp
        include/bow/ParamManager.h
)

add_library(limbo INTERFACE)
add_library(bow SHARED ${SRC_FILES})
target_precompile_headers(bow PUBLIC include/limbo/limbo/limbo.hpp)
target_link_libraries(bow PUBLIC
        Boost::program_options
        Eigen3::Eigen
        -lnlopt_cxx
        yaml-cpp::yaml-cpp
        ${FCL_LIBRARIES}
        ${CCD_LIBRARIES}
        limbo
)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE bow)

# enable benchmark
if(BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif()
