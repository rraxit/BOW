#!/usr/bin/env bash

# get env name
env=$1

# navigate to the build directory and run the test
current_dir=$(pwd)
echo "Running tests... $env"
cd build
./BowPlannerRAL ../test/$env

cd $current_dir/scripts
# run the trajectory viewer
python viz_trajectory.py ../test/$env ../build/trajectory.csv
