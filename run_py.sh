#!/usr/bin/env bash
declare -a ENVS=("env1_f" "env2_f" "env3_f" "env4_f" "env5_f" "env6_f")
declare -a METHODS=("mppi" "cbf")

PYTHON="venv/bin/python"

for seed in {1..10}; do
  for env in ${ENVS[@]}
  do
    for planner in ${METHODS[@]}; do
      echo $planner $seed $env
      mkdir -p "./results/benchmark/$seed/$env/$planner"
      time python main.py seed=$seed test=$env planner=$planner > "./results/benchmark/$seed/$env/$planner/verbose.txt"
    done
  done
done


