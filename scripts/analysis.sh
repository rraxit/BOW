#!/usr/bin/env bash

for i in 1 2 3 4 5
do
  echo "Running ../test/env$i.yaml"
  python obstacle_density.py   --config "../test/env$i.yaml" --plot
done