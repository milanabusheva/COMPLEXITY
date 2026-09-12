#!/bin/bash

g++ -std=c++17 -O2 "$1" -o "${1%.cpp}"

for test in tsp_51_1 tsp_100_3 tsp_200_2 tsp_574_1 tsp_1889_1 tsp_33810_1; do
    echo -e "\033[1;32m$test\033[0m"
    ./"${1%.cpp}" "$test"
    echo
done