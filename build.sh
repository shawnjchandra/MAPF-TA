#!/bin/bash

# Exit immediately if a command fails
set -e

echo "--- Cleaning old build directory ---"
rm -rf build/

echo "--- Configuring project ---"
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release

echo "--- Compiling ---"
make -C build -j$(nproc)

time ./build/lifelong --inputFile  example_problems/random.domain/random_32_32_20_100.json -o ../output/out2.json --simulationTime 200 --planTimeLimit 10000 --preprocessTimeLimit 300000 --numberOfCluster 10 --radius 3 --cPenalty 2

