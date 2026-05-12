#!/bin/bash

# Exit immediately if a command fails
set -e

echo "--- Cleaning old build directory ---"
rm -rf build/

echo "--- Configuring project ---"
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release

echo "--- Compiling ---"
make -C build -j$(nproc)

# time ./build/lifelong --inputFile  example_problems/random.domain/random_32_32_20_100.json -o ../output/out-wppl-lns.json --simulationTime 100 --planTimeLimit 600000 --preprocessTimeLimit 300000 --numberOfCluster 10 --radius 3 --c 2 --mode wppl --w 10
# time ./build/lifelong --inputFile MR24/city.domain/CITY-01.json -o ../output/mr24/city/1/wppl-w10.json --simulationTime 100 --planTimeLimit 600000 --preprocessTimeLimit 1800000 --numberOfCluster 400 --radius 3 --c 2 --mode wppl --w 10 --m 8


