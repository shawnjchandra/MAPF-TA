#pragma once

#include "Types.h"
#include "heuristics.h"
#include "mapf_utils.h"
#include <utility>
#include <set>

namespace CustomAlgo{

    void generateHighways(SharedEnvironment* env,  std::vector<int> centroids);

    std::vector<std::pair<int,int>> extractPathFromH_HW(int start, int goal, SharedEnvironment* env, std::vector<int> prev,  std::vector<bool> visited);

    void reverseHighways(SharedEnvironment* env);
}