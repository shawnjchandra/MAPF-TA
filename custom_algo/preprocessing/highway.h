#pragma once

#include "Types.h"
#include "heuristics.h"
#include "utils.h"
#include <utility>

namespace CustomAlgo{

    void generateHighways(SharedEnvironment* env,  std::vector<int> centroids);

    std::vector<std::pair<int,int>>  extractPathFromH_HW(int start,int goal,SharedEnvironment* env);

    void reverseHighways(SharedEnvironment* env);
}