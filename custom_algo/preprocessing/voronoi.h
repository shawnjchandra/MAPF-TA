#pragma once

#include "Types.h"
#include "SharedEnv.h"
// #include "mapf_utils.h"
#include <queue>

namespace CustomAlgo{
    std::vector<int> maximin_sampling(SharedEnvironment* env);
    
    void voronoi_generation(SharedEnvironment* env, std::vector<int> centroids);
}
