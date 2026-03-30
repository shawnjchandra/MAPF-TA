#pragma once
#include <vector>
#include "SharedEnv.h"

namespace CustomAlgo {
    
    std::vector<int> calc_square_density(SharedEnvironment* env);

    double task_square_density( int task_loc, int dist,SharedEnvironment* env);
}