#pragma once

#include "Types.h"
#include "mapf_utils.h"
#include <queue>

namespace CustomAlgo{


    void init_neighbor(SharedEnvironment* env);

    // Modifikasi
    int query_heuristic(SharedEnvironment* env , int src , int src_orient, int dest);
}


