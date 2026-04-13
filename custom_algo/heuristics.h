#pragma once

#include "Types.h"
#include "mapf_utils.h"
#include <queue>
#include "search_node.h"

namespace CustomAlgo{

    //     std::mutex intra_ht_mutex;
    // std::mutex inter_cache_mutex;
void init_heuristics(SharedEnvironment* env);

void init_neighbor(SharedEnvironment* env);

// void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

// int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);

int get_h(SharedEnvironment* env, int source, int target);

    // Modifikasi
    int query_heuristic(SharedEnvironment* env , int src , int src_orient, int dest);
}


