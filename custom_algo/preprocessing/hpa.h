#pragma once

#include <queue>
#include <set>
#include "Types.h"
#include "utils.h"
#include "highway.h"
#include "SharedEnv.h"

namespace CustomAlgo{
    
    

    void cluster_indexing(SharedEnvironment* env);
    
    void build_entrances(SharedEnvironment* env);

    std::vector<std::array<int, 4>> build_IntraHT(SharedEnvironment* env, int cluster, int dest);  
    
    void build_abstract_graph(SharedEnvironment* env);
    
    void build_InterHT(SharedEnvironment* env );
    
    void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids);  

}