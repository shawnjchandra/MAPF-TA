#include "preprocessing.h"

void Preprocessing::initialize(int preprocess_time_limit){
    std::vector<int> degree_map;
    degree_map.resize(env->map.size());

    //Fase 1 : Subgraf + voronoi 
    std::vector<int> centroids = CustomAlgo::maximin_sampling(env);
    CustomAlgo::voronoi_generation(env);

    // Fase 2+3 : heuristic map + highway;
    CustomAlgo::generate_HPAHMap(env, centroids);
    
    // Fase 4 : Generate degree map
    for (int v = 0 ; v < env->map.size() ; v++) {
        degree_map[v] = CustomAlgo::degreeNeighbors(env,v);
    }

    // Fase 5 : bikin offsets
    std::vector<int> offsets = CustomAlgo::generateOffset(env->r, env->cols);

    // Save degree sama offsets
    env->degree_map = degree_map;
    env->offsets = offsets;
}