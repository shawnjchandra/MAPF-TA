#include "preprocessing.h"
#include <chrono>

void Preprocessing::initialize(int preprocess_time_limit){
    auto _t0 = std::chrono::high_resolution_clock::now();

    std::vector<int> degree_map;
    degree_map.resize(env->map.size());

    //Fase 1 : Subgraf + voronoi 
    std::vector<int> centroids = CustomAlgo::maximin_sampling(env);
    CustomAlgo::voronoi_generation(env,centroids);
    
    auto _t1 = std::chrono::high_resolution_clock::now();
    long voronoi_ms = std::chrono::duration_cast<std::chrono::milliseconds>(_t1 - _t0).count();
    
    // Fase 2+3 : heuristic map + highway;
    CustomAlgo::generate_HPAHMap(env, centroids);
    
    auto _t2 = std::chrono::high_resolution_clock::now();
    long hpa_ms = std::chrono::duration_cast<std::chrono::milliseconds>(_t2 - _t1).count();

    // Fase 4 : Generate degree map
    for (int v = 0 ; v < env->map.size() ; v++) {
        degree_map[v] = CustomAlgo::degreeNeighbors(env,v);
    }

    // Fase 5 : bikin offsets
    std::vector<int> offsets = CustomAlgo::generateOffset(env->r, env->cols);

    // Save degree sama offsets
    env->degree_map = degree_map;
    env->offsets = offsets;

    auto _t3 = std::chrono::high_resolution_clock::now();
    long total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(_t3 - _t0).count();

    std::cout << "[PREPROCESS] total_ms="      << total_ms                          << std::endl;
    std::cout << "[PREPROCESS] voronoi_ms="    << voronoi_ms                        << std::endl;
    std::cout << "[PREPROCESS] hpa_ms="        << hpa_ms                            << std::endl;
    std::cout << "[PREPROCESS] clusters="      << env->k                            << std::endl;
    std::cout << "[PREPROCESS] gates="         << env->hpa_h.AG.gates.size()        << std::endl;
    std::cout << "[PREPROCESS] entrances="     << env->hpa_h.Ents.size()            << std::endl;
    std::cout << "[PREPROCESS] highway_edges=" << env->hpa_h.hw.e_hw.size()         << std::endl;
    std::cout << "[PREPROCESS] n_cells="       << env->map.size()                   << std::endl;


}