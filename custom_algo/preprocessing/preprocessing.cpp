#include "preprocessing.h"
#include <chrono>

void Preprocessing::initialize(int preprocess_time_limit) {
    env->rng.seed(0);
    srand(0);
 
    // Fase 0 : Inisialisasi neighbor
    std::vector<int> centroids;
    CustomAlgo::init_neighbor(env);
 
    // Fase 1a : maximin_sampling
    centroids = CustomAlgo::maximin_sampling(env);
 
    // Fase 1b : voronoi_generation
    CustomAlgo::voronoi_generation(env, centroids);
 
    // Fase 2+3 : generate_HPAHMap (logged internally)
    CustomAlgo::generate_HPAHMap(env, centroids);
 
    // Fase 4 : degree_map
    std::vector<int> degree_map;
    degree_map.resize(env->map.size());
    for (int v = 0; v < (int)env->map.size(); v++) {
        degree_map[v] = CustomAlgo::degreeNeighbors(env, v);
    }
    env->degree_map = degree_map;

    // Fase 5 : generateOffset
    std::vector<int> offsets = CustomAlgo::generateOffset(env->r, env->cols);
    env->offsets = offsets;
 
}