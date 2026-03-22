#include "preprocessing.h"
#include <chrono>

void Preprocessing::initialize(int preprocess_time_limit) {
    CustomAlgo::init_neighbor(env);

    std::vector<int> centroids = CustomAlgo::maximin_sampling(env);

    CustomAlgo::voronoi_generation(env, centroids);

    CustomAlgo::generate_HPAHMap(env, centroids);

    std::vector<int> degree_map;
    degree_map.resize(env->map.size());
    for (int v = 0; v < (int)env->map.size(); v++)
        degree_map[v] = CustomAlgo::degreeNeighbors(env, v);

    std::vector<int> offsets = CustomAlgo::generateOffset(env->r, env->cols);

    env->degree_map = degree_map;
    env->offsets    = offsets;
}