#include "preprocessing.h"
#include <chrono>

void Preprocessing::initialize(int preprocess_time_limit) {
    // Fase 0 :Inisialisasi neighbor (Untuk getNeighborLocs(...))
    CustomAlgo::init_neighbor(env);

    // Fase 1 : Cari centroid dan bikin subgraf / clustering
    std::vector<int> centroids = CustomAlgo::maximin_sampling(env);
    CustomAlgo::voronoi_generation(env, centroids);

    // Fase 2+3 : Buat peta berbasis Hierarichal Pathfinding A* untuk heuristik, kemudian modifikasi dengan highway
    CustomAlgo::generate_HPAHMap(env, centroids);

    // Fase 4 : Buat degree dan offset untuk keperluan scheduling dan planning
    std::vector<int> degree_map;
    degree_map.resize(env->map.size());
    for (int v = 0; v < (int)env->map.size(); v++) {
        degree_map[v] = CustomAlgo::degreeNeighbors(env, v);
    }

    std::vector<int> offsets = CustomAlgo::generateOffset(env->r, env->cols);

    env->degree_map = degree_map;
    env->offsets    = offsets;
}