#include "preprocessing.h"
#include <chrono>

void Preprocessing::initialize(int preprocess_time_limit) {
    std::cerr << "INIT CALLED" << std::endl;
    auto t_total = std::chrono::high_resolution_clock::now();
    auto elapsed = [&](auto t) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - t).count();
    };

    auto t0 = std::chrono::high_resolution_clock::now();
    CustomAlgo::init_neighbor(env);
    std::cerr << "[PRE] Step 0 init_neighbor:        " << elapsed(t0) << "ms" << std::endl;


    std::vector<int> centroids = CustomAlgo::maximin_sampling(env);
    std::cerr << "[PRE] maximin_sampling:   " << elapsed(t0) << "ms"
              << "  centroids=" << centroids.size() << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    CustomAlgo::voronoi_generation(env, centroids);
    std::cerr << "[PRE] voronoi_generation: " << elapsed(t0) << "ms" << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    CustomAlgo::generate_HPAHMap(env, centroids);
    std::cerr << "[PRE] generate_HPAHMap:   " << elapsed(t0) << "ms" << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    std::vector<int> degree_map;
    degree_map.resize(env->map.size());
    for (int v = 0; v < (int)env->map.size(); v++)
        degree_map[v] = CustomAlgo::degreeNeighbors(env, v);
    std::cerr << "[PRE] degree_map:         " << elapsed(t0) << "ms" << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    std::vector<int> offsets = CustomAlgo::generateOffset(env->r, env->cols);
    std::cerr << "[PRE] generateOffset:     " << elapsed(t0) << "ms" << std::endl;

    env->degree_map = degree_map;
    env->offsets    = offsets;

    std::cerr << "[PRE] TOTAL initialize:   " << elapsed(t_total) << "ms" << std::endl;
}