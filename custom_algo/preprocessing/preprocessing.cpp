#include "preprocessing.h"
#include <chrono>

#define LOG_TIME(label, block) \
    { \
        auto _t0 = std::chrono::steady_clock::now(); \
        block \
        auto _t1 = std::chrono::steady_clock::now(); \
        double _ms = std::chrono::duration<double, std::milli>(_t1 - _t0).count(); \
        std::cout << "[PREPROCESS] " << label << " : " << _ms << " ms" << std::endl; \
    }

void Preprocessing::initialize(int preprocess_time_limit) {
     auto total_start = std::chrono::steady_clock::now();
    std::cout << "[PREPROCESS] ========== initialize() START ==========" << std::endl;
    std::cout << "[PREPROCESS] Map size       : " << env->map.size() << std::endl;
    std::cout << "[PREPROCESS] Map cols       : " << env->cols << std::endl;
    std::cout << "[PREPROCESS] Map rows       : " << env->rows << std::endl;
    std::cout << "[PREPROCESS] Time limit     : " << preprocess_time_limit << " ms" << std::endl;
 
    env->rng.seed(0);
    srand(0);
 
    // Fase 0 : Inisialisasi neighbor
    std::vector<int> centroids;
    LOG_TIME("Fase 0 | init_neighbor",
        CustomAlgo::init_neighbor(env);
    )
 
    // Fase 1a : maximin_sampling
    LOG_TIME("Fase 1a | maximin_sampling",
        centroids = CustomAlgo::maximin_sampling(env);
    )
    std::cout << "[PREPROCESS] Fase 1a | centroids count : " << centroids.size() << std::endl;
 
    // Fase 1b : voronoi_generation
    LOG_TIME("Fase 1b | voronoi_generation",
        CustomAlgo::voronoi_generation(env, centroids);
    )
 
    // Fase 2+3 : generate_HPAHMap (logged internally)
    LOG_TIME("Fase 2+3 | generate_HPAHMap (total)",
        CustomAlgo::generate_HPAHMap(env, centroids);
    )
 
    // Fase 4 : degree_map
    LOG_TIME("Fase 4 | degree_map",
        std::vector<int> degree_map;
        degree_map.resize(env->map.size());
        for (int v = 0; v < (int)env->map.size(); v++) {
            degree_map[v] = CustomAlgo::degreeNeighbors(env, v);
        }
        env->degree_map = degree_map;
    )
 
    // Fase 5 : generateOffset
    LOG_TIME("Fase 5 | generateOffset",
        std::vector<int> offsets = CustomAlgo::generateOffset(env->r, env->cols);
        env->offsets = offsets;
    )
 
    auto total_end = std::chrono::steady_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(total_end - total_start).count();
    std::cout << "[PREPROCESS] ========== initialize() END | TOTAL : "
              << total_ms << " ms / " << preprocess_time_limit << " ms limit ==========" << std::endl;
 
    if (total_ms > preprocess_time_limit) {
        std::cerr << "[PREPROCESS] WARNING: Preprocessing EXCEEDED time limit by "
                  << (total_ms - preprocess_time_limit) << " ms!" << std::endl;
    }
}