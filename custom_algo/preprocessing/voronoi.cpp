#include "voronoi.h"

namespace CustomAlgo{

    std::vector<int> maximin_sampling(SharedEnvironment* env) {
        std::vector<bool> selected;
        selected.resize(env->map.size());
        
        std::vector<int> interest_points;
        interest_points.resize(env->k);

        int p;
        do {
            p = CustomAlgo::rng(0, env->map.size() - 1);
        } while (env->map[p] == 1);

        interest_points.push_back(p);
        selected[p] = true;

        for (int i = 0; i < env->k-1; i++) {
            int max_min_dist = -1;
            int selected_idx = -1;
            
            for(int j = 0 ; j < env->map.size() ; j ++) {
                if (!selected[j] || env->map[j] == 1) {
                    int min_dist = INTERVAL_MAX;

                    for (p = 0 ; p < interest_points.size() ; p++) {
                        int dist = CustomAlgo::manhattanDistance(j, interest_points[p], env);
                        if(dist < min_dist) min_dist = dist;
                    }

                    if (min_dist > max_min_dist) {
                        max_min_dist = min_dist;
                        selected_idx = j;
                    }
                }
            }

            if (selected_idx != -1) {
                interest_points.push_back(selected_idx);
                selected[selected_idx] = true;
            }
        }

        return interest_points;
    }
    
    void voronoi_generation(SharedEnvironment* env, std::vector<int> centroids) {
        env->hpa_h.voronoi_map.resize(env->map.size(), -1);

        for (int row = 0 ; row < env->rows; row++) {
            for (int col = 0 ; col < env->cols ; col++) {
                
                if (env->map[row * env->cols + col] == 0) {
                    int min_dist = INTERVAL_MAX;;
                    int label = -1;

                    for (int k = 0; k < centroids.size(); k++) {
                        int dist = CustomAlgo::manhattanDistance(row * env->cols + col, centroids[k], env);

                        if (dist < min_dist) {
                            min_dist = dist;
                            label = k;
                        }
                    }
                    env->hpa_h.voronoi_map[row * env->cols + col] = label;
                }
            }
        }
    }
}