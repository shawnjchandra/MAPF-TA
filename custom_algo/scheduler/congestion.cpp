#include "congestion.h"

namespace CustomAlgo{

    std::vector<int> calc_square_density(SharedEnvironment* env) {
        env->square_density.resize(env->k);
        env->square_density.assign(env->k, 0);
        for (const auto& state : env->curr_states){
            int c_id = env->hpa_h.voronoi_map[state.location];
            env->square_density[c_id]++;
        }
    }

    double task_square_density(int task_loc, int dist, SharedEnvironment* env) {
        int target_c_id = env->hpa_h.voronoi_map[task_loc];

        double rho = (double)env->square_density[target_c_id] / (double) env->hpa_h.local_to_global[target_c_id].size();

        return rho * dist;
    }
}