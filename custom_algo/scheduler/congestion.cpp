#include "congestion.h"

namespace CustomAlgo{

    void calc_square_density(SharedEnvironment* env) {

        env->square_density.assign(env->k, 0);
        for (const auto& state : env->curr_states){
            int c_id = env->hpa_h.voronoi_map[state.location];
            env->square_density[c_id]++;
        }
    }

    double task_square_density(int task_loc, int dist, SharedEnvironment* env) {
        int target_c_id = env->hpa_h.voronoi_map[task_loc];

        int cluster_size = env->hpa_h.local_to_global[target_c_id].size();

        double rho = (double)env->square_density[target_c_id] / (double) cluster_size;

        return rho * dist;
    }
}