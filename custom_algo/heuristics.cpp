#include "heuristics.h"
#include <queue>

namespace CustomAlgo{
    Neighbors global_neighbors;

    void init_neighbor(SharedEnvironment* env){
        global_neighbors.resize(env->rows*env->cols);
        for(int row = 0 ; row < env->rows ; row++) {
            for(int col = 0 ; col < env->cols ; col++) {
                int loc = row*env->cols+col;

                if(env->map[loc] == 0) {
                    if(row>0 && env->map[loc-env->cols]==0){
                        global_neighbors[loc].push_back(loc-env->cols);
                    }
                    if(row<env->rows-1 && env->map[loc+env->cols]==0){
                        global_neighbors[loc].push_back(loc+env->cols);
                    }
                    if(col>0 && env->map[loc-1]==0){
                        global_neighbors[loc].push_back(loc-1);
                    }
                    if(col<env->cols-1 && env->map[loc+1]==0){
                        global_neighbors[loc].push_back(loc+1);
                    }
                } 
            }
        }
        env->ns = global_neighbors;
    }

     int query_heuristic(SharedEnvironment* env , int src , int src_orient, int dest) {
        int c_src = env->hpa_h.voronoi_map[src];
        int c_dest = env->hpa_h.voronoi_map[dest];

        int src_local = env->hpa_h.global_to_local[src];
        int dest_local = env->hpa_h.global_to_local[dest];

        if (c_src == c_dest) return env->hpa_h.IntraHT[c_src][dest_local][src_local][src_orient];

        int best = INTERVAL_MAX;

        int cost_gd_to_dest;
        for(int g_s : env->hpa_h.Gates[c_src]) {
            int g_s_local = env->hpa_h.global_to_local[g_s];
            int g_s_idx = env->hpa_h.gate_index[g_s];

            int cost_src_to_gs = env->hpa_h.IntraHT[c_src][g_s_local][src_local][src_orient];

            if (cost_src_to_gs == INTERVAL_MAX) continue;   

            for(int g_d : env->hpa_h.Gates[c_dest]) {
                int g_d_local = env->hpa_h.global_to_local[g_d];
                int g_d_idx = env->hpa_h.gate_index[g_d];
                int cost_inter = env->hpa_h.InterHT[g_s_idx][g_d_idx];

                if (cost_inter == INTERVAL_MAX) continue;

                cost_gd_to_dest = INTERVAL_MAX;
                for (int orient = 0 ; orient < 4 ; orient++) {
                    cost_gd_to_dest = min(cost_gd_to_dest,env->hpa_h.IntraHT[c_dest][dest_local][src_local][orient]);
                }
                if (cost_gd_to_dest == INTERVAL_MAX) continue;

                int total = cost_src_to_gs + cost_inter + cost_gd_to_dest;
                if (total < best ) best = total;
            }
        }

        return best;
     }
    
}