#include "heuristics.h"
#include <queue>
#include "preprocessing/hpa.h"

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
int query_heuristic(SharedEnvironment* env, int src, int src_orient, int dest) {
    if (src < 0 || src >= (int)env->map.size()) return INTERVAL_MAX;
    if (dest < 0 || dest >= (int)env->map.size()) return INTERVAL_MAX;
    if (env->map[src] == 1 || env->map[dest] == 1) return INTERVAL_MAX;

    int c_src  = env->hpa_h.voronoi_map[src];
    int c_dest = env->hpa_h.voronoi_map[dest];

    if (c_src == -1 || c_dest == -1) return INTERVAL_MAX;
    if (c_src  >= (int)env->hpa_h.IntraHT.size()) return INTERVAL_MAX;
    if (c_dest >= (int)env->hpa_h.IntraHT.size()) return INTERVAL_MAX;

    int src_local  = env->hpa_h.global_to_local[src];
    int dest_local = env->hpa_h.global_to_local[dest];

    if (c_src == c_dest) {
        // Lazy compute if not cached
        if (env->hpa_h.IntraHT[c_src].find(dest_local) ==
            env->hpa_h.IntraHT[c_src].end()) {
            auto ht = CustomAlgo::build_IntraHT(env, c_src, dest);
            env->hpa_h.IntraHT[c_src][dest_local] =
                std::vector<std::array<int,4>>(ht.begin(), ht.end());
        }
        int cluster_size = env->hpa_h.local_to_global[c_src].size();
        if (src_local  >= cluster_size) return INTERVAL_MAX;
        if (dest_local >= cluster_size) return INTERVAL_MAX;
        return env->hpa_h.IntraHT[c_src][dest_local][src_local][src_orient];
    }

    int best = INTERVAL_MAX;

    for (int g_s : env->hpa_h.Gates[c_src]) {
        int g_s_local = env->hpa_h.global_to_local[g_s];
        int g_s_idx   = env->hpa_h.gate_index[g_s];
        if (g_s_idx == INTERVAL_MAX) continue;

        // g_s must be a precomputed gate destination in IntraHT
        if (env->hpa_h.IntraHT[c_src].find(g_s_local) ==
            env->hpa_h.IntraHT[c_src].end()) continue;

        int cluster_size_src = env->hpa_h.local_to_global[c_src].size();
        if (src_local >= cluster_size_src) continue;

        int cost_src_to_gs =
            env->hpa_h.IntraHT[c_src][g_s_local][src_local][src_orient];
        if (cost_src_to_gs == INTERVAL_MAX) continue;

        // Lazy inter cache
        if (env->hpa_h.inter_cache.find(g_s_idx) ==
            env->hpa_h.inter_cache.end())
            CustomAlgo::compute_inter_from(env, g_s_idx);

        for (int g_d : env->hpa_h.Gates[c_dest]) {
            int g_d_local = env->hpa_h.global_to_local[g_d];
            int g_d_idx   = env->hpa_h.gate_index[g_d];
            if (g_d_idx == INTERVAL_MAX) continue;

            int cost_inter = env->hpa_h.inter_cache[g_s_idx][g_d_idx];
            if (cost_inter == INTERVAL_MAX) continue;

            // g_d must be a precomputed gate destination in IntraHT
            if (env->hpa_h.IntraHT[c_dest].find(dest_local) ==
                env->hpa_h.IntraHT[c_dest].end()) continue;

            int cluster_size_dest = env->hpa_h.local_to_global[c_dest].size();
            if (g_d_local  >= cluster_size_dest) continue;
            if (dest_local >= cluster_size_dest) continue;

            int cost_gd_to_dest = INTERVAL_MAX;
            for (int orient = 0; orient < 4; orient++)
                cost_gd_to_dest = min(cost_gd_to_dest,
                    env->hpa_h.IntraHT[c_dest][dest_local][g_d_local][orient]);
            if (cost_gd_to_dest == INTERVAL_MAX) continue;

            int total = cost_src_to_gs + cost_inter + cost_gd_to_dest;
            if (total < best) best = total;
        }
    }

    return best;
}
    
}