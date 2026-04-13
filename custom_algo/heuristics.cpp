#include "heuristics.h"
#include <queue>
#include "preprocessing/hpa.h"

namespace CustomAlgo{
    //Diambil dari framework standard
    Neighbors global_neighbors;

    
    void init_neighbor(SharedEnvironment* env){
        global_neighbors.resize(env->rows*env->cols);
        int cols = env->cols;
        
        for (int loc = 0 ; loc < env->map.size() ; loc ++) {
            if (env->map[loc] == 0 ){
                if (loc - cols >= 0 && env->map[loc - cols] == 0 && validateMove(loc, loc - cols, env)) {
                    global_neighbors[loc].push_back(loc-cols);
                }
                if (loc - 1 >= 0 && env->map[loc - 1] == 0 && validateMove(loc, loc - 1, env)) {
                    global_neighbors[loc].push_back(loc - 1);
                }
                if (loc + 1 < env->map.size() && env->map[loc+1] == 0 && validateMove(loc, loc + 1, env)) {
                    global_neighbors[loc].push_back(loc + 1);
                }
                if (loc + cols < env->map.size() && env-> map[loc+cols] == 0 && validateMove(loc, loc + cols, env)) {
                    global_neighbors[loc].push_back(loc + cols);
                    
                }
            }
        }
        env->ns = global_neighbors;
    }

    /**
     * @brief Untuk mencari heuristik antar destination ke source. Dan melakukan lazy compute jika belum ada.
     * 
     * @param env 
     * @param src 
     * @param src_orient 
     * @param dest 
     * @return int 
     */
    int query_heuristic(SharedEnvironment* env, int src, int src_orient, int dest) {

        // Cek lokasi tujuan dan destinasi valid
        // if (src < 0 || src >= env->map.size()) return INTERVAL_MAX;
        // if (dest < 0 || dest >= env->map.size()) return INTERVAL_MAX;
        if (env->map[src] == 1 || env->map[dest] == 1) return INTERVAL_MAX;

        int c_src  = env->hpa_h.voronoi_map[src];
        int c_dest = env->hpa_h.voronoi_map[dest];

        // env->logger->log_info("QH-PASS=1" + std::to_string(ms_since(t0)) + "ms",
        // env->curr_timestep);
        
        //Cek cluster valid
        if (c_src == -1 || c_dest == -1) return INTERVAL_MAX;
        // if (c_src  >= env->hpa_h.IntraHT.size()) return INTERVAL_MAX;
        // if (c_dest >= env->hpa_h.IntraHT.size()) return INTERVAL_MAX;

        int src_local  = env->hpa_h.global_to_local[src];
        int dest_local = env->hpa_h.global_to_local[dest];

        if (env->hpa_h.IntraHT[c_dest].find(dest_local) ==
            env->hpa_h.IntraHT[c_dest].end()) {
            auto ht = CustomAlgo::build_IntraHT(env, c_dest, dest);
            env->hpa_h.IntraHT[c_dest][dest_local] =
                std::vector<std::array<int,4>>(ht.begin(), ht.end());
        }

        // env->logger->log_info("QH-PASS=2" + std::to_string(ms_since(t0)) + "ms",
        // env->curr_timestep);
        //IntraHT (Jika destination dan source dalam cluster yang sama)
        if (c_src == c_dest) {

            // Validasi source dan destionation secara lokal
            int cluster_size = env->hpa_h.local_to_global[c_src].size();
            // if (src_local  >= cluster_size) return INTERVAL_MAX;
            // if (dest_local >= cluster_size) return INTERVAL_MAX;
            
        //     env->logger->log_info("QH-PASS=3-INTRA" + std::to_string(ms_since(t0)) + "ms",
        // env->curr_timestep);
            // Lazy compute jika belum ada
            if (env->hpa_h.IntraHT[c_src].find(dest_local) == 
                env->hpa_h.IntraHT[c_src].end()) {
                // std::lock_guard<std::mutex> lock(intra_ht_mutex);
                // double-check after lock
                if (env->hpa_h.IntraHT[c_src].find(dest_local) == 
                    env->hpa_h.IntraHT[c_src].end()) {
                    auto ht = CustomAlgo::build_IntraHT(env, c_src, dest);
                    env->hpa_h.IntraHT[c_src][dest_local] = 
                        std::vector<std::array<int,4>>(ht.begin(), ht.end());
                }
            }

        //     env->logger->log_info("QH-PASS=4-INTRA" + std::to_string(ms_since(t0)) + "ms",
        // env->curr_timestep);
            
            // return env->hpa_h.IntraHT[c_src][dest_local][src_local][src_orient];
            int val = env->hpa_h.IntraHT[c_src][dest_local][src_local][src_orient];
            // if (src == 360) {
            //     cout << "same cluster query: c=" << c_src 
            //         << " dest_local=" << dest_local 
            //         << " src_local=" << src_local
            //         << " src_orient=" << src_orient
            //         << " val=" << val << endl;
            // }
            return val;
        }

        int best = INTERVAL_MAX;
        // env->logger->log_info("QH-PASS=5" + std::to_string(ms_since(t0)) + "ms",
        // env->curr_timestep);
        // Cek untuk setiap gate pada cluter source
        for (int g_s : env->hpa_h.Gates[c_src]) {

        //     env->logger->log_info("QH-PASS=6-GATE" + std::to_string(g_s) + " === " + std::to_string(ms_since(t0)) + "ms",
        // env->curr_timestep);

            int g_s_local = env->hpa_h.global_to_local[g_s];
            int g_s_idx   = env->hpa_h.AG.gate_index[g_s];
            if (g_s_idx == INTERVAL_MAX) continue;

        //     env->logger->log_info("QH-PASS=7-GATE" + std::to_string(g_s_local) + " === "+ std::to_string(g_s_idx) + " === " + std::to_string(ms_since(t0)) + "ms",
        // env->curr_timestep);

                //Validasi awal (cek udah dihitung atau belum)
                if (env->hpa_h.IntraHT[c_src].find(g_s_local) ==
                env->hpa_h.IntraHT[c_src].end()) {

                // lazy compute IntraHT for this gate as destination
                auto ht = CustomAlgo::build_IntraHT(env, c_src, g_s);
                env->hpa_h.IntraHT[c_src][g_s_local] =
                    std::vector<std::array<int,4>>(ht.begin(), ht.end());
            }

                
            int cluster_size_src = env->hpa_h.local_to_global[c_src].size();
            if (src_local >= cluster_size_src) continue;



            // FASE 1 : Cost dari lokasi ke gate cluster lokal
            int cost_src_to_gs =
                env->hpa_h.IntraHT[c_src][g_s_local][src_local][src_orient];
            if (cost_src_to_gs >= INTERVAL_MAX) continue;

// env->logger->log_info("QH-PASS=8-GATE-FASE1" + std::to_string(ms_since(t0)) + "ms",
//         env->curr_timestep);

            //Cek InterHT, dari gate cluster source udah pernah dihitung atau belum. Kalau belum, compute dan cache
            if (env->hpa_h.inter_cache.find(g_s_idx) == 
            env->hpa_h.inter_cache.end()) {
            // std::lock_guard<std::mutex> lock(inter_cache_mutex);
            if (env->hpa_h.inter_cache.find(g_s_idx) == 
                env->hpa_h.inter_cache.end()) {
                CustomAlgo::compute_inter_from(env, g_s_idx);
            }
        }

            // Untuk setiap gate pada cluster tujuan
            for (int g_d : env->hpa_h.Gates[c_dest]) {

                // Validasi awal
                int g_d_local = env->hpa_h.global_to_local[g_d];
                int g_d_idx   = env->hpa_h.AG.gate_index[g_d];
                if (g_d_idx == INTERVAL_MAX) continue;

                if (env->hpa_h.IntraHT[c_dest].find(g_d_local) ==
                env->hpa_h.IntraHT[c_dest].end()) {
                auto ht = CustomAlgo::build_IntraHT(env, c_dest, g_d);
                env->hpa_h.IntraHT[c_dest][g_d_local] =
                    std::vector<std::array<int,4>>(ht.begin(), ht.end());
            }
                // env->logger->log_info("QH-PASS=8-GATE-D-FASE-V" + std::to_string(ms_since(t0)) + "ms",
                // env->curr_timestep);
                
                // Fase 2 : Ambil heuristik antar
                //Cek di map inter_cache sudah ada gate tujuan atau belum
                int cost_inter = env->hpa_h.inter_cache[g_s_idx][g_d_idx];
                if (cost_inter >= INTERVAL_MAX) continue;

                // env->logger->log_info("QH-PASS=8-GATE-D-FASE-COST-INTER" + std::to_string(ms_since(t0)) + "ms",
                // env->curr_timestep);

                //Cek lokasinya gate valid atau tidak
                if (env->hpa_h.IntraHT[c_dest].find(dest_local) ==
                    env->hpa_h.IntraHT[c_dest].end()) continue;

                // env->logger->log_info("QH-PASS=8-GATE-D-FASE-COST-VALID" + std::to_string(ms_since(t0)) + "ms",
                // env->curr_timestep);
                
                int cluster_size_dest = env->hpa_h.local_to_global[c_dest].size();
                if (g_d_local  >= cluster_size_dest) continue;
                if (dest_local >= cluster_size_dest) continue;

                // env->logger->log_info("QH-PASS=8-GATE-D-FASE-2" + std::to_string(ms_since(t0)) + "ms",
                // env->curr_timestep);

                //Fase 3 : Heuristik dari gate destinasi ke tujuan (dest)
                // Ambil nilai dari ke 4 orientasi untuk jarak terpendek
                int cost_gd_to_dest = INTERVAL_MAX;
                for (int orient = 0; orient < 4; orient++) {
                    cost_gd_to_dest = min(cost_gd_to_dest,
                        env->hpa_h.IntraHT[c_dest][dest_local][g_d_local][orient]);

                // env->logger->log_info("QH-PASS=8-GATE-D-FASE-COST-INTER-LOOP" + std::to_string(ms_since(t0)) + "ms",
                // env->curr_timestep);
                }


                if (cost_gd_to_dest >= INTERVAL_MAX) continue;

                //Tambahkan total keseluruhan
                int total = cost_src_to_gs + cost_inter + cost_gd_to_dest;
                
                if (total < best) best = total;
            }
        }

                //         env->logger->log_info("QH-DONE" + std::to_string(ms_since(t0)) + "ms",
                // env->curr_timestep);
        return best;
    }
    
}