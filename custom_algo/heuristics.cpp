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
        if (env->map[src] == 1 || env->map[dest] == 1) return INTERVAL_MAX;

        int c_src  = env->hpa_h.voronoi_map[src];
        int c_dest = env->hpa_h.voronoi_map[dest];

        //Cek cluster valid
        if (c_src == -1 || c_dest == -1) return INTERVAL_MAX;

        int src_local  = env->hpa_h.global_to_local[src];
        int dest_local = env->hpa_h.global_to_local[dest];

        if (env->hpa_h.IntraHT[c_dest].find(dest_local) == env->hpa_h.IntraHT[c_dest].end()) {
                
            auto ht = build_IntraHT(env, c_dest, dest);
            env->hpa_h.IntraHT[c_dest][dest_local] = std::vector<std::array<int,4>>(ht.begin(), ht.end());
        }

        //IntraHT (Jika destination dan source dalam cluster yang sama)
        if (c_src == c_dest) {

            // Validasi source dan destionation secara lokal
            int cluster_size = env->hpa_h.local_to_global[c_src].size();

            // Lazy compute jika belum ada , tapi seharusnya udah ada karena c_dest = c_src ,dan c_dest sudah dicari sebelumnya
            if (env->hpa_h.IntraHT[c_src].find(dest_local) == 
                env->hpa_h.IntraHT[c_src].end()) {

                if (env->hpa_h.IntraHT[c_src].find(dest_local) == 
                    env->hpa_h.IntraHT[c_src].end()) {
                    auto ht = build_IntraHT(env, c_src, dest);
                    env->hpa_h.IntraHT[c_src][dest_local] = 
                        std::vector<std::array<int,4>>(ht.begin(), ht.end());
                }
            }

            int val = env->hpa_h.IntraHT[c_src][dest_local][src_local][src_orient];

            //Kalau satu cluster, tapi tidak dibatasi sama sebuah "dinding", bisa kembalikan nilai val. Kalau ternyata ada batas, lanjut InterHT
            if (val < INTERVAL_MAX) return val;
        }

        int best = INTERVAL_MAX;

        // Cek untuk setiap gate pada cluter source

        for (int g_s : env->hpa_h.Gates[c_src]) {


            int g_s_local = env->hpa_h.global_to_local[g_s];
            int g_s_idx   = env->hpa_h.AG.gate_index[g_s];
            if (g_s_idx >= INTERVAL_MAX) continue;

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

            //============================================================

            // FASE 1 : Cost dari lokasi ke gate cluster lokal
            int cost_src_to_gs = env->hpa_h.IntraHT[c_src][g_s_local][src_local][src_orient];
            if (cost_src_to_gs >= INTERVAL_MAX) continue;

            //Cek InterHT, dari gate cluster source udah pernah dihitung atau belum. Kalau belum, compute dan cache
            if (env->hpa_h.inter_cache.find(g_s_idx) == 
            env->hpa_h.inter_cache.end()) {
                CustomAlgo::compute_inter_from(env, g_s_idx);
            }

            // Untuk setiap gate pada cluster tujuan
            for (int g_d : env->hpa_h.Gates[c_dest]) {

                // Validasi awal
                int g_d_local = env->hpa_h.global_to_local[g_d];
                int g_d_idx   = env->hpa_h.AG.gate_index[g_d];
                if (g_d_idx >= INTERVAL_MAX) continue;

                if (env->hpa_h.IntraHT[c_dest].find(g_d_local) ==
                env->hpa_h.IntraHT[c_dest].end()) {

                    auto ht = CustomAlgo::build_IntraHT(env, c_dest, g_d);
                    env->hpa_h.IntraHT[c_dest][g_d_local] =
                        std::vector<std::array<int,4>>(ht.begin(), ht.end());
                }

                
                // Fase 2 : Ambil heuristik antar
                int cost_inter = env->hpa_h.inter_cache[g_s_idx].at(g_d_idx);
                if (cost_inter >= INTERVAL_MAX) continue;

                //Cek lokasinya gate valid atau tidak
                if (env->hpa_h.IntraHT[c_dest].find(dest_local) ==
                    env->hpa_h.IntraHT[c_dest].end()) continue;

                int cluster_size_dest = env->hpa_h.local_to_global[c_dest].size();
                if (g_d_local  >= cluster_size_dest) continue;
                if (dest_local >= cluster_size_dest) continue;

                //Fase 3 : Heuristik dari gate destinasi ke tujuan (dest)
                // Ambil nilai dari ke 4 orientasi untuk jarak terpendek
                int cost_gd_to_dest = INTERVAL_MAX;
                for (int orient = 0; orient < 4; orient++) {
                    cost_gd_to_dest = min(cost_gd_to_dest,
                        env->hpa_h.IntraHT[c_dest][dest_local][g_d_local][orient]);
                }


                if (cost_gd_to_dest >= INTERVAL_MAX) continue;

                //Tambahkan total keseluruhan
                int total = cost_src_to_gs + cost_inter + cost_gd_to_dest;
                
                if (total < best) best = total;
            }
        }

        //Fallback kalo ga nemu...
        // Untuk coba atasi agent yang ga dapet assigned sama sekali.. (rejected agent)
        if (best >= INTERVAL_MAX) return manhattanDistance(src, dest, env);

        return best;
    }
    
}