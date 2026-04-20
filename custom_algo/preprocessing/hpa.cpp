#include "hpa.h"
#include <fstream>
#include <unordered_set>
namespace CustomAlgo{

    
    /**
     * @brief Bikin dua array untuk konversi lokasi dari lokasi lokal dalam cluster ke peta secara global, dan sebaliknya.
     * 
     * @param env 
     */
    void cluster_indexing(SharedEnvironment* env) {    
        env->hpa_h.local_to_global.resize(env->k);
        env->hpa_h.global_to_local.resize(env->map.size(), -1);

        // Iterasi lokasi
        for(int loc = 0; loc < env->map.size() ; loc++){
            
            // Cek lokasi obstacle atau bukan
            if (env->map[loc] == 1 || env->hpa_h.voronoi_map[loc] == -1) continue;
            
            int c = env->hpa_h.voronoi_map[loc];
            int idx = env->hpa_h.local_to_global[c].size();
            
            // Index lokal dapat dicatat mengikuti ukuran cluster c terkini, jadi nilai yang sama bisa berulang 
            env->hpa_h.global_to_local[loc] = idx;

            // Index global dicatat langsung memasukkan nilai global ke array dari cluster yang bersangkutan 
            env->hpa_h.local_to_global[c].push_back(loc);
            
        }
    }

    void build_cluster_graph(SharedEnvironment* env) {
        int k = env->k;
        env->hpa_h.cluster_adj.assign(k, std::unordered_map<int,int>());
        
        for (const Entrances& e : env->hpa_h.Ents) {
            int c_a = e.c_a, c_b = e.c_b;
            
            int size_a = env->hpa_h.local_to_global[c_a].size();
            int size_b = env->hpa_h.local_to_global[c_b].size();
            int traversal_cost = (size_a + size_b) / 2;
            
            int orient = getOrientationBetween(e.loc_a, e.neigh);
            int w = env->hpa_h.hw.r_e_hw.count({e.loc_a, orient}) 
                    ? env->c_penalty 
                    : traversal_cost; 
            
            auto& ab = env->hpa_h.cluster_adj[c_a][c_b];
            if (ab == 0 || w < ab) ab = w;
            
            auto& ba = env->hpa_h.cluster_adj[c_b][c_a];
            if (ba == 0 || w < ba) ba = w;
        }
    }

    /**
     * @brief Catat seluruh Entrances (pasangan Gate) secara global
     * 
     * @param env 
     */
    void build_entrances(SharedEnvironment* env) {
        env->hpa_h.Ents.clear();
        env->hpa_h.Gates.clear();
        env->hpa_h.Gates.resize(env->k);

        std::set<std::pair<int,int>> visited;
        std::vector<int> Neighbors;

    
        for (int loc = 0 ; loc < env->map.size(); loc++) {
            if(env->map[loc] == 1) continue;
            
            int c_a = env->hpa_h.voronoi_map[loc];
            CustomAlgo::getNeighborLocs(&(env->ns), Neighbors, loc);

            for (int neigh : Neighbors) {
                int c_b = env->hpa_h.voronoi_map[neigh];
                if (c_a == c_b || c_b == -1) continue;

                // Pastikan lokasi gate a dan b yang sama tidak dicatat berulang
                std::pair<int,int> pair_key = {min(loc,neigh), max(loc,neigh)};

                // Uniknya set::end() itu menunjuk ke benda SETELAH barang terakhir yang ada dalam set. Jadi selalu menunjuk ke barang yang tidak ada
                if (visited.find(pair_key) == visited.end()){
                    visited.insert(pair_key);

                    // Daftar Entrance
                    Entrances ent;
                    ent.c_a = c_a;
                    ent.c_b = c_b;
                    ent.loc_a = loc;
                    ent.neigh = neigh;
                    env->hpa_h.Ents.push_back(ent);

                    //Daftar Gates
                    env->hpa_h.Gates[c_a].push_back(loc);
                    env->hpa_h.Gates[c_b].push_back(neigh);
                }
            }
        }
    };

    /**
     * @brief Bangun IntraHT (heuristic antar gate DALAM cluster) menggunakan Backward Dijkstra. Proses dilakukan hanya untuk lokasi dest karena akan iterasi dari proses parent. PQ selalu menyimpan nilai global yang perlu di konversi ke lokal. 
     * 
     * Alasan: jika tidak salah, untuk konsistensi saat pertama kali pembuatan
     * 
     * @param env 
     * @param cluster 
     * @param dest 
     * @return std::vector<std::array<int, 4>> [src_local][orientation]
     */
    std::vector<std::array<int, 4>> build_IntraHT(SharedEnvironment* env, int cluster, int dest){

        //Untuk Dijkstra, pakai PQ
        priority_queue<HNode> pq;

        // Vektor yang berisikan [dest_local][src_local][orientasi]. Nanti akan dikembalikan untuk disimpan di vektor cluster
        std::vector<std::array<int, 4>> ht;

        int cluster_size = env->hpa_h.local_to_global[cluster].size();
        int dest_local = env->hpa_h.global_to_local[dest];

        // Langsung assign aja
        ht.assign(cluster_size, {INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX});

        //Inisialisasi untuk setiap orientasi di lokasi awal
        for (int orient = 0; orient < 4 ; orient++ ){
            ht[dest_local][orient] = 0;
            pq.push(HNode(dest,orient,0)); //Lokasi global
        }
        
        // Dijkstra 
        while (!pq.empty())
        {
            HNode curr = pq.top();
            pq.pop();
            
            int curr_local = env->hpa_h.global_to_local[curr.location];

            //Proses kalau sama dalam satu cluster, dan valuenya lebih rendah
            if (curr.value > ht[curr_local][curr.direction] || env->hpa_h.voronoi_map[curr.location] != cluster)  continue;

            //Hanya untuk mendapatkan seluruh orientasi pada lokasi
            for (int orient = 0; orient < 4 ; orient++ ){
                int rotCost = getRotationCost(curr.direction,orient);
                int newCost = curr.value + rotCost;
                if (newCost < ht[curr_local][orient]) {
                    ht[curr_local][orient] = newCost;
                    pq.push(HNode(curr.location,orient, newCost)); // Lokasi global
                }
            }

            //Ambil lokasi mundur berdasarkan orientasi dari current
            int prevLoc = getBackwardLocation(curr.location,curr.direction,env);

            if (prevLoc == -1 || env->hpa_h.voronoi_map[prevLoc] != cluster) continue;

            int prevLoc_local = env->hpa_h.global_to_local[prevLoc];
            
            int moveCost = 1;

            //Cek ada ato engga di r_e_hw
            if (env->hpa_h.hw.r_e_hw.count({prevLoc, curr.direction})){ 
                moveCost = env->c_penalty;
            }

            int newCost = curr.value + moveCost;
            if (newCost < ht[prevLoc_local][curr.direction]) {
                ht[prevLoc_local][curr.direction] = newCost;
                pq.push(HNode(prevLoc, curr.direction, newCost)); // Lokasi global
            }
        }

        return ht;
    };  

    /**
     * @brief Buat Abstract Graph (AG). AG berfungsi untuk mengabstraksi peta utama dan menggabungkan setiap komponen (gates, subgraph map, dll) menjadi sebuah objek
     * 
     * @param env 
     */
    void build_abstract_graph(SharedEnvironment* env) {
        int k = env->k;
        
        env->hpa_h.AG.intra.clear();     
        env->hpa_h.AG.inter.clear();     
        env->hpa_h.AG.neighbors.clear();
        
        // Assign aja untuk timpa, jadi ga perlu 2 kali pemanggilan
        env->hpa_h.IntraHT.assign(k, {});

        // Jalanin hanya sekali karena gates tidak akan berubah (tidak terpengaruh HW)
        if (env->hpa_h.AG.gates.empty())
            for (int c = 0; c < k; c++)
                for (int gate_loc : env->hpa_h.Gates[c])
                    env->hpa_h.AG.gates.push_back(gate_loc);
        
        //Masukkan Gate dan edges global
        for (const Entrances e : env->hpa_h.Ents) {
            env->hpa_h.AG.inter[{e.loc_a, e.neigh}] = 1;
            env->hpa_h.AG.inter[{e.neigh, e.loc_a}] = 1;
            env->hpa_h.AG.neighbors[e.loc_a].push_back(e.neigh);
            env->hpa_h.AG.neighbors[e.neigh].push_back(e.loc_a);
        }

        // Ganti dari (semua lokasi dalam cluster) ke (semua gates dalam cluster) untuk IntraHT
        std::vector<std::array<int, 4>> ht;
        for (int c = 0; c < k; c++) {
            for (int dest : env->hpa_h.Gates[c]) {
                int dest_local = env->hpa_h.global_to_local[dest];
                ht = build_IntraHT(env, c, dest);

                env->hpa_h.IntraHT[c][dest_local] = std::vector<std::array<int,4>>(ht.begin(), ht.end());
  
            }
        }
    }

    /**
     * @brief InterHT berfungsi untuk menghubungkan setiap gate dalam cluster A dengan seluruh gate dalam cluster B. Tetapi dimodifikasi sehingga menggunakan konsep lazy dan tidak dibangun sejak awal. Hanya ketika diminta pada scheduling/planning*
     * 
     * @param env 
     */
    void build_InterHT(SharedEnvironment* env) {
    
        // Build gate_index hanya pada pertama kali pemanggilan
        if (env->hpa_h.AG.gate_index.empty() ||
        env->hpa_h.AG.gate_index[env->hpa_h.AG.gates[0]] == INTERVAL_MAX) {
            
            env->hpa_h.AG.gate_index.assign(env->map.size(), INTERVAL_MAX);
    
            int gates_size = env->hpa_h.AG.gates.size();
            for (int i = 0; i < gates_size; i++) {
                env->hpa_h.AG.gate_index[env->hpa_h.AG.gates[i]] = i;
            }
        } 

        env->hpa_h.inter_cache.clear();
    }

    /**
     * @brief Dijkstra untuk InterHT. Mencari jalur terdekat antar gate-gate di cluster tujuan, dengan gate dari gate_idx. O(P x G_c)
     * 
     * @param env 
     * @param gate_idx 
     */
    // void compute_inter_from(SharedEnvironment* env, int gate_idx) {
    //     int gates_size = env->hpa_h.AG.gates.size();
    //     std::vector<int> dist(gates_size, INTERVAL_MAX);
        
    //     std::priority_queue<HNode> pq;
    //     pq.push(HNode::createForInterHT(0, gate_idx));
        
    //     dist[gate_idx] = 0;

    //     std::unordered_set<int> expanded_clusters;

    //     while (!pq.empty()) { //Dijkstra
    //         HNode curr = pq.top();
    //         pq.pop();
    //         int d = curr.value;
    //         int u = curr.idx;

    //         if (d > dist[u]) continue;

    //         int u_loc     = env->hpa_h.AG.gates[u];
    //         int u_cluster = env->hpa_h.voronoi_map[u_loc];
    //         int u_local   = env->hpa_h.global_to_local[u_loc];

    //         for (int v_loc : env->hpa_h.AG.neighbors[u_loc]) {  //Setiap tetangganya (1-2 seharusnya)

    //             int w;
    //             int orient = getOrientationBetween(u_loc, v_loc); 
               
    //             if (env->hpa_h.hw.r_e_hw.count({u_loc, orient}))   {// Kalau berlawanan arah highway
    //                 w = env->c_penalty;
    //             } else {
    //                 if (env->hpa_h.AG.inter.count({u_loc, v_loc})) { //Kalau termasuk diluar cluster
    //                     w = env->hpa_h.AG.inter[{u_loc, v_loc}];
    //                 } else {                                           //Kalau dalam cluster
    //                     w = env->hpa_h.AG.intra[{u_loc, v_loc}];
    //                 }
    //             }
                
    //             int v = env->hpa_h.AG.gate_index[v_loc];
    //             if (v == INTERVAL_MAX) continue;

    //             int nd = dist[u] + w;
    //             if (nd < dist[v]) {
    //                 dist[v] = nd;
    //                 pq.push(HNode::createForInterHT(nd, v));
    //             }
    //         }

    //     }
        
    //     env->hpa_h.inter_cache[gate_idx] = dist;
    // }

    void compute_inter_from(SharedEnvironment* env, int gate_idx) {
        int gates_size = env->hpa_h.AG.gates.size();
        std::vector<int> dist(gates_size, INTERVAL_MAX);
        std::priority_queue<HNode> pq;
        pq.push(HNode::createForInterHT(0, gate_idx));
        dist[gate_idx] = 0;

        std::unordered_set<int> expanded_gates;

        while (!pq.empty()) {
            HNode curr = pq.top(); pq.pop();
            int d = curr.value;
            int u = curr.idx;
            if (d > dist[u]) continue;

            int u_loc     = env->hpa_h.AG.gates[u];
            int u_cluster = env->hpa_h.voronoi_map[u_loc];
            int u_local   = env->hpa_h.global_to_local[u_loc];

            // INTRA — only once per gate
            if (expanded_gates.insert(u).second) {
                for (int g2 : env->hpa_h.Gates[u_cluster]) {
                    
                    if (g2 == u_loc) continue;
                    int g2_local = env->hpa_h.global_to_local[g2];
                    
                    auto intra_it = env->hpa_h.IntraHT[u_cluster].find(g2_local);
                    if (intra_it == env->hpa_h.IntraHT[u_cluster].end()) continue;
                   
                    int w = INTERVAL_MAX;
                    for (int o = 0; o < 4; o++) {
                        w = std::min(w, intra_it->second[u_local][o]);
                    }
                    if (w >= INTERVAL_MAX) continue;

                    int v = env->hpa_h.AG.gate_index[g2];
                    if (v >= INTERVAL_MAX) continue;
                    
                    int nd = d + w;
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        pq.push(HNode::createForInterHT(nd, v));
                    }
                }
            }

            // INTER
            for (int v_loc : env->hpa_h.AG.neighbors[u_loc]) {
                int orient = getOrientationBetween(u_loc, v_loc);
                int w;
                if (env->hpa_h.hw.r_e_hw.count({u_loc, orient})) {
                    w = env->c_penalty;
                } else {
                    w = env->hpa_h.AG.inter[{u_loc, v_loc}];
                }
                int v = env->hpa_h.AG.gate_index[v_loc];
                if (v >= INTERVAL_MAX) continue;
                int nd = d + w;
                if (nd < dist[v]) {
                    dist[v] = nd;
                    pq.push(HNode::createForInterHT(nd, v));
                }
            }
        }

        env->hpa_h.inter_cache[gate_idx] = dist;
    }

    // void compute_cluster_from(SharedEnvironment* env, int c_src) {
    //     int k = env->k;
    //     std::vector<int> dist(k, INTERVAL_MAX);
    //     std::priority_queue<std::pair<int,int>, std::vector<std::pair<int,int>>, std::greater<>> pq;
    //     dist[c_src] = 0;
    //     pq.push({0, c_src});
        
    //     while (!pq.empty()) {
    //         auto [d, u] = pq.top(); pq.pop();
    //         if (d > dist[u]) continue;
    //         for (auto& [v, w] : env->hpa_h.cluster_adj[u]) {
    //             int nd = d + w;
    //             if (nd < dist[v]) { dist[v] = nd; pq.push({nd, v}); }
    //         }
    //     }
    //     env->hpa_h.cluster_cache[c_src] = dist;
    // }

    // /**
    //  * @brief Warmup inter cache. Membuat InterHT hanya untuk cluster yang ditempati oleh agen sejak pertama kali dimasukkan.
    //  * 
    //  * @param env 
    //  */
    // void warmup_inter_cache(SharedEnvironment* env) {
    //     std::set<int> agent_clusters;
    //     for (int loc : env->agent_starts) {
    //         int c = env->hpa_h.voronoi_map[loc];
    //         agent_clusters.insert(c);
    //     }
        
    //     // Untuk setiap cluster yang ditempat, buat interHT nya
    //     for (int c = 0; c < env->k; c++) compute_cluster_from(env, c);
    // }

    void warmup_inter_cache(SharedEnvironment* env) {
        std::set<int> agent_clusters;
        for (int loc : env->agent_starts) {
            int c = env->hpa_h.voronoi_map[loc];
            agent_clusters.insert(c);
        }

        auto t_plan_start = std::chrono::steady_clock::now();
        auto elapsed = [&]() {
            return std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_plan_start).count();
        };

        int gates_size = env->hpa_h.Gates.size();
        // Untuk setiap cluster yang ditempat, buat interHT nya
        for (int c : agent_clusters){
            for (int g : env->hpa_h.Gates[c]){
                int g_idx = env->hpa_h.AG.gate_index[g];
                if(g_idx == INTERVAL_MAX || env->hpa_h.inter_cache.count(g_idx)) continue;

                std::cout << "[WARMUP] calling compute_inter_from gate_idx=" << g_idx << std::flush;
                auto t0 = std::chrono::steady_clock::now();

                compute_inter_from(env, g_idx);
                
                double ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count();
                std::cout << " done in " << ms << "ms" << std::endl;
            }
        }
    }

    void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids) {
        auto _phase_start = std::chrono::steady_clock::now();
        auto elapsed_ms = [&]() {
            return std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - _phase_start).count();
        };
        auto log_step = [&](const std::string& label, double ms) {
            std::cout << "[PREPROCESS]   " << label << " : " << ms << " ms"
                    << "  (cumulative: " << elapsed_ms() << " ms)" << std::endl;
        };
    
        std::cout << "[PREPROCESS] --- generate_HPAHMap START ---" << std::endl;
    
        // Step 1 : cluster_indexing
        {
            auto t0 = std::chrono::steady_clock::now();
            cluster_indexing(env);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            log_step("Step 1 | cluster_indexing", ms);
        }
    
        // Step 2a : build_entrances
        {
            auto t0 = std::chrono::steady_clock::now();
            build_entrances(env);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            log_step("Step 2a | build_entrances", ms);
            // Log gate/entrance counts if accessible
            if (env->hpa_h.AG.gates.size() > 0) {
                std::cout << "[PREPROCESS]   -> Gates count : " << env->hpa_h.AG.gates.size() << std::endl;
            }
        }
        // // Step 2- build cluster graph
        // {
        //     auto t0 = std::chrono::steady_clock::now();
        //     build_cluster_graph(env);
        //     double ms = std::chrono::duration<double, std::milli>(
        //         std::chrono::steady_clock::now() - t0).count();
        //     log_step("Step 2---- | build cluster graph", ms);
        // }
    
        // Step 2b : build_abstract_graph (1st pass, no highway)
        {
            auto t0 = std::chrono::steady_clock::now();
            build_abstract_graph(env);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            log_step("Step 2b | build_abstract_graph (1st, pre-highway)", ms);
            if (ms > 5000) {
                std::cerr << "[PREPROCESS]   WARNING: build_abstract_graph (1st) took " << ms
                        << " ms — this is the known bottleneck (IntraHT per gate per cluster)." << std::endl;
            }
        }
    
        // Step 2c : build_InterHT (1st pass)
        {
            auto t0 = std::chrono::steady_clock::now();
            build_InterHT(env);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            log_step("Step 2c | build_InterHT (1st, lazy)", ms);
        }
    
        // Step 3 : generateHighways
        {
            auto t0 = std::chrono::steady_clock::now();
            generateHighways(env, centroids);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            log_step("Step 3 | generateHighways", ms);
            // Log highway edge count if accessible
            // if (env->r_e_hw.size() > 0) {
            //     std::cout << "[PREPROCESS]   -> Highway reverse-edges count : "
            //               << env->r_e_hw.size() << std::endl;
            // }
        }
    
        // Step 4a : build_abstract_graph (2nd pass, with highway)
        {
            auto t0 = std::chrono::steady_clock::now();
            build_abstract_graph(env);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            log_step("Step 4a | build_abstract_graph (2nd, post-highway)", ms);
            if (ms > 5000) {
                std::cerr << "[PREPROCESS]   WARNING: build_abstract_graph (2nd) took " << ms
                        << " ms — check if R_E_HW penalty is causing extra work." << std::endl;
            }
        }

         if (env->hpa_h.AG.gates.size() > 0) {
                std::cout << "[PREPROCESS]   -> Gates count : " << env->hpa_h.AG.gates.size() << std::endl;
            }
    
        // Step 4b : build_InterHT (2nd pass)
        {
            auto t0 = std::chrono::steady_clock::now();
            build_InterHT(env);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            log_step("Step 4b | build_InterHT (2nd, lazy)", ms);
        }
    
        // Step 5 : warmup_inter_cache
        {
            auto t0 = std::chrono::steady_clock::now();
            warmup_inter_cache(env);
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            log_step("Step 5 | warmup_inter_cache", ms);
            if (ms > 2000) {
                std::cerr << "[PREPROCESS]   WARNING: warmup_inter_cache took " << ms
                        << " ms — number of agent-cluster gates may be large." << std::endl;
            }
        }
    
        std::cout << "[PREPROCESS] --- generate_HPAHMap END | total : "
                << elapsed_ms() << " ms ---" << std::endl;
    }

    // void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids) {
    //     // Step 0 : Inisialisasi vector neighbor, menyimpan setiap lokasi neighbor dari setiap usable cell
    //     // CustomAlgo::init_neighbor(env);

    //     //Step 1 : Index semua cluster
    //     cluster_indexing(env);

    //     // Step 2 : Bikin HPA pertama
    //     build_entrances(env);
    //     build_abstract_graph(env);
    //     build_InterHT(env);
    
    //     // Step 3 : Generate Highways
    //     generateHighways(env, centroids);

    //     // Step 4 : HPA + Highways, Perubahan dari pseudocode: r_e_hw disimpan dalam env
    //     build_abstract_graph(env);    
    //     build_InterHT(env);
        
    //     warmup_inter_cache(env);
    // }
}