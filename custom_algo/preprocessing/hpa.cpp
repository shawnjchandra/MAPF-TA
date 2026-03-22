#include "hpa.h"
#include <fstream>
#include <unordered_set>
namespace CustomAlgo{
    void cluster_indexing(SharedEnvironment* env) {    
        env->hpa_h.local_to_global.resize(env->k);
        env->hpa_h.global_to_local.resize(env->map.size(), -1);
        for(int loc = 0; loc < env->map.size() ; loc++){
            
            // Cek lokasi obstacle atau bukan
            if (env->map[loc] == 1) continue;
            if (env->hpa_h.voronoi_map[loc] == -1) continue;
            int c = env->hpa_h.voronoi_map[loc];

            int idx = env->hpa_h.local_to_global[c].size();

            env->hpa_h.local_to_global[c].push_back(loc);
            env->hpa_h.global_to_local[loc] = idx;
        }
    }

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
     * @brief 
     * Perubahan
     * 
     * @param env 
     * @param cluster 
     * @param dest 
     * @return std::vector<std::array<int, 4>> 
     */
    std::vector<std::array<int, 4>> build_IntraHT(SharedEnvironment* env, int cluster, int dest){
        priority_queue<HNode> pq;
        std::vector<std::array<int, 4>> ht;

        int cluster_size = env->hpa_h.local_to_global[cluster].size();
        int dest_local = env->hpa_h.global_to_local[dest];

        ht.resize(cluster_size);
        ht.assign(cluster_size, {INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX});

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
            if (curr.value> ht[curr_local][curr.direction] || env->hpa_h.voronoi_map[curr.location] != cluster)  continue;

            for (int orient = 0; orient < 4 ; orient++ ){
                int rotCost = getRotationCost(curr.direction,orient);
                int newCost = curr.value + rotCost;
                if (newCost < ht[curr_local][orient]) {
                    ht[curr_local][orient] = newCost;
                    pq.push(HNode(curr.location,orient, newCost)); // Lokakasi global
                }
            }
            int prevLoc = getBackwardLocation(curr.location,curr.direction,env);

            if (prevLoc == -1 || env->hpa_h.voronoi_map[prevLoc] != cluster) continue;
            int prevLoc_local = env->hpa_h.global_to_local[prevLoc];
            
            int moveCost = 1;
            if (env->hpa_h.hw.r_e_hw.count({prevLoc, curr.direction})){ //Cek ada ato engga di r_e_hw
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
     * @brief 
     * 
     * @param env 
     */
    void build_abstract_graph(SharedEnvironment* env) {
        int k = env->k;

        env->hpa_h.IntraHT.clear();      
        // env->hpa_h.AG.gates.clear();     
        env->hpa_h.AG.intra.clear();     
        env->hpa_h.AG.inter.clear();     
        env->hpa_h.AG.neighbors.clear();
        env->hpa_h.IntraHT.resize(k);

        // Jalanin hanya sekali karena gates tidak akan berubah (tidak terpengaruh HW)
        if (env->hpa_h.AG.gates.empty())
            for (int c = 0; c < k; c++)
                for (int gate_loc : env->hpa_h.Gates[c])
                    env->hpa_h.AG.gates.push_back(gate_loc);
        
        for (const Entrances e : env->hpa_h.Ents) {
            env->hpa_h.AG.inter[{e.loc_a, e.neigh}] = 1;
            env->hpa_h.AG.inter[{e.neigh, e.loc_a}] = 1;
            env->hpa_h.AG.neighbors[e.loc_a].push_back(e.neigh);
            env->hpa_h.AG.neighbors[e.neigh].push_back(e.loc_a);
        }

        // Ganti dari (semua lokasi dalam cluster) ke (semua gates dalam cluster)
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
     * @brief Map atau hubungkan setiap gate dengan sebuah index penanda 
     * 
     * @param env 
     */
    void build_InterHT(SharedEnvironment* env) {
    
        // Build gate_index hanya pada pertama kali pemanggilan
        if (env->hpa_h.AG.gate_index.empty() ||
        env->hpa_h.AG.gate_index[env->hpa_h.AG.gates[0]] == INTERVAL_MAX) {
            
            env->hpa_h.AG.gate_index.assign(env->map.size(), INTERVAL_MAX);
    
            int gates_size = env->hpa_h.AG.gates.size();
            for (int i = 0; i < gates_size; i++)
                env->hpa_h.AG.gate_index[env->hpa_h.AG.gates[i]] = i;
        } 

        env->hpa_h.inter_cache.clear();
    }

    /**
     * @brief Dijkstra untuk interHT, Konsep Lazy, hanya digunakan 
     * 
     * @param env 
     * @param gate_idx 
     */
    void compute_inter_from(SharedEnvironment* env, int gate_idx) {
        int gates_size = env->hpa_h.AG.gates.size();
        std::vector<int> dist(gates_size, INTERVAL_MAX);
        
        std::priority_queue<HNode> pq;
        pq.push(HNode::createForInterHT(0, gate_idx));
        
        dist[gate_idx] = 0;

        while (!pq.empty()) { //Dijkstra
            HNode curr = pq.top();
            pq.pop();
            int d = curr.value;
            int u = curr.idx;

            if (d > dist[u]) continue;

            int u_loc = env->hpa_h.AG.gates[u];
            for (int v_loc : env->hpa_h.AG.neighbors[u_loc]) {  //Setiap tetangganya (1-2 seharusnya)

                int w;
                int orient = getOrientationBetween(u_loc, v_loc); 
               
                if (env->hpa_h.hw.r_e_hw.count({u_loc, orient}))   {// Kalau berlawanan arah highway
                    w = env->c_penalty;
                } else {
                    if (env->hpa_h.AG.inter.count({u_loc, v_loc})) { //Kalau termasuk diluar cluster
                        w = env->hpa_h.AG.inter[{u_loc, v_loc}];
                    } else {                                           //Kalau dalam cluster
                        w = env->hpa_h.AG.intra[{u_loc, v_loc}];
                    }
                    }
                
                int v = env->hpa_h.AG.gate_index[v_loc];
                if (v == INTERVAL_MAX) continue;

                int nd = dist[u] + w;
                if (nd < dist[v]) {
                    dist[v] = nd;
                    pq.push(HNode::createForInterHT(nd, v));
                }
            }
        }
        
        env->hpa_h.inter_cache[gate_idx] = dist;
    }

    void warmup_inter_cache(SharedEnvironment* env) {
        std::set<int> agent_clusters;
        for (int loc : env->agent_starts) {
            int c = env->hpa_h.voronoi_map[loc];
            agent_clusters.insert(c);
        }
        
        for (int c : agent_clusters){
            for (int g : env->hpa_h.Gates[c]){
                int g_idx = env->hpa_h.AG.gate_index[g];
                if(g_idx == INTERVAL_MAX || env->hpa_h.inter_cache.count(g_idx)) continue;
                compute_inter_from(env, g_idx);
            }
        }
    }

    // void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids) {
    //     // Step 0 : Inisialisasi vector neighbor, menyimpan setiap lokasi neighbor dari setiap usable cell
    //     CustomAlgo::init_neighbor(env);

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
    //     }   
    // }

    void dumpPreprocessingData(SharedEnvironment* env) {
    std::string prefix = "/tmp/" + env->map_name + "_";
    std::cerr << "[DUMP] writing to " << prefix << "..." << std::endl;

    // 1. Base map (obstacles)
    std::ofstream f1(prefix + "preprocess_map.csv");
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            f1 << env->map[row * env->cols + col];
            if (col < env->cols - 1) f1 << ",";
        }
        f1 << "\n";
    }
    f1.close();

    // 2. Voronoi map
    std::ofstream f2(prefix + "preprocess_voronoi.csv");
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            f2 << env->hpa_h.voronoi_map[row * env->cols + col];
            if (col < env->cols - 1) f2 << ",";
        }
        f2 << "\n";
    }
    f2.close();

    // 3. Gates map (1 = gate, 0 = not gate)
    std::unordered_set<int> gate_set(
        env->hpa_h.AG.gates.begin(),
        env->hpa_h.AG.gates.end()
    );
    std::ofstream f3(prefix + "preprocess_gates.csv");
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            int loc = row * env->cols + col;
            f3 << (gate_set.count(loc) ? 1 : 0);
            if (col < env->cols - 1) f3 << ",";
        }
        f3 << "\n";
    }
    f3.close();

    // 4. Highway edges (row1,col1,row2,col2)
    std::ofstream f4(prefix + "preprocess_highways.csv");
    for (auto& [from, to] : env->hpa_h.hw.e_hw) {
        int loc_from = from.first;
        int loc_to   = to.first;
        f4 << (loc_from / env->cols) << "," << (loc_from % env->cols) << ","
           << (loc_to   / env->cols) << "," << (loc_to   % env->cols) << "\n";
    }
    f4.close();

    // 5. Cluster centers (cluster_id, row, col) — first gate per cluster
    std::ofstream f5(prefix + "preprocess_cluster_centers.csv");
    for (int c = 0; c < env->k; c++) {
        if (!env->hpa_h.Gates[c].empty()) {
            int loc = env->hpa_h.Gates[c][0];
            f5 << c << "," << (loc / env->cols) << "," << (loc % env->cols) << "\n";
        }
    }
    f5.close();

    std::cerr << "[DUMP] done. gates=" << gate_set.size()
              << " hw_edges=" << env->hpa_h.hw.e_hw.size() << std::endl;
}

    void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids) {
    auto t_total = std::chrono::high_resolution_clock::now();
    auto elapsed = [&](auto t) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - t).count();
    };

    auto t0 = std::chrono::high_resolution_clock::now();
    // CustomAlgo::init_neighbor(env);
    // std::cerr << "[HPA] Step 0 init_neighbor:        " << elapsed(t0) << "ms" << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    cluster_indexing(env);
    std::cerr << "[HPA] Step 1 cluster_indexing:     " << elapsed(t0) << "ms" << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    build_entrances(env);
    std::cerr << "[HPA] Step 2a build_entrances:     " << elapsed(t0) << "ms"
              << "  ents=" << env->hpa_h.Ents.size() << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    build_abstract_graph(env);
    std::cerr << "[HPA] Step 2b build_abstract(1st): " << elapsed(t0) << "ms"
              << "  gates=" << env->hpa_h.AG.gates.size() << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    build_InterHT(env);
    std::cerr << "[HPA] Step 2c build_InterHT(1st):  " << elapsed(t0) << "ms" << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    generateHighways(env, centroids);
    std::cerr << "[HPA] Step 3  generateHighways:    " << elapsed(t0) << "ms"
              << "  hw_edges=" << env->hpa_h.hw.e_hw.size() << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    build_abstract_graph(env);
    std::cerr << "[HPA] Step 4a build_abstract(2nd): " << elapsed(t0) << "ms" << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    build_InterHT(env);
    std::cerr << "[HPA] Step 4b build_InterHT(2nd):  " << elapsed(t0) << "ms" << std::endl;
    
    // t0 = std::chrono::high_resolution_clock::now();
    // warmup_inter_cache(env);
    // std::cerr << "[HPA] Step 5  warmup_inter_cache:  " << elapsed(t0) << "ms"
    //           << "  cache_size=" << env->hpa_h.inter_cache.size() << std::endl;
    
    t0 = std::chrono::high_resolution_clock::now();
    warmup_inter_cache(env);
    std::cerr << "[HPA] Step 5 warmup inter cache:  " << elapsed(t0) << "ms" << std::endl;
    

    t0 = std::chrono::high_resolution_clock::now();
dumpPreprocessingData(env);
std::cerr << "[HPA] Step 6 dump:             " << elapsed(t0) << "ms" << std::endl;

    std::cerr << "[HPA] TOTAL generate_HPAHMap:      " << elapsed(t_total) << "ms" << std::endl;
    }
}