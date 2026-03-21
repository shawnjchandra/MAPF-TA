#include "hpa.h"
#include <fstream>

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

    std::vector<std::array<int, 4>> build_IntraHT(SharedEnvironment* env, int cluster, int dest){
        priority_queue<HNode> pq;
        std::vector<std::array<int, 4>> ht;

        int cluster_size = env->hpa_h.local_to_global[cluster].size();
        int dest_local = env->hpa_h.global_to_local[dest];

        ht.resize(cluster_size);
        ht.assign(cluster_size, {INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX});

        for (int orient = 0; orient < 4 ; orient++ ){
            ht[dest_local][orient] = 0;
            pq.push(HNode(dest,orient,0));
        }
        
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
                    pq.push(HNode(curr.location,orient, newCost));
                }
            }
            int prevLoc = getBackwardLocation(curr.location,curr.direction,env);

            if (prevLoc == -1 || env->hpa_h.voronoi_map[prevLoc] != cluster) continue;
            int prevLoc_local = env->hpa_h.global_to_local[prevLoc];
            int moveCost = 1;

          

            if (env->hpa_h.hw.r_e_hw.count({prevLoc, curr.direction})){
                moveCost = env->c_penalty;
            }

            int newCost = curr.value + moveCost;
            if (newCost < ht[prevLoc_local][curr.direction]) {
                ht[prevLoc_local][curr.direction] = newCost;
                pq.push(HNode(prevLoc, curr.direction, newCost));
            }
        }

        return ht;
    };  

    // void build_abstract_graph(SharedEnvironment* env) {
    //     int k = env->k;

    //     // Clear in dulu karena akan dibuat 2 kali (non highway + highway)
    //     env->hpa_h.IntraHT.clear();      
    //     env->hpa_h.AG.gates.clear();     
    //     env->hpa_h.AG.intra.clear();     
    //     env->hpa_h.AG.inter.clear();     
    //     env->hpa_h.AG.neighbors.clear();

    //     env->hpa_h.IntraHT.resize(k);

        
    //     for (int c = 0; c < k ; c++){
    //         for( int gate_loc : env->hpa_h.Gates[c]){
    //             env->hpa_h.AG.gates.push_back(gate_loc);
    //         }
    //     }
        
    //     for(const Entrances e : env->hpa_h.Ents) {  //Append Entrances (cluster bersebelahan) ke AG
    //         env->hpa_h.AG.inter[{e.loc_a,e.neigh}] = 1;
    //         env->hpa_h.AG.inter[{e.neigh,e.loc_a}] = 1;
    //         env->hpa_h.AG.neighbors[e.loc_a].push_back(e.neigh);
    //         env->hpa_h.AG.neighbors[e.neigh].push_back(e.loc_a);
    //     }
        
    //     std::vector<std::array<int, 4>> ht;
    //     for (int c = 0 ; c < k ; c++) { //IntraHT
    //         int cluster_size = env->hpa_h.local_to_global[c].size();

    //         env->hpa_h.IntraHT[c].assign(cluster_size, std::vector<std::array<int,4>>(cluster_size,{INTERVAL_MAX, INTERVAL_MAX, INTERVAL_MAX, INTERVAL_MAX}));
            
    //         // for (int dest_local = 0 ; dest_local < cluster_size ; dest_local++){
    //         for (int dest : env->hpa_h.Gates[c]){ //Ganti sementara ,dari (untuk setiap cell) jadi (untuk setiap gate) saja

    //             // int dest = env->hpa_h.local_to_global[c][dest_local];
    //             int dest_local = env->hpa_h.global_to_local[dest];
                
    //             ht = build_IntraHT(env, c, dest);

    //             for (int src_local = 0 ; src_local < cluster_size; src_local++) {
    //                 for (int orient = 0 ; orient < 4 ; orient++) {
    //                     env->hpa_h.IntraHT[c][dest_local][src_local][orient] = ht[src_local][orient];
    //                 }
    //             }

    //             if (std::find(env->hpa_h.Gates[c].begin(), env->hpa_h.Gates[c].end(), dest) != env->hpa_h.Gates[c].end()) { //Intra Edges (Antar Gate dalam cluster yang sama)
                    
    //                 for (int g_src : env->hpa_h.Gates[c]){
                        
    //                     if (g_src == dest) continue;

    //                     int g_src_local = env->hpa_h.global_to_local[g_src];
    //                     int best_cost = INTERVAL_MAX;
    //                     for ( int orient = 0 ; orient < 4 ; orient++ ) {
    //                         best_cost = min(best_cost, ht[g_src_local][orient]);
    //                     }

    //                     if (best_cost != INTERVAL_MAX) {
    //                         env->hpa_h.AG.intra[{g_src,dest}] = best_cost;
    //                         env->hpa_h.AG.neighbors[g_src].push_back(dest);
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    void build_abstract_graph(SharedEnvironment* env) {
        auto t_start = std::chrono::high_resolution_clock::now();
        auto elapsed = [&](auto t) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - t).count();
        };

        int k = env->k;

        env->hpa_h.IntraHT.clear();      
        // env->hpa_h.AG.gates.clear();     
        env->hpa_h.AG.intra.clear();     
        env->hpa_h.AG.inter.clear();     
        env->hpa_h.AG.neighbors.clear();
        env->hpa_h.IntraHT.resize(k);

        auto t0 = std::chrono::high_resolution_clock::now();
        if (env->hpa_h.AG.gates.empty())
            for (int c = 0; c < k; c++)
                for (int gate_loc : env->hpa_h.Gates[c])
                    env->hpa_h.AG.gates.push_back(gate_loc);
        
        std::cerr << "  [BAG] gates push: " << elapsed(t0) << "ms" << std::endl;

        t0 = std::chrono::high_resolution_clock::now();
        for (const Entrances e : env->hpa_h.Ents) {
            env->hpa_h.AG.inter[{e.loc_a, e.neigh}] = 1;
            env->hpa_h.AG.inter[{e.neigh, e.loc_a}] = 1;
            env->hpa_h.AG.neighbors[e.loc_a].push_back(e.neigh);
            env->hpa_h.AG.neighbors[e.neigh].push_back(e.loc_a);
        }
        std::cerr << "  [BAG] inter edges: " << elapsed(t0) << "ms" << std::endl;

        std::vector<std::array<int, 4>> ht;
        for (int c = 0; c < k; c++) {
            auto t_cluster = std::chrono::high_resolution_clock::now();
            int cluster_size = env->hpa_h.local_to_global[c].size();
            int num_gates = env->hpa_h.Gates[c].size();

            // env->hpa_h.IntraHT[c].assign(
            //     cluster_size,
            //     std::vector<std::array<int,4>>(
            //         cluster_size,
            //         {INTERVAL_MAX, INTERVAL_MAX, INTERVAL_MAX, INTERVAL_MAX}
            //     )
            // );

            std::cerr << "  [BAG] cluster=" << c 
                    << " size=" << cluster_size 
                    << " gates=" << num_gates << std::endl;

            long intra_ht_ms = 0, intra_edge_ms = 0;

            for (int dest : env->hpa_h.Gates[c]) {
                auto t_ht = std::chrono::high_resolution_clock::now();

                int dest_local = env->hpa_h.global_to_local[dest];
                ht = build_IntraHT(env, c, dest);

                env->hpa_h.IntraHT[c][dest_local] = std::vector<std::array<int,4>>(ht.begin(), ht.end());

                // intra_ht_ms += elapsed(t_ht);

                // // Check dest_local is valid
                // if (dest_local < 0 || dest_local >= cluster_size) {
                //     std::cerr << "  [BAG] ERROR dest_local=" << dest_local 
                //             << " out of bounds cluster_size=" << cluster_size << std::endl;
                //     continue;
                // }

                
                // for (int src_local = 0; src_local < cluster_size; src_local++)
                //     for (int orient = 0; orient < 4; orient++)
                //         env->hpa_h.IntraHT[c][dest_local][src_local][orient] = ht[src_local][orient];

                // auto t_edge = std::chrono::high_resolution_clock::now();
                // for (int g_src : env->hpa_h.Gates[c]) {
                //     if (g_src == dest) continue;

                //     int g_src_local = env->hpa_h.global_to_local[g_src];

                //     // Guard against out of bounds
                //     if (g_src_local < 0 || g_src_local >= cluster_size) {
                //         std::cerr << "  [BAG] ERROR g_src_local=" << g_src_local
                //                 << " out of bounds cluster_size=" << cluster_size << std::endl;
                //         continue;
                //     }

                //     int best_cost = INTERVAL_MAX;
                //     for (int orient = 0; orient < 4; orient++)
                //         best_cost = min(best_cost, ht[g_src_local][orient]);

                //     if (best_cost != INTERVAL_MAX) {
                //         env->hpa_h.AG.intra[{g_src, dest}] = best_cost;
                //         env->hpa_h.AG.neighbors[g_src].push_back(dest);
                //     }
                // }
                // intra_edge_ms += elapsed(t_edge);
            }

            // std::cerr << "  [BAG] cluster=" << c 
            //         << " intra_ht=" << intra_ht_ms << "ms"
            //         << " intra_edges=" << intra_edge_ms << "ms"
            //         << " total=" << elapsed(t_cluster) << "ms" << std::endl;
        }

        std::cerr << "  [BAG] TOTAL: " << elapsed(t_start) << "ms" << std::endl;
    }

    // void build_InterHT(SharedEnvironment* env ) {
    //     std::vector<int> dist;
    //     std::vector<int> gates_index;
    //     std::vector<std::vector<int>> InterHT;
    //     priority_queue<HNode> pq;
    //     int i, w;
        
    //     gates_index.resize(env->map.size());
    //     env->hpa_h.gate_index.resize(env->map.size(),INTERVAL_MAX);
        
    //     int gates_size = env->hpa_h.AG.gates.size();
    //     for (i = 0 ; i < gates_size ; i++) {
    //         gates_index[env->hpa_h.AG.gates[i]] = i;
    //     }

    //     InterHT.resize(env->map.size());
    //     InterHT.assign(env->map.size(), std::vector<int>{INTERVAL_MAX});    
    //     env->hpa_h.InterHT.clear();
    //     env->hpa_h.InterHT.resize(gates_size, std::vector<int>(gates_size, INTERVAL_MAX));
        
    //     for ( i = 0 ; i < gates_size; i++) {
    //         int g_src = env->hpa_h.AG.gates[i];
    //         dist.assign(gates_size, INTERVAL_MAX);
    //         dist[i] = 0 ;

    //         pq = priority_queue<HNode>();
    //         pq.push(HNode::createForInterHT(0,i));
    //         while (!pq.empty())
    //         {
    //             HNode curr = pq.top();
    //             pq.pop();
    //             int d = curr.value;
    //             int u = curr.idx;

    //             if (d > dist[u]) continue;
                
    //             int u_loc = env->hpa_h.AG.gates[u];
    //             for (int v_loc : env->hpa_h.AG.neighbors[u_loc] ) {
    //                 if (env->hpa_h.AG.inter.find({u_loc, v_loc}) !=  env->hpa_h.AG.inter.end()) {
                        
    //                     w = env->hpa_h.AG.inter[{u_loc, v_loc}];
    //                     int orient = getOrientationBetween(u_loc,v_loc);
                        
    //                     if (env->hpa_h.hw.r_e_hw.count({u_loc,orient})) w = env->c_penalty;
    //                 }
    //                 else {
    //                     w = env->hpa_h.AG.intra[{u_loc, v_loc}];
    //                 }
                    
    //                 int v = gates_index[v_loc];
    //                 int neighDist = dist[u] + w;

    //                 if (neighDist < dist[v]) {
    //                     dist[v] = neighDist;
    //                     pq.push(HNode::createForInterHT(neighDist,v));
    //                 }

    //             }
    //         }
    //         env->hpa_h.InterHT[i] = dist;
    //     }
    //    env->hpa_h.gate_index = gates_index;
    // }

    void build_InterHT(SharedEnvironment* env) {
        auto t_start = std::chrono::high_resolution_clock::now();
        auto elapsed = [&](auto t) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - t).count();
        };

        int gates_size = env->hpa_h.AG.gates.size();

        // Build gate_index only on first call
        if (env->hpa_h.gate_index.empty() ||
            env->hpa_h.gate_index[env->hpa_h.AG.gates[0]] == INTERVAL_MAX) {
            env->hpa_h.gate_index.assign(env->map.size(), INTERVAL_MAX);
            for (int i = 0; i < gates_size; i++)
                env->hpa_h.gate_index[env->hpa_h.AG.gates[i]] = i;
            std::cerr << "  [IHT] gate_index built" << std::endl;
        } else {
            std::cerr << "  [IHT] gate_index reused" << std::endl;
        }

        // Always clear cache since highway costs may have changed
        env->hpa_h.inter_cache.clear();

        std::cerr << "  [IHT] lazy mode, gates=" << gates_size
                << "  TOTAL=" << elapsed(t_start) << "ms" << std::endl;
    }

    void compute_inter_from(SharedEnvironment* env, int gate_idx) {
    int gates_size = env->hpa_h.AG.gates.size();
    std::vector<int> dist(gates_size, INTERVAL_MAX);
    dist[gate_idx] = 0;

    std::priority_queue<HNode> pq;
    pq.push(HNode::createForInterHT(0, gate_idx));

    while (!pq.empty()) {
        HNode curr = pq.top();
        pq.pop();
        int d = curr.value;
        int u = curr.idx;

        if (d > dist[u]) continue;

        int u_loc = env->hpa_h.AG.gates[u];
        for (int v_loc : env->hpa_h.AG.neighbors[u_loc]) {
            int w;
            if (env->hpa_h.AG.inter.count({u_loc, v_loc})) {
                w = env->hpa_h.AG.inter[{u_loc, v_loc}];
                int orient = getOrientationBetween(u_loc, v_loc);
                if (env->hpa_h.hw.r_e_hw.count({u_loc, orient}))
                    w = env->c_penalty;
            } else {
                w = env->hpa_h.AG.intra[{u_loc, v_loc}];
            }

            int v = env->hpa_h.gate_index[v_loc];
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


void dumpPreprocessingData(SharedEnvironment* env) {
    std::string prefix = "/tmp/" + env->map_name + "_";
    
    std::cerr << "[PREPROCESS] dump_starting path=" << prefix << std::endl;

    // Dump base map
    std::ofstream f1(prefix + "preprocess_map.csv");
    if (!f1.is_open()) { std::cerr << "[PREPROCESS] ERROR: cannot open map csv" << std::endl; return; }
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            f1 << env->map[row * env->cols + col];
            if (col < env->cols - 1) f1 << ",";
        }
        f1 << "\n";
    }
    f1.close();

    // Dump voronoi map
    std::ofstream f2(prefix + "preprocess_voronoi.csv");
    if (!f2.is_open()) { std::cerr << "[PREPROCESS] ERROR: cannot open voronoi csv" << std::endl; return; }
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            f2 << env->hpa_h.voronoi_map[row * env->cols + col];
            if (col < env->cols - 1) f2 << ",";
        }
        f2 << "\n";
    }
    f2.close();

    // Dump gates
    std::unordered_set<int> gate_set(
        env->hpa_h.AG.gates.begin(),
        env->hpa_h.AG.gates.end()
    );
    std::ofstream f3(prefix + "preprocess_gates.csv");
    if (!f3.is_open()) { std::cerr << "[PREPROCESS] ERROR: cannot open gates csv" << std::endl; return; }
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            int loc = row * env->cols + col;
            f3 << (gate_set.count(loc) ? 1 : 0);
            if (col < env->cols - 1) f3 << ",";
        }
        f3 << "\n";
    }
    f3.close();

    // Dump highway edges
    std::ofstream f4(prefix + "preprocess_highways.csv");
    if (!f4.is_open()) { std::cerr << "[PREPROCESS] ERROR: cannot open highways csv" << std::endl; return; }
    for (auto& [from, to] : env->hpa_h.hw.e_hw) {
        int loc_from = from.first;
        int loc_to   = to.first;
        f4 << (loc_from / env->cols) << "," << (loc_from % env->cols) << ","
           << (loc_to   / env->cols) << "," << (loc_to   % env->cols) << "\n";
    }
    f4.close();

    // Dump cluster centers (first gate per cluster)
    std::ofstream f5(prefix + "preprocess_cluster_centers.csv");
    if (!f5.is_open()) { std::cerr << "[PREPROCESS] ERROR: cannot open centers csv" << std::endl; return; }
    for (int c = 0; c < env->k; c++) {
        if (!env->hpa_h.Gates[c].empty()) {
            int loc = env->hpa_h.Gates[c][0];
            f5 << c << "," << (loc / env->cols) << "," << (loc % env->cols) << "\n";
        }
    }
    f5.close();

    std::cerr << "[PREPROCESS] dump_done=1" << std::endl;
}

    // void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids) {

    //     std::cerr << "Mulai generate" << std::endl;
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
        
    //     std::cerr << "Mulai Dumping" << std::endl;
    //     dumpPreprocessingData(env);
    //     std::cerr << "Selesai Dumping" << std::endl;
    // }

    void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids) {
        std::cerr << "Step 0: init_neighbor" << std::endl;
        CustomAlgo::init_neighbor(env);

        std::cerr << "Step 1: cluster_indexing" << std::endl;
        cluster_indexing(env);
        
        std::cerr << "Step 2: build_entrances" << std::endl;
        build_entrances(env);
        
        std::cerr << "Step 3: build_abstract_graph (1st)" << std::endl;
        build_abstract_graph(env);
        
        std::cerr << "Step 4: build_InterHT (1st)" << std::endl;
        build_InterHT(env);
        
        int gates_size = env->hpa_h.AG.gates.size();
        std::cerr << "gates_size=" << gates_size << std::endl;
        std::cerr << "gate_index built=" << !env->hpa_h.gate_index.empty() << std::endl;
        std::cerr << "inter_cache size=" << env->hpa_h.inter_cache.size() 
                << " (lazy, 0 is expected)" << std::endl;

        if (gates_size > 1) {
            compute_inter_from(env, 0);  // compute from first gate
            std::cerr << "inter_cache[0][1]=" << env->hpa_h.inter_cache[0][1] << std::endl;
            env->hpa_h.inter_cache.clear();  // clear test, let it build lazily during scheduling
        }

        int non_inf = 0;
        for (int i = 0; i < (int)env->hpa_h.InterHT.size(); i++)
            for (int j = 0; j < (int)env->hpa_h.InterHT[i].size(); j++)
                if (env->hpa_h.InterHT[i][j] != INTERVAL_MAX) non_inf++;
        std::cerr << "Step 5: generateHighways" << std::endl;

        generateHighways(env, centroids);

        std::cerr << "InterHT non-INTERVAL_MAX entries=" << non_inf << std::endl;
        
        std::cerr << "Step 6: build_abstract_graph (2nd)" << std::endl;
        build_abstract_graph(env);
        
        std::cerr << "Step 7: build_InterHT (2nd)" << std::endl;
        build_InterHT(env);
        
        std::cerr << "Step 8: dumping" << std::endl;
        dumpPreprocessingData(env);
        std::cerr << "Step 9: done" << std::endl;
    }   
    }