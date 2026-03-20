#include "hpa.h"

namespace CustomAlgo{
    void cluster_indexing(SharedEnvironment* env) {    
        for(int loc = 0; loc < env->map.size() ; loc++){
            
            // Cek lokasi obstacle atau bukan
            int c = env->map[loc];
            if (c == -1) continue;

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
                    env->hpa_h.Gates[c_a].push_back(c_b);
                    env->hpa_h.Gates[c_b].push_back(c_a);
                }
            }
        }
    };

    std::vector<std::array<int, 4>> build_IntraHT(SharedEnvironment* env, int cluster, int dest){
        priority_queue<HNode> pq;
        std::vector<std::array<int, 4>> ht;

        int dest_local = env->hpa_h.global_to_local[dest];

        ht.resize(env->map.size());
        ht.assign(env->map.size(), {INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX,INTERVAL_MAX});

        for (int orient = 0; orient < 4 ; orient++ ){
            ht[dest_local][orient] = 0;
            pq.push(HNode(dest_local,orient,0));
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
                ht[prevLoc][curr.direction] = newCost;
                pq.push(HNode(prevLoc, curr.direction, newCost));
            }
        }

        return ht;
    };  

    void build_abstract_graph(SharedEnvironment* env) {
        int k = env->k;
        env->hpa_h.IntraHT.resize(k);
        
        for (int c = 0; c < k ; c++){
            for( int gate_loc : env->hpa_h.Gates[c]){
                env->hpa_h.AG.gates.push_back(gate_loc);
            }
        }
        
        for(const Entrances e : env->hpa_h.Ents) {  //Append Entrances (cluster bersebelahan) ke AG
            env->hpa_h.AG.inter[{e.loc_a,e.neigh}] = 1;
            env->hpa_h.AG.inter[{e.neigh,e.loc_a}] = 1;
            env->hpa_h.AG.neighbors[e.loc_a].push_back(e.neigh);
            env->hpa_h.AG.neighbors[e.neigh].push_back(e.loc_a);
        }
        
        std::vector<std::array<int, 4>> ht;
        for (int c = 0 ; c < k ; c++) { //IntraHT
            int cluster_size = env->hpa_h.local_to_global[c].size();

            env->hpa_h.IntraHT[c].assign(cluster_size, std::vector<std::array<int,4>>(cluster_size,{INTERVAL_MAX, INTERVAL_MAX, INTERVAL_MAX, INTERVAL_MAX}));
            
            for (int dest_local = 0 ; dest_local < cluster_size ; dest_local++){

                int dest = env->hpa_h.local_to_global[c][dest_local];
                
                ht = build_IntraHT(env, c, dest);

                for (int src_local = 0 ; src_local < cluster_size; src_local++) {
                    for (int orient = 0 ; orient < 4 ; orient++) {
                        env->hpa_h.IntraHT[c][dest_local][src_local][orient] = ht[src_local][orient];
                    }
                }

                if (std::find(env->hpa_h.Gates[c].begin(), env->hpa_h.Gates[c].end(), dest) != env->hpa_h.Gates[c].end()) { //Intra Edges (Antar Gate dalam cluster yang sama)
                    
                    for (int g_src = 0 ; g_src < env->hpa_h.Gates[c].size() ; g_src++ ){
                        
                        if (g_src == dest) continue;

                        int g_src_local = env->hpa_h.global_to_local[g_src];
                        int best_cost = INTERVAL_MAX;
                        for ( int orient = 0 ; orient < 4 ; orient++ ) {
                            best_cost = min(best_cost, ht[g_src_local][orient]);
                        }

                        if (best_cost != INTERVAL_MAX) {
                            env->hpa_h.AG.intra[{g_src,dest}] = best_cost;
                            env->hpa_h.AG.neighbors[g_src].push_back(dest);
                        }
                    }
                }
            }
        }
    }

    void build_InterHT(SharedEnvironment* env ) {
        std::vector<int> dist;
        std::vector<int> gates_index;
        std::vector<std::vector<int>> InterHT;
        priority_queue<HNode> pq;
        int i, w;
        
        gates_index.resize(env->map.size());
        
        InterHT.resize(env->map.size());
        InterHT.assign(env->map.size(), std::vector<int>{INTERVAL_MAX});    
                
        int gates_size = env->hpa_h.AG.gates.size();
        for (i = 0 ; i < gates_size ; i++) {
            gates_index[env->hpa_h.AG.gates[i]] = i;
        }

        for ( i = 0 ; i < gates_size; i++) {
            int g_src = env->hpa_h.AG.gates[i];
            dist.assign(gates_size, INTERVAL_MAX);
            dist[i] = 0 ;

            pq = priority_queue<HNode>();
            pq.push(HNode::createForInterHT(0,i));
            while (!pq.empty())
            {
                HNode curr = pq.top();
                pq.pop();
                int d = curr.value;
                int u = curr.idx;

                if (d > dist[u]) continue;
                
                int u_loc = env->hpa_h.AG.gates[u];
                for (int v_loc = 0 ; v_loc < env->hpa_h.AG.neighbors[u_loc].size() ; v_loc++) {
                    if (env->hpa_h.AG.inter.find({u_loc, v_loc}) !=  env->hpa_h.AG.inter.end()) {
                        
                        w = env->hpa_h.AG.inter[{u_loc, v_loc}];
                        int orient = getOrientationBetween(u_loc,v_loc);
                        
                        if (env->hpa_h.hw.r_e_hw.count({u_loc,orient})) w = env->c_penalty;
                    }
                    else {
                        w = env->hpa_h.AG.intra[{u_loc, v_loc}];
                    }
                    
                    int v = gates_index[v_loc];
                    int neighDist = dist[u] + w;

                    if (neighDist < dist[v]) {
                        dist[v] = neighDist;
                        pq.push(HNode::createForInterHT(neighDist,v));
                    }

                }
            }
            env->hpa_h.InterHT[i] = dist;
        }
       env->hpa_h.gate_index = gates_index;
    }

    void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids) {
        //Step 1 : Index semua cluster
        cluster_indexing(env);
        
        // Step 2 : Bikin HPA pertama
        build_entrances(env);
        build_abstract_graph(env);
        build_InterHT(env);
        
        // Step 3 : Generate Highways
        generateHighways(env, centroids);

        // Step 4 : HPA + Highways, Perubahan dari pseudocode: r_e_hw disimpan dalam env
        build_abstract_graph(env);
        build_InterHT(env);
    }
}