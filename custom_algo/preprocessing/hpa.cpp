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
     * @return std::vector<std::array<int, 4>> 
     */
    std::vector<std::array<int, 4>> build_IntraHT(SharedEnvironment* env, int cluster, int dest){

        //Untuk Dijkstra, pakai PQ
        priority_queue<HNode> pq;

        // Vektor yang berisikan [dest_local][src_local][orientasi]. Nanti akan dikembalikan untuk disimpan di vektor cluster
        std::vector<std::array<int, 4>> ht;

        int cluster_size = env->hpa_h.local_to_global[cluster].size();
        int dest_local = env->hpa_h.global_to_local[dest];

        ht.resize(cluster_size);
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
                    pq.push(HNode(curr.location,orient, newCost)); // Lokakasi global
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

        env->hpa_h.IntraHT.clear();      
        env->hpa_h.AG.intra.clear();     
        env->hpa_h.AG.inter.clear();     
        env->hpa_h.AG.neighbors.clear();
        env->hpa_h.IntraHT.resize(k);

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
            for (int i = 0; i < gates_size; i++)
                env->hpa_h.AG.gate_index[env->hpa_h.AG.gates[i]] = i;
        } 

        env->hpa_h.inter_cache.clear();
    }

    /**
     * @brief Dijkstra untuk InterHT. Mencari jalur terdekat antar gate
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

    /**
     * @brief Warmup inter cache. Membuat InterHT hanya untuk cluster yang ditempati oleh agen sejak pertama kali dimasukkan.
     * 
     * @param env 
     */
    void warmup_inter_cache(SharedEnvironment* env) {
        std::set<int> agent_clusters;
        for (int loc : env->agent_starts) {
            int c = env->hpa_h.voronoi_map[loc];
            agent_clusters.insert(c);
        }
        
        // Untuk setiap cluster yang ditempat, buat interHT nya
        for (int c : agent_clusters){
            for (int g : env->hpa_h.Gates[c]){
                int g_idx = env->hpa_h.AG.gate_index[g];
                if(g_idx == INTERVAL_MAX || env->hpa_h.inter_cache.count(g_idx)) continue;
                compute_inter_from(env, g_idx);
            }
        }
    }

    void generate_HPAHMap(SharedEnvironment* env, std::vector<int> centroids) {
        // Step 0 : Inisialisasi vector neighbor, menyimpan setiap lokasi neighbor dari setiap usable cell
        CustomAlgo::init_neighbor(env);

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
        
        warmup_inter_cache(env);
    }
}