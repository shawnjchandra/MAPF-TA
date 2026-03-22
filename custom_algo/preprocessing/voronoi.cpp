#include "voronoi.h"

namespace CustomAlgo{
    
   void bfs_update(int src, std::vector<int>& min_dist, int map_size, SharedEnvironment* env) {
        std::queue<int> q;
        std::vector<int> Neighbors;
        std::vector<int> bfs_dist;
        bfs_dist.resize(map_size, INTERVAL_MAX);
        
        q.push(src);
        bfs_dist[src] = 0;
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            CustomAlgo::getNeighborLocs(&(env->ns), Neighbors, cur);
            for (int nb : Neighbors) {
                if (env->map[nb] == 1 || bfs_dist[nb] != INTERVAL_MAX) continue; // Obstacle atau udah pernah diisi

                // Update dan masuk ke queue kalau sebelumnya belum pernahb 
                bfs_dist[nb] = bfs_dist[cur] + 1;
                q.push(nb);
            }
        }

        // Update global min_dist
        for (int i = 0; i < map_size; i++) {
            if (bfs_dist[i] < min_dist[i]) min_dist[i] = bfs_dist[i];
        }
    }

    std::vector<int> maximin_sampling(SharedEnvironment* env) {
        int map_size = env->map.size();
        
        std::vector<int> interest_points;
        std::vector<int> min_dist(map_size, INTERVAL_MAX);
        interest_points.reserve(env->k);
        
        int p;
        do { 
            p = CustomAlgo::rng(0, map_size - 1); } 
        while (env->map[p] == 1);
        interest_points.push_back(p);

        bfs_update(p,min_dist,map_size, env);

        for (int i = 1; i < env->k; i++) {
            int best_loc = -1;
            int best_dist = -1;
            for (int j = 0; j < map_size; j++) {
                if (env->map[j] == 1) continue;
                
                if (min_dist[j] > best_dist) {
                    best_dist = min_dist[j];
                    best_loc = j;
                }
            }

            interest_points.push_back(best_loc);
            bfs_update(best_loc, min_dist, map_size, env);  
        }

        return interest_points;
    }

    void voronoi_generation(SharedEnvironment* env, std::vector<int> centroids) {
        std::queue<int> q;
        std::vector<int> Neighbors;

        env->hpa_h.voronoi_map.resize(env->map.size(), -1);
        int map_size = env->map.size();
        std::vector<int> dist(map_size, INTERVAL_MAX);

        for (int k = 0; k < centroids.size(); k++) {
            int src = centroids[k];

            if (env->map[src] == 1) continue;
            
            dist[src] = 0;
            env->hpa_h.voronoi_map[src] = k;
            q.push(src);
        }

        while (!q.empty()) {  // FIFO-Based clustering / Multi source BFS
            int cur = q.front(); q.pop();
            CustomAlgo::getNeighborLocs(&(env->ns), Neighbors, cur);
            for (int nb : Neighbors) {
                if (env->map[nb] == 1 || dist[nb] != INTERVAL_MAX) continue;

                dist[nb] = dist[cur] + 1;
                env->hpa_h.voronoi_map[nb] = env->hpa_h.voronoi_map[cur];
                q.push(nb);
            }
        }

    }
}