#include "highway.h"

namespace CustomAlgo{

    /**
     * @brief Buat highway (penaltized one-way directed edges) untuk setiap centroid (sejumlah limitNumHW)
     * 
     * @param env 
     * @param centroids 
     */
    void generateHighways(SharedEnvironment* env, std::vector<int> centroids) {
        env->hpa_h.hw.e_hw.clear();
        env->hpa_h.hw.r_e_hw.clear();

        std::vector<int> prev;
        prev.resize(env->map.size(), -1);
        std::vector<bool> visited;
        visited.resize(env->map.size(), false);

        std::set<std::pair<int,int>> done_pairs;
        std::vector<std::pair<int,int>> path;

        int done = 0;

        for (const Entrances& e : env->hpa_h.Ents) {

            // Batas jumlah highway
            if (done >= env->max_hw) break;

            int c_a = e.c_a;
            int c_b = e.c_b;

            std::pair<int,int> key = std::make_pair(min(c_a, c_b), max(c_a, c_b));
            if (done_pairs.count(key)) continue;
            done_pairs.insert(key);

            int start = centroids[c_a];
            int goal  = centroids[c_b];
            if (start == goal) continue;

            path = extractPathFromH_HW(start, goal, env, prev, visited);
            done++;

            if (path.empty()) continue;

            //Catat untuk setiap edges pada path
            for (int i = 0; i <  path.size() - 1; i++) {
                int curr_loc         = path[i].first;
                int curr_orientation = path[i].second;
                int next_loc         = path[i+1].first;
                int next_orientation = path[i+1].second;

                //Arah dan jalan yang TIDAK dipenalty
                env->hpa_h.hw.e_hw[{curr_loc, curr_orientation}] =
                    {next_loc, next_orientation};

                int curr_r_o = reverseOrientation(curr_orientation);
                int next_r_o = reverseOrientation(next_orientation);
                
                //Arah dan jalur yang dipenalty
                env->hpa_h.hw.r_e_hw[{next_loc, next_r_o}] =
                    {curr_loc, curr_r_o};
            }
        }
    }


    /**
     * @brief BFS dari lokasi destination (start) ke source (goal) dan reverse ketika telah selesai. Tidak mengutamakan cost.
     * 
     * @param start 
     * @param goal 
     * @param env 
     * @param prev 
     * @param visited 
     * @return std::vector<std::pair<int,int>> 
     */
    std::vector<std::pair<int,int>> extractPathFromH_HW(int start, int goal, SharedEnvironment* env, std::vector<int> prev,  std::vector<bool> visited) {
        if (start == goal) return {{start, 0}};
        
        std::vector<std::pair<int,int>> path;
        std::vector<int> Neighbors;
        std::queue<int> q;

        q.push(start);
        visited[start] = true;

        // Data seluruh neighbor dan jalur dari destination ke source ke dalam array prev
        while (!q.empty()) {
            int cur = q.front(); 
            q.pop();
            if (cur == goal) break;

            CustomAlgo::getNeighborLocs(&(env->ns), Neighbors, cur);
            for (int neigh : Neighbors) {
                if (env->map[neigh] == 1 || visited[neigh]) continue; 
                visited[neigh] = true;
                prev[neigh] = cur;
                q.push(neigh);
            }
        }

        if (prev[goal] == -1) {
            return {};
        }

        int cur = goal;

        // Backtrace goal -> start
        while (cur != start) {
            int p = prev[cur];
            if (p == -1) {
                return {};
            }
            int orient = getOrientationBetween(p, cur,env->cols);
            path.push_back({cur, orient});
            cur = p;
        }
        path.push_back({start, path.empty() ? 0 : path.back().second});
        std::reverse(path.begin(), path.end());

        return path;
    }

    //Putar balikan jalur penalty
    void reverseHighways(SharedEnvironment* env) {
        auto temp = env->hpa_h.hw.e_hw;
        env->hpa_h.hw.e_hw = env->hpa_h.hw.r_e_hw;
        env->hpa_h.hw.r_e_hw = temp;
    }
}