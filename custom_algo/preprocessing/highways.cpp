#include "highway.h"

namespace CustomAlgo{
    // void generateHighways(SharedEnvironment* env,  std::vector<int> centroids) {
    //     std::vector<std::pair<int,int>> path;
    //     Highways hw;

    //     std::map<std::pair<int,int>,std::pair<int,int>> e_hw;
    //     std::map<std::pair<int,int>,std::pair<int,int>> r_e_hw;

    //     for (int c_i = 0 ; c_i < centroids.size() ; c_i++) {
    //         for (int c_j = 0 ; c_j < centroids.size() ; c_j++) {
    //             if (c_i  == c_j) continue;

    //             path = extractPathFromH_HW(centroids[c_i], centroids[c_j], env);

    //             for(int i = 0 ; i < path.size() - 1 ; i++) {
    //                 int curr_loc = path[i].first;
    //                 int curr_orientation = path[i].second;
    //                 int next_loc = path[i+1].first;
    //                 int next_orientation = path[i+1].second;

    //                 e_hw[{curr_loc,curr_orientation}] = {next_loc,next_orientation};

    //                 int curr_r_o = reverseOrientation(curr_orientation);
    //                 int next_r_o = reverseOrientation(next_orientation);

    //                 r_e_hw[{next_loc, next_r_o}] = {curr_loc, curr_r_o};
    //             }
    //         }
    //     }
    //     hw.e_hw = e_hw;
    //     hw.r_e_hw = r_e_hw;

    //     env->hpa_h.hw = hw;
    // }

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
        if (done >= env->max_hw) break;
        int c_a = e.c_a;
        int c_b = e.c_b;

        std::pair<int,int> key = std::make_pair(min(c_a, c_b), max(c_a, c_b));
        if (done_pairs.count(key)) continue;
        done_pairs.insert(key);

        
        if (c_a >= (int)centroids.size() || c_b >= (int)centroids.size()) continue;
        int start = centroids[c_a];
        int goal  = centroids[c_b];
        if (start == goal) continue;

        path = extractPathFromH_HW(start, goal, env, prev, visited);
        done++;

        if (path.empty()) continue;

        for (int i = 0; i < (int)path.size() - 1; i++) {
            int curr_loc         = path[i].first;
            int curr_orientation = path[i].second;
            int next_loc         = path[i+1].first;
            int next_orientation = path[i+1].second;

            env->hpa_h.hw.e_hw[{curr_loc, curr_orientation}] =
                {next_loc, next_orientation};

            int curr_r_o = reverseOrientation(curr_orientation);
            int next_r_o = reverseOrientation(next_orientation);
            env->hpa_h.hw.r_e_hw[{next_loc, next_r_o}] =
                {curr_loc, curr_r_o};
        }
    }

}


    std::vector<std::pair<int,int>> extractPathFromH_HW(int start, int goal, SharedEnvironment* env, std::vector<int> prev,  std::vector<bool> visited) {
        if (start == goal) return {{start, 0}};
        
        std::vector<std::pair<int,int>> path;
        std::vector<int> Neighbors;
        std::queue<int> q;

        

        q.push(start);
        visited[start] = true;

        while (!q.empty()) {
            int cur = q.front(); q.pop();
            if (cur == goal) break;

            CustomAlgo::getNeighborLocs(&(env->ns), Neighbors, cur);
            for (int neigh : Neighbors) {
                if (env->map[neigh] == 1 || visited[neigh]) continue; 
                visited[neigh] = true;
                prev[neigh] = cur;
                q.push(neigh);
            }
        }

        if (prev[goal] == -1 && start != goal) {
            return {};
        }

        int cur = goal;
        while (cur != start) {
            int p = prev[cur];
            if (p == -1) {
                return {};
            }
            int orient = getOrientationBetween(p, cur);
            path.push_back({cur, orient});
            cur = p;
        }
        path.push_back({start, path.empty() ? 0 : path.back().second});
        std::reverse(path.begin(), path.end());

        return path;
    }

    void reverseHighways(SharedEnvironment* env) {
        auto temp = env->hpa_h.hw.e_hw;
        env->hpa_h.hw.e_hw = env->hpa_h.hw.r_e_hw;
        env->hpa_h.hw.r_e_hw = temp;
    }
}