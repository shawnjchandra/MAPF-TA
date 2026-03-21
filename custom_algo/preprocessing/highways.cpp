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

    auto t0 = std::chrono::high_resolution_clock::now();
    auto elapsed = [&](auto t) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - t).count();
    };

    // Only connect centroids of neighboring clusters (have shared entrance)
    // Use set to avoid duplicate pairs
    std::set<std::pair<int,int>> done_pairs;

    std::vector<std::pair<int,int>> path;
    int done = 0;

    for (const Entrances& e : env->hpa_h.Ents) {
        int c_a = e.c_a;
        int c_b = e.c_b;

        // Skip if already processed this cluster pair
        auto key = std::make_pair(min(c_a, c_b), max(c_a, c_b));
        if (done_pairs.count(key)) continue;
        done_pairs.insert(key);

        // Get centroid for each cluster
        if (c_a >= (int)centroids.size() || c_b >= (int)centroids.size()) continue;
        int start = centroids[c_a];
        int goal  = centroids[c_b];
        if (start == goal) continue;

        path = extractPathFromH_HW(start, goal, env);
        done++;

        if (done % 50 == 0)
            std::cerr << "  [HW] progress=" << done 
                      << "/" << done_pairs.size()
                      << " elapsed=" << elapsed(t0) << "ms" << std::endl;

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

    std::cerr << "[HW] DONE pairs=" << done_pairs.size()
              << " edges=" << env->hpa_h.hw.e_hw.size()
              << " total=" << elapsed(t0) << "ms" << std::endl;
}


    std::vector<std::pair<int,int>> extractPathFromH_HW(int start, int goal, SharedEnvironment* env) {
        if (start == goal) return {{start, 0}};

        auto t0 = std::chrono::high_resolution_clock::now();
        auto elapsed = [&](auto t) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - t).count();
        };

        std::vector<int> prev(env->map.size(), -1);
        std::vector<bool> visited(env->map.size(), false);
        std::queue<int> q;
        std::vector<int> Neighbors;

        q.push(start);
        visited[start] = true;

        int steps = 0;
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            steps++;

            if (cur == goal) break;

            CustomAlgo::getNeighborLocs(&(env->ns), Neighbors, cur);
            for (int neigh : Neighbors) {
                if (env->map[neigh] == 1) continue;
                if (visited[neigh]) continue;
                visited[neigh] = true;
                prev[neigh] = cur;
                q.push(neigh);
            }
        }

        long bfs_ms = elapsed(t0);

        if (prev[goal] == -1 && start != goal) {
            std::cerr << "  [PATH] start=" << start << " goal=" << goal 
                    << " UNREACHABLE steps=" << steps 
                    << " time=" << bfs_ms << "ms" << std::endl;
            return {};
        }

        // Reconstruct path
        auto t1 = std::chrono::high_resolution_clock::now();
        std::vector<std::pair<int,int>> path;
        int cur = goal;
        while (cur != start) {
            int p = prev[cur];
            if (p == -1) {
                std::cerr << "  [PATH] ERROR broken path at cur=" << cur << std::endl;
                return {};
            }
            int orient = getOrientationBetween(p, cur);
            path.push_back({cur, orient});
            cur = p;
        }
        path.push_back({start, path.empty() ? 0 : path.back().second});
        std::reverse(path.begin(), path.end());

        std::cerr << "  [PATH] start=" << start << " goal=" << goal
                << " path_len=" << path.size()
                << " bfs=" << bfs_ms << "ms"
                << " reconstruct=" << elapsed(t1) << "ms" << std::endl;

        return path;
    }

    void reverseHighways(SharedEnvironment* env) {
        auto temp = env->hpa_h.hw.e_hw;
        env->hpa_h.hw.e_hw = env->hpa_h.hw.r_e_hw;
        env->hpa_h.hw.r_e_hw = temp;
    }
}