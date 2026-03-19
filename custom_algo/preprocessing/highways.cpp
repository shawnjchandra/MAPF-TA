#include "highway.h"
#include "heuristics.h"
#include "utils.h"

namespace CustomAlgo{
    void generateHighways(SharedEnvironment* env,  std::vector<int> centroids) {
        std::vector<std::pair<int,int>> path;
        Highways hw;

        std::map<std::pair<int,int>,std::pair<int,int>> e_hw;
        std::map<std::pair<int,int>,std::pair<int,int>> r_e_hw;
        for (int c_i = 0 ; c_i < centroids.size() ; c_i++) {
            for (int c_j = 0 ; c_j < centroids.size() ; c_j++) {
                if (c_i  == c_j) continue;

                path = extractPathFromH_HW(centroids[c_i], centroids[c_j], env);

                for(int i = 0 ; i < path.size() - 1 ; i++) {
                    int curr_loc = path[i].first;
                    int curr_orientation = path[i].second;
                    int next_loc = path[i+1].first;
                    int next_orientation = path[i+1].second;

                    e_hw[{curr_loc,curr_orientation}] = {next_loc,next_orientation};

                    int curr_r_o = reverseOrientation(curr_orientation);
                    int next_r_o = reverseOrientation(next_orientation);

                    r_e_hw[{next_loc, next_r_o}] = {curr_loc, curr_r_o};
                }
            }
        }
        hw.e_hw = e_hw;
        hw.r_e_hw = r_e_hw;

        env->hpa_h.hw = hw;
    }

    std::vector<std::pair<int,int>> extractPathFromH_HW(int start,int goal, SharedEnvironment* env) {
        std::vector<std::pair<int,int>> path;
        std::vector<int> Neighbors;
        int best_next_orient;
        int best_next;
        int best_cost;
        
        int best_orientation = -1;
        int current = start;

        int best = INTERVAL_MAX;
        for (int orient = 0 ; orient < 4 ; orient++) {
            int curr = min(best, query_heuristic(env, current, orient, goal));
            if (curr < best) {
                best = curr;
                best_orientation = orient;
            }    
        }

        path.push_back({current, best_orientation});
        while (current != goal )
        {
            best_next = INTERVAL_MAX;
            best_cost = INTERVAL_MAX;
            best_next_orient = -1;
            
            CustomAlgo::getNeighborLocs(&(env->ns), Neighbors, current);
            for (int neigh : Neighbors) {
                if (env->map[neigh] == 1) continue;
                
                best = INTERVAL_MAX;
                for (int next_orient = 0 ; next_orient < 4 ; next_orient++) {
                    best = query_heuristic(env, neigh, next_orient, goal);
                    if (best < best_cost) {
                        best_cost = best;
                        best_next = neigh;
                        best_next_orient = next_orient;
                    }
                }
            }

            path.push_back({best_next, best_next_orient});
        }
        return path;
    }

    void reverseHighways(SharedEnvironment* env) {
        auto temp = env->hpa_h.hw.e_hw;
        env->hpa_h.hw.e_hw = env->hpa_h.hw.r_e_hw;
        env->hpa_h.hw.r_e_hw = temp;
    }
}