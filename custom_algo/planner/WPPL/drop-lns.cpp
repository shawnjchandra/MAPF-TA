#include "drop-lns.h"
#include "pibt.h"
#include "SharedEnv.h"
#include "heuristics.h"

namespace CustomAlgo {
    int compute_soc(
        std::vector<std::vector<State>>& planned,
        int w,
        std::vector<int>& goal_per_agent,
        SharedEnvironment* env
    ) {
        int soc = 0;
        int n = env->num_of_agents;
        for (int a = 0 ; a < n ; a++) {
            auto& curr_s = planned[w][a];
            if (curr_s.location >= 0 && curr_s.orientation >= 0 ) soc += query_heuristic(env, curr_s.location, curr_s.orientation, goal_per_agent[a]);
        }

        return soc;
    }

    std::vector<int> destroy_random(
        int num_agents,
        int N,
        std::mt19937 rng
    ) {
        std::vector<int> agents(num_agents);
        
        std::iota(agents.begin(), agents.end(),0);
        std::shuffle(agents.begin(), agents.end(), rng);

        // agent yang di destroy maksimal seukuran N  (10% seharusnya)
        agents.resize(std::min(N, num_agents));
        return agents;
    }

    std::vector<std::vector<State>> repair(
        std::vector<std::vector<State>>& current_plan,
        std::vector<int>& destroyed_agents,
        std::vector<int>& ids,
        std::vector<int>& goal_per_agent,
        const std::vector<double>& p,
        SharedEnvironment* env
    ) {
        auto& ps = env->planner_state;
        int w = ps.w;
        int n = env->num_of_agents;
        int map_size = env->map.size();

        std::unordered_set<int> destroyed(destroyed_agents.begin(), destroyed_agents.end());

        auto new_plan = current_plan;
        auto w_map = ps.wait_map;

        // mirip kayak pibt/wppl lagi untuk simulation
        std::vector<int> sim_prev_dec(map_size, -1);
        std::vector<int> sim_dec(map_size, -1);
        vector<bool> sim_occupied(map_size, false);

        vector<int> fxd_dest(map_size, -1);

        for (int t = 0 ; t < w ; t++) {
            vector<State>& prev_state = new_plan[t];
            vector<State> next_state(n, State());

            fill(sim_prev_dec.begin(), sim_prev_dec.end(), -1);
            fill(sim_dec.begin(), sim_dec.end(), -1);
            fill(sim_occupied.begin(), sim_occupied.end(), false);
            fill(fxd_dest.begin(), fxd_dest.end(), -1);

            for (int a = 0 ; a < n ; a++) {
                sim_prev_dec[prev_state[a].location] = a;
            }

            for (int a = 0 ; a < n ; a++) {
                if (destroyed.count(a)) continue;
                
                next_state[a] = current_plan[t+1][a];
                
                int next_loc = next_state[a].location;
                if (next_loc >= 0 ) sim_dec[next_loc] = a;

                //Tracking si cegah si swap conflict. Dari lokasi mana ke lokasi mana
                int curr_loc = prev_state[a].location;
                if (curr_loc >= 0 && next_loc >= 0 && curr_loc != next_loc) fxd_dest[curr_loc] = next_loc;
            
            }

            //Run PIBT untuk agent yang di destroy aja (dan belum punya tujuan) ,ikutin priority p
            for (int id : ids) {
                if (!destroyed.count(id)) continue;
                if (next_state[id].location != -1) continue;
                
                pibt(id, -1, prev_state, next_state, sim_prev_dec, sim_dec, sim_occupied, goal_per_agent, env, p);
            }

            //Cek ada destroyed agent yang nabrak agen diam tidak (swap conflict)
            for (int a : destroyed_agents) {
                int from = prev_state[a].location;
                int to = next_state[a].location;

                if (from >= 0 && to >= 0 && from != to && fxd_dest[to] == from) {
                    sim_dec[to] = -1;
                    next_state[a] = prev_state[a];

                    if (from >= 0) sim_dec[from] = a;
                }
            }

            new_plan[t+1] = next_state;
        }

        ps.wait_map = w_map;

        return new_plan;
    }

    void run_lns (
        WPPLState& wppl,
        SharedEnvironment* env,
        std::vector<int>& ids,
        std::vector<int>& goal_per_agent,
        const std::vector<double>& p,
        TimePoint deadline
    ) {
        int w = env->planner_state.w;
        int n = env->num_of_agents;

        // Neighbourhood 
        int N = max(1 , n / 10);

        //Mastiin beda" hasilnya untuk setiap timestep
        std::mt19937 rng(env->curr_timestep);

        int current_soc = compute_soc(wppl.wppl_plan, w, goal_per_agent, env);

        while ( std::chrono::steady_clock::now() < deadline) {
            auto destroyed = destroy_random(n, N, rng);

            auto new_plan = repair(wppl.wppl_plan, destroyed, ids, goal_per_agent, p, env);

            int new_soc = compute_soc(new_plan, w, goal_per_agent, env);
            if (new_soc < current_soc) {
                wppl.wppl_plan = new_plan;
                current_soc = new_soc;
            }
        }
    }
}