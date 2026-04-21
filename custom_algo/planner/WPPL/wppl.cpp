#include "wppl.h"
#include "pibt.h"

namespace CustomAlgo {
    void wppl_init (WPPLState& wppl, SharedEnvironment* env) {
        int w = env->planner_state.w;
        int n = env->num_of_agents;
        wppl.wppl_plan.assign(w + 1, std::vector<State>(n, State()));
        wppl.steps_since_plan = 0;
    }
    
    void run_windowed_pibt (WPPLState& wppl,
        SharedEnvironment* env,
        const std::vector<int>& ids,
        const std::vector<int>& goal_per_agent,
        const std::vector<double>& p
    ) {
        auto& ps = env->planner_state;
        int w = env->planner_state.w;
        int n = env->num_of_agents;
        int map_size = env->map.size();

        if (wppl.wppl_plan.size() != w+1)  wppl.wppl_plan.assign(w+1 , std::vector<State>(n, State()));
        
        auto wait_map_sim = ps.wait_map;

        //Simulation buat w plan, ga bisa langsung taroh
        std::vector<int> sim_prev_dec(map_size, -1);
        std::vector<int> sim_dec(map_size, -1);
        std::vector<bool> sim_occupied(map_size, false);

        for (int t = 0 ; t < w ; t++) {
            auto& prev_state = wppl.wppl_plan[t];
            std::vector<State> next_state(n, State());

            //Reset ke awal untuk setiap timestep ,pastiin bersih
            std::fill(sim_prev_dec.begin(), sim_prev_dec.end(), -1);
            std::fill(sim_dec.begin(), sim_dec.end(), -1);
            std::fill(sim_occupied.begin(), sim_occupied.end(), false);

            // Simpan lokasi yang sebelumnya ditempatin agen ke timestep baru
            for (int a = 0 ; a < n ; a++) {
                if (prev_state[a].location >= 0) sim_prev_dec[prev_state[a].location] = a; 
            }

            for (int id : ids) {
                if (next_state[id].location == -1) {
                    pibt(id, -1, 
                        prev_state, next_state, 
                        sim_prev_dec, sim_dec, 
                        sim_occupied, goal_per_agent, 
                        env, p);
                }
            }
            wppl.wppl_plan[t+1] = next_state;
        }

        ps.wait_map = wait_map_sim;
    }
}