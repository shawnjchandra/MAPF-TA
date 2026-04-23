#include "wppl.h"
#include "pibt.h"

namespace CustomAlgo {
    void wppl_init(WPPLState& wppl, SharedEnvironment* env) {
        int w = env->planner_state.w;
        int n = env->num_of_agents;
        wppl.wppl_plan.assign(w + 1, std::vector<State>(n, State()));
        wppl.steps_since_plan = 0;
    }

    void run_windowed_pibt(WPPLState& wppl,
                           SharedEnvironment* env,
                           const std::vector<int>& ids,
                           const std::vector<int>& goal_per_agent,
                           std::vector<double>& p,
                           const std::vector<double>& p_copy,
                           const std::vector<DCR>& decided) {
        auto& ps = env->planner_state;
        int w = ps.w;
        int n = env->num_of_agents;
        int map_size = env->map.size();

        if ((int)wppl.wppl_plan.size() != w + 1)
            wppl.wppl_plan.assign(w + 1, std::vector<State>(n, State()));

        // Save state that gets mutated during simulation
        std::vector<double> p_backup = p;
        auto wait_map_backup = ps.wait_map;

        // Local mutable copy of ids for sorting
        std::vector<int> local_ids(ids.begin(), ids.end());

        // Sim-local conflict maps
        std::vector<int> sim_prev_dec(map_size, -1);
        std::vector<int> sim_dec(map_size, -1);
        std::vector<bool> sim_occupied(map_size, false);

        // Initialize plan[0] = current real state
        for (int a = 0; a < n; a++)
            wppl.wppl_plan[0][a] = env->curr_states[a];

        for (int t = 0; t < w; t++) {
            auto& prev_state = wppl.wppl_plan[t];
            std::vector<State> next_state(n, State());

            std::fill(sim_prev_dec.begin(), sim_prev_dec.end(), -1);
            std::fill(sim_dec.begin(),      sim_dec.end(),      -1);
            std::fill(sim_occupied.begin(), sim_occupied.end(), false);

            for (int a = 0; a < n; a++) {
                if (prev_state[a].location >= 0)
                    sim_prev_dec[prev_state[a].location] = a;

                // Per-timestep priority update (mirrors real plan())
                int sim_loc = prev_state[a].location;
                bool is_at_goal = (sim_loc == goal_per_agent[a]);

                if (env->goal_locations[a].empty() || is_at_goal) {
                    p[a] = p_copy[a];
                } else {
                    p[a] += 1.0;
                }

                if (!env->goal_locations[a].empty() &&
                    env->degree_map[sim_loc] == 1) {
                    p[a] += 10.0;
                }
            }

            // Re-sort every simulated timestep
            std::sort(local_ids.begin(), local_ids.end(),
                      [&](int a, int b) { return p[a] > p[b]; });

            // PIBT picks targets — no rotation reconciliation
            for (int id : local_ids) {
                if (next_state[id].location == -1) {
                    pibt(id, -1,
                         prev_state, next_state,
                         sim_prev_dec, sim_dec,
                         sim_occupied, goal_per_agent,
                         env, p);
                }
            }

            // Store as-is. The window is optimistic (assumes instant moves).
            // The real solver handles rotation via decided + getAction.
            wppl.wppl_plan[t + 1] = next_state;
        }

        // Restore real solver state
        p = p_backup;
        ps.wait_map = wait_map_backup;
    }
}