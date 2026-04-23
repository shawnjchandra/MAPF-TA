#include "pibt-solver.h"
#include "mapf_utils.h"
#include "heuristics.h"

namespace CustomAlgo {

    void PIBTSolver::initialize(int preprocess_time_limit, SharedEnvironment* env) {
        int map_size   = env->map.size();
        int agent_size = env->num_of_agents;

        assert(env->num_of_agents != 0);

        auto& ps = env->planner_state;
        ps.w = max(3, ps.w);
        ps.h = max(1, ps.h);
        env->horizon = ps.h;
        env->m = 3;

        // Inisialisasi dan masukuin nilai penalty highway ke gcm (sebagai base val nya)
        ps.gcm.assign(map_size, {1.0f, 1.0f, 1.0f, 1.0f});
        for (auto& [key, val] : env->hpa_h.hw.r_e_hw) {
            int loc    = key.first;
            int orient = key.second;
            env->planner_state.gcm[loc][orient] = (float)env->c_penalty;
        }

        ps.wait_map.assign(map_size, {0, 0, 0, 0});
        ps.w_peak.resize(map_size);
        for (auto& wp : ps.w_peak) {
            wp.val       = 1.0f;
            wp.timestep  = 0;
        }

        ps.gcm_freq  = max(ps.gcm_freq, 5);
        ps.w_baseline = max(ps.w_baseline, 1.0f);
        ps.k_base    = max(ps.k_base, 0.1f);
        ps.max_degree = 4;

        ps.current_plans.assign(agent_size, {});

        //Bagian bawah, ngikutin framework (Untuk PIBT)

        p.resize(agent_size);
        decision.resize(map_size, -1);
        prev_states.resize(agent_size);
        next_states.resize(agent_size);
        decided.resize(agent_size, DCR({-1, DONE::DONE}));
        occupied.resize(map_size, false);
        checked.resize(agent_size, false);
        ids.resize(agent_size);

        goal_per_agent.assign(agent_size, -1);

        // Randomize initial priorities (sama dengan original framework)
        std::iota(ids.begin(), ids.end(), 0);
        mt1.seed(0);
        srand(0);

        std::shuffle(ids.begin(), ids.end(), mt1);
        for (int i = 0; i < (int)ids.size(); i++) {
            p[ids[i]] = ((double)(ids.size() - i)) / ((double)(ids.size() + 1));
        }
        p_copy = p;
    }

    void PIBTSolver::plan(int time_limit, std::vector<Action>& actions, SharedEnvironment* env) {
        TimePoint start_time = std::chrono::steady_clock::now();

        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
        if (pibt_time <= 0) pibt_time = 1;

        auto& ps = env->planner_state;

        // Reset tiap timestep state 
        prev_decision.assign(env->map.size(), -1);
        decision.assign(env->map.size(), -1);

        for (int a = 0; a < env->num_of_agents; a++) {

            assert(env->curr_states[a].location >= 0);

            prev_states[a] = env->curr_states[a];
            next_states[a] = State();  // location = -1

            prev_decision[env->curr_states[a].location] = a;

            // Inisialisasi decided pertama kali
            if (decided[a].loc == -1) {
                decided[a].loc   = env->curr_states[a].location;
                assert(decided[a].state == DONE::DONE);
            }

            // Cek apakah agen sudah sampai ke decided.loc
            if (prev_states[a].location == decided[a].loc) {
                decided[a].state = DONE::DONE;
            }

            // Kalau masih rotasi (NOT_DONE), klaim lokasi tujuan rotasi di decision
            if (decided[a].state == DONE::NOT_DONE) {
                decision[decided[a].loc] = a;
                next_states[a] = State(decided[a].loc, -1, -1);
            }

            // Update goal
            goal_per_agent[a] = env->goal_locations[a].empty() ? prev_states[a].location : env->goal_locations[a].front().first;

            // Update priorities (pakai dari original framework)
            bool is_at_goal = (prev_states[a].location == goal_per_agent[a]);

            if (env->goal_locations[a].empty()) {
                // Tidak ada goal = reset ke base priority (sama dengan original)
                p[a] = p_copy[a];
            } else if (is_at_goal) {
                // Sudah di goal = reset ke base priority
                p[a] = p_copy[a];
            } else {
                // Belum sampai = naikkan priority 
                p[a] += 1.0;
            }

            // Bonus priority untuk dead-end (sama dengan original framework)
            if (!env->goal_locations[a].empty() &&
                env->degree_map[env->curr_states[a].location] == 1) {
                p[a] += 10.0;
            }
        }

        // sort descending, utamain yang priority tinggi
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
            return p[a] > p[b];  
        });

        std::fill(occupied.begin(), occupied.end(), false);

        // Jalankan PIBT
        for (int i : ids) {
            // Skip agen yang masih dalam proses rotasi
            if (decided[i].state == DONE::NOT_DONE) continue;

            // Hanya proses agen yang belum punya rencana gerak
            if (next_states[i].location == -1) {
                assert(prev_states[i].location >= 0 &&
                       prev_states[i].location < (int)env->map.size());
                pibt(i, -1, prev_states, next_states,
                     prev_decision, decision, occupied, goal_per_agent, env, p);
            }
        }

        // Extract actions 
        actions.resize(env->num_of_agents);
        for (int id : ids) {
            // Bebaskan slot decision yang sudah diassign ke next_states
            if (next_states[id].location != -1)
                decision[next_states[id].location] = -1;

            // Update decided kalau agen gerak
            if (next_states[id].location >= 0)
                decided[id] = DCR({next_states[id].location, DONE::NOT_DONE});

            // safety net, seharusnya ga sih
            if (decided[id].loc == -1) {
                decided[id] = DCR({prev_states[id].location, DONE::DONE});
            }

            actions[id] = getAction(prev_states[id], decided[id].loc, env);
            checked[id] = false;
        }

        // moveCheck smua agen, validasi tidak ada forward move yang blokir satu sama lain 
        for (int id = 0; id < env->num_of_agents; id++) {
            if (!checked[id] && actions[id] == Action::FW) {
                moveCheck(id, checked, decided, actions, prev_decision);
            }
        }

        
        gcm_cooldown(env, env->curr_timestep);

        // GCM update 
        for (int a : ids) {
            gcm_update(env,
                       env->curr_states[a].location,
                       env->curr_states[a].orientation,
                       env->curr_timestep);
            // current_plans tidak dipakai (pure PIBT), tidak perlu di-erase
        }

        // Reset wait_map setiap gcm_freq timestep
        if (env->curr_timestep > 0 &&
            env->curr_timestep % env->planner_state.gcm_freq == 0) {
            for (auto& loc : env->planner_state.wait_map) loc.fill(0);
        }
    }

}