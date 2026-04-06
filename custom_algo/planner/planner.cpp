#include "planner.h"
#include "pibt.h"
#include "const.h"
#include "drop-lns.h"
#include "guidance_cost_map.h"

namespace CustomAlgo{
    void planner_initialize(int preprocess_time_limit, SharedEnvironment* env) {
        int map_size = env->map.size();
        int agent_size = env->num_of_agents;

        auto& ps = env->planner_state;
        ps.w = max(10, ps.w);
        ps.h = max(5, ps.h);

        ps.gcm.assign(map_size, {1.0f, 1.0f, 1.0f, 1.0f});
        ps.wait_map.assign(map_size, {0,0,0,0});
        ps.w_peak.resize(map_size);
        for (auto& wp : ps.w_peak) {
            wp.val = 1.0f;
            wp.timestep = 0;
        }

        ps.gcm_freq = max(ps.gcm_freq, 5);
        ps.w_baseline = max(ps.w_baseline, 1.0f);
        ps.k_base = max(ps.k_base, 0.1f);
        ps.max_degree = 4;

        ps.current_plans.assign(agent_size, {});
        
        ps.priorities_base.resize(agent_size);
        ps.priorities.resize(agent_size);
        ps.decided.resize(agent_size, DCR({-1, DONE::DONE}));
        
        std::vector<int> ids(agent_size);

        //Randomizer priorities dari original framework
        std::iota(ids.begin(), ids.end(), 0);
        

        std::shuffle(ids.begin(), ids.end(), env->rng);
        for (int i = 0 ; i <ids.size() ; i++) {
            ps.priorities[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
        }
        ps.priorities_base = ps.priorities;

    }

    void planner_plan(int time_limit,vector<Action> & actions,  SharedEnvironment* env) {
        TimePoint start_time = std::chrono::steady_clock::now();

        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;

        TimePoint end_time = start_time + std::chrono::milliseconds(time_limit - pibt_time - LNS_TIMELIMIT_TOLERANCE);

        std::unordered_set<int> disabled = find_deadend_agents(env);
        
        //Fase 2: active agents
        std::vector<int> non_disabled_agents;
        for(int a = 0 ; a < env->num_of_agents ; a++) {
            if (!disabled.count(a)) non_disabled_agents.push_back(a); 
        }

        
        //Fase 4: PIBT + DROP-LNS
        if (env->curr_timestep % env->planner_state.h == 0) {
            //Fase 3: warm_start
            std::vector<std::vector<int>> warm_plan(env->num_of_agents);
            for (int a : non_disabled_agents) {
                const auto& prev = env->planner_state.current_plans[a];
    
                if (prev.size() > env->planner_state.h) 
                    warm_plan[a] = std::vector<int>(prev.begin() + env->planner_state.h, prev.end());
            }

            auto plans = init_pibt_window(non_disabled_agents,warm_plan, env);
            
            //Waktu random,ngikutin const.h
            int budget_time = max(10, time_limit - 20);
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count();
            int remaining_time = budget_time - elapsed_time;
    
            if (remaining_time > 15 ) {
                LNS lns;
                lns.P_min = plans;
                lns.P_min_soc = CustomAlgo::compute_soc(plans, non_disabled_agents, env);
                
                CustomAlgo::run_lns(lns, non_disabled_agents,env, remaining_time);
                {
                    std::lock_guard<std::mutex> lk(lns.mtx);
                    plans = lns.P_min;
                }
            }

            for (int a : non_disabled_agents)
                env->planner_state.current_plans[a] = plans[a];
        }
        
        //Extract actions
        actions.assign(env->num_of_agents, Action::W);
        for (int a : non_disabled_agents) {

            const auto& plan = env->planner_state.current_plans[a];
        
            if (plan.size() < 2) {
                actions[a] = Action::W;
                continue;
            }

            State cur(env->curr_states[a].location, -1, env->curr_states[a].orientation);

            actions[a] = CustomAlgo::getAction(cur, plan[1], env);

            if ( plan[1] == cur.location) {
                env-> planner_state.wait_map[cur.location][cur.orientation]++;
            }
        }

        for (int a : disabled) actions[a] = Action::W;

        
        
        //Fase 7 : GCM
        for (int a : non_disabled_agents) {
            // Shift plan 1 ke depan
            
            gcm_update(env, env->curr_states[a].location, env->curr_states[a].orientation, env->curr_timestep);
            
            env->planner_state.current_plans[a].erase(env->planner_state.current_plans[a].begin());
        }

        gcm_cooldown(env, env->curr_timestep);
        if (env->curr_timestep > 0 && env->curr_timestep % env->planner_state.gcm_freq == 0) {
            for (auto& loc : env->planner_state.wait_map) loc.fill(0); 
        }
    

    }

    std::unordered_set<int> find_deadend_agents(SharedEnvironment* env) {
        std::unordered_set<int> disabled;
        for (int a = 0 ; a < env->num_of_agents ; a++) {
            int loc = env->curr_states[a].location;
            if (env->degree_map[loc] == 1) disabled.insert(a);
        }

        return disabled;
    }


}