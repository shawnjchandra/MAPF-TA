#include "task_reassignment.h"
#include "scheduler.h"
#include "heuristics.h"
#include "congestion.h"

namespace CustomAlgo{
    
    void schedule_tasks(std::vector<int>& agt_dtr, SharedEnvironment* env, std::vector<int>& proposed_schedule, double gamma, std::unordered_set<int>& reserved_set) {
        auto t0 = std::chrono::steady_clock::now();
        auto ms_since = [&](std::chrono::steady_clock::time_point from) {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - from).count();
        };
                   
        std::unordered_map<int,int> task_to_agt;
        
        // CRITICAL FIX 1: Strictly bound to num_of_agents! 
        // proposed_schedule.size() can sometimes be larger than the actual agent count.
        for (int i = 0 ; i < env->num_of_agents ; i++) {
            if (proposed_schedule[i] != -1) task_to_agt[proposed_schedule[i]] = i;
        } 

        // PRE-FILTER: We no longer cache the owner's makespan here to avoid Ping-Pong loops.
        struct CandidateTask {
            int task_id;
            int loc;
        };
        std::vector<CandidateTask> candidate_tasks;
        candidate_tasks.reserve(env->task_pool.size());

        for (auto& pair : env->task_pool) {
            Task& t = pair.second;
            if (t.agent_assigned != -1) continue; 
            if (reserved_set.count(t.task_id) && task_to_agt.find(t.task_id) == task_to_agt.end()) continue;

            candidate_tasks.push_back({t.task_id, t.locations[t.idx_next_loc]});
        }

        std::vector<int> max_retry(env->num_of_agents, 0);
        int cols = env->cols; 
        
        for (size_t i = 0; i < agt_dtr.size(); ++i) {
            int agt_id = agt_dtr[i];
            
            // CRITICAL FIX 2: Absolute safety net against memory corruption
            if (agt_id < 0 || agt_id >= env->num_of_agents) continue;
            
            if(max_retry[agt_id] >= 5) continue;

            int best_task = -1;
            int best_makespan = INTERVAL_MAX;

            State agt_state = env->curr_states[agt_id];
            int agt_x = agt_state.location % cols;
            int agt_y = agt_state.location / cols;
        
            for (const auto& cand : candidate_tasks) {
                
                // CRITICAL FIX 3: Fetch the LIVE makespan. 
                // This guarantees that if Agent A steals a task, Agent B sees the new 
                // harder-to-beat makespan and doesn't endlessly steal it back.
                int owner_makespan = -1;
                if (env->makespan.count(cand.task_id)) {
                    owner_makespan = env->makespan[cand.task_id];
                }

                int task_x = cand.loc % cols;
                int task_y = cand.loc / cols;
                int manhattan_dist = std::abs(agt_x - task_x) + std::abs(agt_y - task_y);

                if (manhattan_dist >= best_makespan) continue;
                if (owner_makespan != -1 && manhattan_dist >= owner_makespan) continue;

                int curr_makespan = CustomAlgo::query_heuristic(env, agt_state.location, agt_state.orientation, cand.loc);

                if (curr_makespan >= INTERVAL_MAX) continue; 

                double delta = gamma * CustomAlgo::task_square_density(cand.loc, curr_makespan, env);
                int estimated_task_makespan = curr_makespan + (int) delta;

                if (estimated_task_makespan >= best_makespan ) continue;

                if (owner_makespan != -1 && estimated_task_makespan >= owner_makespan)         
                    continue;

                best_task = cand.task_id;
                best_makespan = estimated_task_makespan;
            }
           
            if (best_task == -1) continue;
            
            auto it = task_to_agt.find(best_task);
            if (it != task_to_agt.end()) {
                int old_agent = it->second;
                if (old_agent != agt_id) {
                    proposed_schedule[old_agent] = -1;
                    agt_dtr.push_back(old_agent); 
                }
            }
            
            proposed_schedule[agt_id] = best_task;
            env->makespan[best_task] = best_makespan;
            task_to_agt[best_task] = agt_id;

            max_retry[agt_id]++;
        }
        
        env->logger->log_info("4(DONE)=" + std::to_string(ms_since(t0)) + "ms", env->curr_timestep);
        env->logger->flush();
    }
}